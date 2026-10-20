#include "RotationTools.h"
#include "aolion_driver.h"
#include "lowcontroller.h"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include "yaml-cpp/yaml.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <atomic>
#include <array>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Geometry>

// #define DEBUG_OBS
using namespace noetix;

//  全局常量
namespace
{
    constexpr int kNumJoints = 21;
    constexpr double kFallGravityZ = -0.3;
    constexpr double kDanceTiltGuard = -0.5;
    constexpr double kDanceTargetStep = 0.012;
    constexpr long kLoopPeriodUs = 2000;
    constexpr int kStandFrames = 1000;
    constexpr auto kStatusTimeout = std::chrono::milliseconds(150);
    constexpr double kWalkToDanceMaxCommand = 0.10;
    constexpr auto kDanceSettleWait = std::chrono::milliseconds(1200);
    constexpr auto kDanceStableDuration = std::chrono::milliseconds(500);
    constexpr double kDanceStableJointSpeed = 0.15; // rad/s
    constexpr double kDanceStableGyro = 0.25;       // rad/s
    constexpr double kDanceStableTiltRad = 0.25;    // pitch/roll absolute limit

    // 退出阻尼指令参数
    constexpr int kShutdownDampFrames = 10;
    constexpr auto kShutdownDampInterval = std::chrono::milliseconds(20);
} // namespace

//  退出信号处理：
static std::atomic<bool> g_running{true};
static void signalHandler(int /*signum*/)
{                                                      // ctrl + c
    g_running.store(false, std::memory_order_release); // 异步信号安全：仅设置原子标志，不进行任何 IO
}

//  全局状态
LowController *ctrl = nullptr; 
AoLionDriver aoliondriver;

std::vector<std::string> jointNames{
    "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
    "leg_r2_joint", "arm_l1_joint", "arm_r1_joint", "leg_l3_joint",
    "leg_r3_joint", "arm_l2_joint", "arm_r2_joint", "leg_l4_joint",
    "leg_r4_joint", "arm_l3_joint", "arm_r3_joint", "leg_l5_joint",
    "leg_r5_joint", "arm_l4_joint", "arm_r4_joint", "leg_l6_joint",
    "leg_r6_joint"};

std::string modelname;
int64_t count;
RobotCfg robotconfig;

Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
std::shared_ptr<Ort::Env> onnxEnvPrt_;
std::unique_ptr<Ort::Session> policySessionPtr;
std::vector<const char *> policyInputNames_;
std::vector<const char *> policyOutputNames_;
std::vector<Ort::AllocatedStringPtr> policyInputNodeNameAllocatedStrings;
std::vector<Ort::AllocatedStringPtr> policyOutputNodeNameAllocatedStrings;
std::vector<std::vector<int64_t>> policyInputShapes_;
std::vector<std::vector<int64_t>> policyOutputShapes_;
vector_t lastActions_;
vector_t defaultJointAngles_;
int actuatedDofNum_ = 0;
int actionsSize_ = 0;
int observationSize_ = 0;
int stackSize_ = 0; 
float scalez = 1.0f, scalex = 1.0f, scaley = 1.0f;
std::vector<tensor_element_t> actions_;
std::vector<tensor_element_t> policyObservations_;
Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;

bool g_isfirstRecObs = true;
bool g_isfirstCompAct = true;
vector_t command_;
Proprioception propri_;
joydata remote_data{};
double standPercent = 0.0;
scalar_t standDuration = kStandFrames;

WorkMode mode_ = WorkMode::DEFAULT;
bool isChangeMode_ = false;
bool startcontrol = false;
int initfinish = 0;

vector_t lieJointAngles_;
vector_t standJointAngles_;
vector_t currentJointAngles_;
vector_t standStartAngles_;

bool found_joint_names = false;
bool found_default_joint_pos = false;
bool found_action_scale = false;
bool found_joint_stiffness = false;
bool found_joint_damping = false;

std::vector<double> action_scale;
std::vector<double> joint_stiffness;
std::vector<double> joint_damping;
std::vector<double> default_joint_pos;
std::vector<std::string> joint_names;

float current_vel_limit_ = 1.5f;
std::array<double, kNumJoints> g_lastWalkTarget{}; // WALK 最近一次发送给电机的目标
bool g_lastWalkTargetValid = false;

std::array<double, kNumJoints> g_walkResumeFromTarget{};
bool g_walkResumeBlendActive = false;
std::chrono::steady_clock::time_point g_walkResumeBlendStart{};
constexpr double kWalkResumeBlendSeconds = 0.85;
constexpr double kWalkResumeArmBlendSeconds = 1.50;

static RobotHardwareStatus latest_hw_status{};
static std::mutex hardware_mutex;
static std::atomic<bool> hardware_received{false};
static std::chrono::steady_clock::time_point last_hardware_rx{};

// 请求切舞后，WALK 策略持续输出，直到姿态和关节速度稳定。
static bool g_danceStartPending = false;
static std::chrono::steady_clock::time_point g_danceRequestTime{};
static std::chrono::steady_clock::time_point g_danceStableSince{};
static bool g_danceStableTimerStarted = false;

static void cancelDanceStartRequest(const char *reason)
{
    if (g_danceStartPending)
        std::cout << "[DANCE] pending start cancelled: " << reason << "\n";
    g_danceStartPending = false;
    g_danceStableTimerStarted = false;
}

static bool hardwareFresh()
{
    std::lock_guard<std::mutex> lock(hardware_mutex);
    return hardware_received.load(std::memory_order_acquire) &&
           (std::chrono::steady_clock::now() - last_hardware_rx < kStatusTimeout);
}

static std::array<MotorState, kNumJoints> getMotorSnapshot()
{
    std::lock_guard<std::mutex> lock(hardware_mutex);
    return latest_hw_status.motor_data;
}

//  工具函数
std::vector<std::string> SplitString(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        auto start = token.find_first_not_of(" \t\r\n");
        if (start != std::string::npos)
        {
            auto end = token.find_last_not_of(" \t\r\n");
            tokens.push_back(token.substr(start, end - start + 1));
        }
    }
    return tokens;
}

std::vector<double> SplitString2Double(const std::string &str, char delimiter)
{
    std::vector<double> values;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter))
    {
        if (!token.empty())
        {
            try
            {
                values.push_back(std::stod(token));
            }
            catch (const std::exception &)
            {
                values.push_back(0.0);
            }
        }
    }
    return values;
}

void init()
{
    mode_ = WorkMode::DEFAULT;
    aoliondriver.init("/dev/input/js0", 115200);

    // 注册硬件状态回调
    ctrl->subscribe_robot_hardware_status(
        [](const RobotHardwareStatus &status)
        {
            std::lock_guard<std::mutex> lock(hardware_mutex);
            latest_hw_status = status;
            last_hardware_rx = std::chrono::steady_clock::now();
            hardware_received.store(true, std::memory_order_release);
        });

    actuatedDofNum_ = kNumJoints;
    standDuration = kStandFrames;
    standPercent = 0.0;

    lieJointAngles_.resize(actuatedDofNum_);
    standJointAngles_.resize(actuatedDofNum_);
    currentJointAngles_.resize(actuatedDofNum_);
    standStartAngles_.resize(actuatedDofNum_);

    lieJointAngles_.setZero();
    // leg_r2, arm_l1, arm_r1,
    //             leg_l3, leg_r3, arm_l2, arm_r2, leg_l4, leg_r4, arm_l3,
    //             arm_r3, leg_l5, leg_r5, arm_l4, arm_r4, leg_l6, leg_r6
    standJointAngles_ << -0.1495, -0.1495, // leg_l1, leg_r1
        0.0,                               // waist_1
        0.0, 0.0,                          // leg_l2, leg_r2
        0.0, 0.0,                          // arm_l1, arm_r1
        0.0, 0.0,                          // leg_l3, leg_r3
        0.2618, -0.2618,                   // arm_l2, arm_r2
        0.3215, 0.3215,                    // leg_l4, leg_r4
        0.0, 0.0,                          // arm_l3, arm_r3
        -0.1720, -0.1720,                  // leg_l5, leg_r5
        0.0, 0.0,                          // arm_l4, arm_r4
        0.0, 0.0;                          // leg_l6, leg_r6
    currentJointAngles_.setZero();
    standStartAngles_ = standJointAngles_;

    command_.resize(3);
    command_.setZero();
}

void setparameter(Command &cmd, bool *isfirst)
{
    if (isfirst && *isfirst)
    {
        g_isfirstRecObs = true;
        g_isfirstCompAct = true;
        *isfirst = false;
    }
    command_[0] = cmd.x;
    command_[1] = cmd.y;
    command_[2] = cmd.yaw;
}

//  状态估计
bool updateStateEstimation()
{
    vector_t jointPos = vector_t::Zero(actuatedDofNum_);
    vector_t jointVel = vector_t::Zero(actuatedDofNum_);
    quaternion_t quat;
    vector3_t angularVel, linearAccel;

    RobotHardwareStatus snapshot;
    {
        std::lock_guard<std::mutex> lock(hardware_mutex);
        if (!hardware_received.load(std::memory_order_acquire) ||
            std::chrono::steady_clock::now() - last_hardware_rx >= kStatusTimeout)
            return false;
        snapshot = latest_hw_status;
    }
    const auto &joint_state = snapshot.motor_data;

    // 按 jointNames 顺序读取关节状态
    for (int i = 0; i < actuatedDofNum_; ++i)
    {
        int hwIdx = ctrl->getJointsIndex(jointNames[i]);
        if (hwIdx >= 0 && hwIdx < kNumJoints)
        {
            jointPos(i) = joint_state[hwIdx].pos;
            jointVel(i) = joint_state[hwIdx].vel;
        }
        else
        {
            return false;
        }
    }

    const auto &imudata = snapshot.imu_data;
    for (int i = 0; i < 4; ++i)
        quat.coeffs()(i) = imudata.ori[i];
    for (int i = 0; i < 3; ++i)
    {
        angularVel(i) = imudata.angular_vel[i];
        linearAccel(i) = imudata.linear_acc[i];
    }

    propri_.jointPos = jointPos;
    propri_.jointVel = jointVel;
    propri_.baseAngVel = angularVel;

    if (!std::isfinite(quat.norm()) || quat.norm() < 0.5)
        return false;
    quat.normalize();
    propri_.robot_quat_ = quat;

    vector3_t gravityVector(0, 0, -1);
    vector3_t zyx = quatToZyx(quat);
    propri_.baseEulerXyz[0] = zyx[2];
    propri_.baseEulerXyz[1] = zyx[1];
    propri_.baseEulerXyz[2] = zyx[0];
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    propri_.projectedGravity = inverseRot * gravityVector;
    return true;
}

//  基础模式
void handleDefautMode()
{
    std::array<MotorCmd, kNumJoints> motorcmd{};
    for (int j = 0; j < actuatedDofNum_; j++)
    {
        int hwIdx = ctrl->getJointsIndex(jointNames[j]);
        if (hwIdx >= 0 && hwIdx < kNumJoints)
        {
            motorcmd[hwIdx].pos = 0;
            motorcmd[hwIdx].vel = 0;
            motorcmd[hwIdx].kp = 0;
            motorcmd[hwIdx].kd = 0.1;
            motorcmd[hwIdx].tau = 0;
            motorcmd[hwIdx].motor_id = hwIdx;
        }
    }
    ctrl->set_joint(motorcmd);
}

void handleStandMode()
{
    std::array<MotorCmd, kNumJoints> motorcmd{};
    const bool finished = (standPercent >= 1.0);
    for (int j = 0; j < actuatedDofNum_; j++)
    {
        scalar_t pos_des;
        if (finished)
        {
            pos_des = standJointAngles_[j];
        }
        else
        {
            pos_des = standStartAngles_[j] * (1.0 - standPercent) +
                      standJointAngles_[j] * standPercent;
        }
        int hwIdx = ctrl->getJointsIndex(jointNames[j]);
        if (!std::isfinite(pos_des) || hwIdx < 0 || hwIdx >= kNumJoints)
            continue;

        motorcmd[hwIdx].pos = pos_des;
        motorcmd[hwIdx].vel = 0;
        motorcmd[hwIdx].kp = 10.0;
        motorcmd[hwIdx].kd = 0.5;
        motorcmd[hwIdx].tau = 0;
        motorcmd[hwIdx].motor_id = hwIdx;
    }
    ctrl->set_joint(motorcmd);
    if (!finished)
    {
        standPercent += 1.0 / standDuration;
        standPercent = std::min(standPercent, scalar_t(1));
    }
}

void handleLieMode()
{
    std::array<MotorCmd, kNumJoints> motorcmd{};
    const bool finished = (standPercent >= 1.0);
    for (int j = 0; j < actuatedDofNum_; j++)
    {
        scalar_t pos_des;
        if (finished)
        {
            pos_des = lieJointAngles_[j];
        }
        else
        {
            pos_des = currentJointAngles_[j] * (1.0 - standPercent) +
                      lieJointAngles_[j] * standPercent;
        }
        int hwIdx = ctrl->getJointsIndex(jointNames[j]);
        if (hwIdx >= 0 && hwIdx < kNumJoints)
        {
            motorcmd[hwIdx].pos = pos_des;
            motorcmd[hwIdx].vel = 0;
            motorcmd[hwIdx].kp = 10.0;
            motorcmd[hwIdx].kd = 0.5;
            motorcmd[hwIdx].tau = 0;
            motorcmd[hwIdx].motor_id = hwIdx;
        }
    }
    ctrl->set_joint(motorcmd);
    if (!finished)
    {
        standPercent += 1.0 / standDuration;
        standPercent = std::min(standPercent, scalar_t(1));
    }
}

// =====================================================================
//  走路策略
// =====================================================================
void computeActions()
{
    std::vector<Ort::Value> policyInputValues;
    policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
        memoryInfo, policyObservations_.data(), policyObservations_.size(),
        policyInputShapes_[0].data(), policyInputShapes_[0].size()));
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = policySessionPtr->Run(
        runOptions, policyInputNames_.data(), policyInputValues.data(), 1,
        policyOutputNames_.data(), 1);

#ifdef DEBUG_OBS
    if (g_isfirstCompAct)
    {
        for (int i = 0; i < (int)policyObservations_.size(); ++i)
        {
            std::cout << policyObservations_[i] << " ";
            if ((i + 1) % observationSize_ == 0)
                std::cout << std::endl;
        }
        g_isfirstCompAct = false;
    }
#endif

    for (int i = 0; i < actionsSize_; i++)
    {
        actions_[i] =
            *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
}

void computeObservation()
{
    vector_t command(3);

    double cmd_x = command_[0] * scalex;
    double cmd_y = command_[1] * scaley;
    double cmd_z = command_[2] * scalez;

    if (cmd_x < -0.5)
        cmd_x = -0.5;
    if (cmd_x < 0.3 && cmd_x >= 0)
        cmd_x = 0.0;
    if (std::abs(cmd_z) < 0.3)
        cmd_z = 0.0;
    if (cmd_x > current_vel_limit_)
        cmd_x = current_vel_limit_;
    if (cmd_x > 0.6 && cmd_z > 1.0)
        cmd_z = 1.0;
    if (cmd_x > 0.6 && cmd_z < -1.0)
        cmd_z = -1.0;

    command[0] = cmd_x;
    command[1] = cmd_y;
    command[2] = cmd_z;

    vector_t actions(lastActions_);
    vector_t proprioObs(observationSize_);

    proprioObs << command,
        propri_.baseAngVel,
        propri_.baseEulerXyz(0),
        propri_.baseEulerXyz(1),
        (propri_.jointPos - defaultJointAngles_),
        propri_.jointVel,
        actions;

    if (g_isfirstRecObs)
    {
        for (int i = observationSize_ - actionsSize_; i < observationSize_; i++)
        {
            proprioObs(i, 0) = 0.0;
        }
        for (int i = 0; i < stackSize_; i++)
        {
            proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) =
                proprioObs.cast<tensor_element_t>();
        }
        g_isfirstRecObs = false;
        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
    }

    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_)
            .eval();
    proprioHistoryBuffer_.tail(observationSize_) =
        proprioObs.cast<tensor_element_t>();

    for (size_t i = 0; i < (size_t)(observationSize_ * stackSize_); i++)
    {
        policyObservations_[i] =
            static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
    }

    scalar_t obsMin = -robotconfig.clipObs;
    scalar_t obsMax = robotconfig.clipObs;
    std::transform(policyObservations_.begin(), policyObservations_.end(),
                   policyObservations_.begin(),
                   [obsMin, obsMax](scalar_t x)
                   {
                       return std::max(obsMin, std::min(obsMax, x));
                   });
}

bool handleUserMode()
{
    if (!updateStateEstimation())
        return false;

    if (propri_.projectedGravity(2) >= kFallGravityZ)
    {
#ifdef DEBUG_OBS
        printf("[WALK] Fall Protection Triggered\n");
#endif
        mode_ = WorkMode::DEFAULT;
        return true;
    }

    if (count % robotconfig.controlCfg.decimation == 0)
    {
        count = 0;
        computeObservation();
        computeActions();
        scalar_t actionMin = -robotconfig.clipActions;
        scalar_t actionMax = robotconfig.clipActions;
        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                       [actionMin, actionMax](scalar_t x)
                       {
                           return std::max(actionMin, std::min(actionMax, x));
                       });
    }

    std::array<MotorCmd, kNumJoints> motorcmd{};
    for (int i = 0; i < actionsSize_; i++)
    {
        if (i >= (int)jointNames.size())
            break;
        int hwIdx = ctrl->getJointsIndex(jointNames[i]);
        scalar_t pos_des =
            actions_[i] * action_scale[i] + defaultJointAngles_(i);

        // Smooth DANCE -> WALK handoff
        if (g_walkResumeBlendActive)
        {
            const double duration =
                (i == 5 || i == 6 || i == 9 || i == 10 ||
                 i == 13 || i == 14 || i == 17 || i == 18)
                    ? kWalkResumeArmBlendSeconds
                    : kWalkResumeBlendSeconds;
            const double elapsed = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() - g_walkResumeBlendStart)
                                       .count();
            const double u = std::clamp(elapsed / duration, 0.0, 1.0);
            const double smooth = u * u * (3.0 - 2.0 * u); // smoothstep
            pos_des = (1.0 - smooth) * g_walkResumeFromTarget[i] +
                      smooth * pos_des;
        }

        if (hwIdx >= 0 && hwIdx < kNumJoints)
        {
            motorcmd[hwIdx].pos = pos_des;
            motorcmd[hwIdx].vel = 0;
            motorcmd[hwIdx].kp = joint_stiffness[i];
            motorcmd[hwIdx].kd = joint_damping[i];
            motorcmd[hwIdx].tau = 0;
            motorcmd[hwIdx].motor_id = hwIdx;
        }
        lastActions_(i) = actions_[i];
    }

    // 保存最近一次 WALK 电机目标，用于分析 WALK -> DANCE 切换
    static int walkLogCounter = 0;

    if (++walkLogCounter % 250 == 0)
    {
        for (int i = 0; i < kNumJoints; ++i)
        {
            int hw = ctrl->getJointsIndex(jointNames[i]);
            if (hw < 0 || hw >= kNumJoints)
                continue;
#ifdef DEBUG_OBS
            std::cout << "joint[" << i << "] "
                      << jointNames[i]
                      << " target=" << motorcmd[hw].pos
                      << " measured=" << propri_.jointPos[i]
                      << " delta="
                      << motorcmd[hw].pos - propri_.jointPos[i]
                      << std::endl;
#endif
        }
    }

    bool valid = true;
    for (int i = 0; i < kNumJoints; ++i)
    {
        int hw = ctrl->getJointsIndex(jointNames[i]);
        if (hw < 0 || hw >= kNumJoints)
        {
            valid = false;
            break;
        }

        g_lastWalkTarget[i] = motorcmd[hw].pos;
    }

    g_lastWalkTargetValid = valid;
    ctrl->set_joint(motorcmd);
    if (g_walkResumeBlendActive &&
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - g_walkResumeBlendStart)
                .count() >= kWalkResumeArmBlendSeconds)
    {
        g_walkResumeBlendActive = false;
        std::cout << "[DANCE] smooth DANCE -> WALK target blend complete\n";
    }

    count++;
    return true;
}

// =====================================================================
//  舞蹈策略
// =====================================================================
namespace dance
{

    constexpr int N = 21;
    constexpr int STEP = 114;
    constexpr int HISTORY = 5;
    constexpr int OBS = STEP * HISTORY;
    constexpr int DECIMATION = 10;
    static_assert(21 + 21 + 6 + 3 + 21 + 21 + 21 == STEP, "dance STEP layout mismatch");
    std::array<double, N> kp{}, kd{}, defaults{}, scale{};
    const std::array<std::string, N> expectedDanceNames{{"l_leg_pitch_joint", "r_leg_pitch_joint", "waist_yaw_joint",
                                                         "l_leg_roll_joint", "r_leg_roll_joint", "l_arm_pitch_joint",
                                                         "r_arm_pitch_joint", "l_leg_yaw_joint", "r_leg_yaw_joint",
                                                         "l_arm_roll_joint", "r_arm_roll_joint", "l_knee_pitch_joint",
                                                         "r_knee_pitch_joint", "l_arm_yaw_joint", "r_arm_yaw_joint",
                                                         "l_ankle_pitch_joint", "r_ankle_pitch_joint", "l_elbow_pitch_joint",
                                                         "r_elbow_pitch_joint", "l_ankle_roll_joint", "r_ankle_roll_joint"}};

    struct Frame
    {
        std::array<double, N> pos{}, vel{};
        Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    };

    std::vector<Frame> frames;
    std::unique_ptr<Ort::Session> session;
    std::vector<Ort::AllocatedStringPtr> inputOwners, outputOwners;
    const char *inputName = nullptr;
    const char *outputName = nullptr;
    std::array<float, OBS> obs{};
    std::array<float, N> actions{};
    std::array<double, N> lastTarget{};
    Eigen::Quaterniond initToWorld = Eigen::Quaterniond::Identity();
    bool ready = false, active = false;
    bool historyReady = false, targetReady = false;
    size_t frameIndex = 0;
    int ticks = 0;
    bool hardwareEnabled = false;

    // DANCE 接管后直接运行策略。
    enum class Phase
    {
        IDLE,
        PLAYING
    };
    Phase phase = Phase::IDLE;
    Eigen::Quaterniond yawQuat(const Eigen::Quaterniond &q)
    {
        double y = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                              1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        return Eigen::Quaterniond(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
    }

    Eigen::Quaterniond robotTorsoQuat()
    {
        Eigen::Quaterniond base = propri_.robot_quat_.normalized();
        return (base * Eigen::Quaterniond(
                           Eigen::AngleAxisd(propri_.jointPos[2], Eigen::Vector3d::UnitZ())))
            .normalized();
    }

    bool validateJointMapping()
    {
        if (jointNames.size() != (size_t)N)
            return false;
        const std::array<std::string, N> hardwareExpected{{"leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
                                                           "leg_r2_joint", "arm_l1_joint", "arm_r1_joint", "leg_l3_joint",
                                                           "leg_r3_joint", "arm_l2_joint", "arm_r2_joint", "leg_l4_joint",
                                                           "leg_r4_joint", "arm_l3_joint", "arm_r3_joint", "leg_l5_joint",
                                                           "leg_r5_joint", "arm_l4_joint", "arm_r4_joint", "leg_l6_joint",
                                                           "leg_r6_joint"}};
        std::array<bool, N> seen{};
        for (int i = 0; i < N; ++i)
        {
            if (jointNames[i] != hardwareExpected[i])
                return false;
            int hw = ctrl->getJointsIndex(jointNames[i]);
            if (hw < 0 || hw >= N || seen[hw])
                return false;
            seen[hw] = true;
        }
        return true;
    }

    bool loadConfig(const std::string &yamlPath)
    {
        try
        {
            YAML::Node root = YAML::LoadFile(yamlPath);
            const auto node = root["dance"];
            if (!node)
                throw std::runtime_error("missing dance YAML section");

            const auto names = node["joint_names"].as<std::vector<std::string>>();
            if (names.size() != (size_t)N)
                throw std::runtime_error("dance joint_names must have 21 entries");
            for (int i = 0; i < N; ++i)
            {
                if (names[i] != expectedDanceNames[i])
                    throw std::runtime_error(
                        "dance joint order mismatch at index " + std::to_string(i) +
                        " (got " + names[i] + ", want " + expectedDanceNames[i] + ")");
            }

            auto copy21 = [&](const char *key, std::array<double, N> &dest)
            {
                auto v = node[key].as<std::vector<double>>();
                if (v.size() != (size_t)N)
                    throw std::runtime_error(std::string(key) + " must have 21 values");
                for (int i = 0; i < N; ++i)
                {
                    if (!std::isfinite(v[i]))
                        throw std::runtime_error(std::string(key) + " contains nonfinite value");
                    dest[i] = v[i];
                }
            };
            copy21("joint_stiffness", kp);
            copy21("joint_damping", kd);
            copy21("default_joint_pos", defaults);
            copy21("action_scale", scale);

            if (node["control"]["decimation"].as<int>() != DECIMATION ||
                std::abs(node["control"]["cycle_time"].as<double>() - 0.02) > 1e-8 ||
                node["size"]["actions_size"].as<int>() != N ||
                node["size"]["observations_size"].as<int>() != STEP ||
                node["size"]["stack_size"].as<int>() != HISTORY)
                throw std::runtime_error("dance inference dimensions or timing mismatch");

            for (int i = 0; i < N; ++i)
                if (kp[i] < 0 || kd[i] < 0 || scale[i] <= 0)
                    throw std::runtime_error("invalid dance gains or action_scale");

            if (!validateJointMapping())
                return false;

            if (!default_joint_pos.empty())
            {
                double max_diff = 0.0;
                int max_idx = -1;
                for (int i = 0; i < N && i < (int)default_joint_pos.size(); ++i)
                {
                    double d = std::abs(defaults[i] - default_joint_pos[i]);
                    if (d > max_diff)
                    {
                        max_diff = d;
                        max_idx = i;
                    }
                }
                if (max_diff > 1e-3)
                {
                    std::cerr << "[DANCE] WARNING: default_joint_pos differs from "
                                 "walk policy, max_diff="
                              << max_diff
                              << " at idx=" << max_idx
                              << " (dance=" << defaults[max_idx]
                              << ", walk=" << default_joint_pos[max_idx] << ")\n";
                }
            }
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[DANCE] config failed: " << e.what() << "\n";
            return false;
        }
    }

    bool load(const std::string &model, const std::string &jsonPath)
    {
        try
        {
            std::cerr << "[DANCE DEBUG] Opening JSON: " << jsonPath << std::endl;
            std::ifstream f(jsonPath);
            if (!f)
                throw std::runtime_error("cannot open motion JSON: " + jsonPath);
            nlohmann::json j;
            f >> j;
            std::cerr << "[DANCE DEBUG] JSON parsed" << std::endl;
            const auto &p = j.at("joint_pos");
            const auto &v = j.at("joint_vel");
            const auto &q = j.at("body_quat_w");

            if (!p.is_array() || p.empty() || p.size() != v.size() || p.size() != q.size())
                throw std::runtime_error("motion array sizes mismatch");

            frames.clear();
            std::cerr << "[DANCE DEBUG] Frame count: " << p.size() << std::endl;
            frames.reserve(p.size());
            for (size_t t = 0; t < p.size(); ++t)
            {
                if (p[t].size() != (size_t)N || v[t].size() != (size_t)N ||
                    q[t].size() != 4)
                    throw std::runtime_error("bad motion frame " + std::to_string(t));
                Frame fr;
                for (int i = 0; i < N; ++i)
                {
                    fr.pos[i] = p[t][i].get<double>();
                    fr.vel[i] = v[t][i].get<double>();
                    if (!std::isfinite(fr.pos[i]) || !std::isfinite(fr.vel[i]))
                        throw std::runtime_error("nonfinite motion joint");
                }
                fr.quat = Eigen::Quaterniond(q[t][0].get<double>(),
                                             q[t][1].get<double>(),
                                             q[t][2].get<double>(),
                                             q[t][3].get<double>());
                if (!std::isfinite(fr.quat.norm()) || fr.quat.norm() < 0.5)
                    throw std::runtime_error("invalid motion quaternion");
                fr.quat.normalize();
                frames.push_back(fr);
            }

            // 创建 ONNX Session 配置
            Ort::SessionOptions opt;
            opt.SetInterOpNumThreads(1);
            session = std::make_unique<Ort::Session>(*onnxEnvPrt_, model.c_str(), opt); // 创建舞蹈 ONNX Session

            // 检查模型输入输出数量
            auto inputCount = session->GetInputCount();
            auto outputCount = session->GetOutputCount();
            std::cerr << "[DANCE DEBUG] InputCount=" << inputCount
                      << " OutputCount=" << outputCount << std::endl;

            if (inputCount != 1 || outputCount != 1)
            {
                throw std::runtime_error("expected one input/output");
            }

            //  检查输入类型
            // 保留 TypeInfo 对象，防止临时对象提前析构
            auto inputTypeInfo = session->GetInputTypeInfo(0);
            auto outputTypeInfo = session->GetOutputTypeInfo(0);

            auto in = inputTypeInfo.GetTensorTypeAndShapeInfo();
            auto out = outputTypeInfo.GetTensorTypeAndShapeInfo();
            std::cerr << "[DANCE DEBUG] OutputTypeInfo OK" << std::endl;

            // 检查输入维度
            auto a = in.GetShape();
            for (auto x : a)
                std::cerr << " " << x;
            std::cerr << std::endl;

            // 检查输出维度
            auto b = out.GetShape();
            std::cerr << "[DANCE DEBUG] Output shape:";
            for (auto x : b)
                std::cerr << " " << x;
            std::cerr << std::endl;
            if (in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                a.size() != 2 || (a[0] != 1 && a[0] != -1) || a[1] != OBS ||
                b.size() != 2 || (b[0] != 1 && b[0] != -1) || b[1] != N)
                throw std::runtime_error("expected float32 [1,570] -> [1,21]");

            Ort::AllocatorWithDefaultOptions allocator;
            inputOwners.clear();
            outputOwners.clear();

            // 获取输入名称
            inputOwners.push_back(session->GetInputNameAllocated(0, allocator));
            //  获取输出名称
            outputOwners.push_back(session->GetOutputNameAllocated(0, allocator));
            std::cerr << "[DANCE DEBUG] Output name allocated" << std::endl;

            inputName = inputOwners[0].get();
            outputName = outputOwners[0].get();

            std::cerr << "[DANCE DEBUG] InputName=" << inputName
                      << " OutputName=" << outputName
                      << std::endl;
            std::cout << "[DANCE] loaded " << frames.size()
                      << " frames, model " << model << "\n";

            ready = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[DANCE] load failed: " << e.what() << "\n";
            ready = false;
            session.reset();
            frames.clear();
            return false;
        }
    }

    void stop(const char *reason)
    {
        std::cout << "[DANCE] stop requested: " << reason << "\n";
        if (phase == Phase::IDLE && !active)
            return;

        // 正常结束：不发送阻尼停机指令，也不执行 EXITING 插值。
        // 只在硬件反馈仍然新鲜且状态估计有效时，直接把控制权交回 WALK ONNX。
        const bool stateOk = hardwareFresh() && updateStateEstimation();

        if (stateOk && targetReady && startcontrol)
        {
            for (int i = 0; i < N; ++i)
                g_walkResumeFromTarget[i] = lastTarget[i];
            g_walkResumeBlendStart = std::chrono::steady_clock::now();
            g_walkResumeBlendActive = true;
            std::cout << "[DANCE] enabling live WALK-target blend; arms blend over "
                      << kWalkResumeArmBlendSeconds << " s\n";
        }
        else
        {
            g_walkResumeBlendActive = false;
        }

        active = false;
        phase = Phase::IDLE;
        targetReady = false;
        historyReady = false;
        frameIndex = 0;
        ticks = 0;
        obs.fill(0.0f);
        actions.fill(0.0f);
        g_danceStartPending = false;
        g_danceStableTimerStarted = false;

        if (stateOk && startcontrol)
        {
            mode_ = WorkMode::USERMODE;
            // 重置 WALK 的观测/动作历史，让 WALK ONNX 从最新实测状态重新建立历史。
            g_isfirstRecObs = true;
            g_isfirstCompAct = true;
            count = 0;
            lastActions_.setZero();
            std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
            proprioHistoryBuffer_.setZero();
            std::cout << "[DANCE] returned directly to WALK ONNX; no damping stop, no EXITING interpolation\n";
        }
        else
        {
            // 反馈失效时不运行 WALK 推理，避免使用过期状态生成动作。 此分支不主动发送阻尼指令。
            mode_ = WorkMode::DEFAULT;
            std::cerr << "[DANCE] WALK resume unavailable: hardware state stale/invalid; no damping command sent\n";
        }
    }

    bool start()
    {
        if (!ready || !hardwareEnabled || !startcontrol)
        {
            std::cerr << "[DANCE] start rejected: ready=" << ready
                      << " hardwareEnabled=" << hardwareEnabled
                      << " startcontrol=" << startcontrol << "\n";
            return false;
        }
        if (mode_ != WorkMode::USERMODE)
        {
            std::cerr << "[DANCE] start rejected: enter WALK mode first\n";
            return false;
        }
        if (!updateStateEstimation() ||
            propri_.projectedGravity.z() >= kDanceTiltGuard ||
            std::abs(propri_.baseEulerXyz[0]) >= kDanceStableTiltRad ||
            std::abs(propri_.baseEulerXyz[1]) >= kDanceStableTiltRad)
        {
            std::cerr << "[DANCE] start rejected: invalid or tilted body\n";
            return false;
        }
        if (std::abs(remote_data.axes[0]) > kWalkToDanceMaxCommand ||
            std::abs(remote_data.axes[1]) > kWalkToDanceMaxCommand)
        {
            std::cerr << "[DANCE] start rejected: release joystick axes\n";
            return false;
        }
        for (int i = 0; i < N; ++i)
        {
            if (!std::isfinite(propri_.jointPos[i]) ||
                !std::isfinite(propri_.jointVel[i]) ||
                std::abs(propri_.jointVel[i]) > kDanceStableJointSpeed)
            {
                std::cerr << "[DANCE] start rejected: joint not stable, index="
                          << i << " velocity=" << propri_.jointVel[i] << "\n";
                return false;
            }
        }
        if (frames.empty())
        {
            std::cerr << "[DANCE] start rejected: no motion frames\n";
            return false;
        }

        const auto torso = robotTorsoQuat();
        initToWorld = yawQuat(torso) * yawQuat(frames[0].quat).conjugate();

        double maxDelta = 0.0;
        int maxDeltaJoint = -1;
        std::cout << "\n= DIRECT WALK -> DANCE CHECK =\n";
        for (int i = 0; i < N; ++i)
        {
            const double delta = frames[0].pos[i] - propri_.jointPos[i];
            if (std::abs(delta) > maxDelta)
            {
                maxDelta = std::abs(delta);
                maxDeltaJoint = i;
            }
#ifdef DEBUG_OBS
            std::cout << "joint[" << i << "] " << jointNames[i]
                      << " measured=" << propri_.jointPos[i]
                      << " dance_frame0=" << frames[0].pos[i]
                      << " delta=" << delta << "\n";
#endif
        }
       // std::cout << "[DANCE] max frame0 pose difference=" << maxDelta << " rad at joint=" << maxDeltaJoint << "\n";

        // 清空 DANCE 历史并直接进入 PLAYING；首个电机目标仍从实测姿态开始，每控制周期受 kDanceTargetStep 限幅。
        obs.fill(0.0f);
        actions.fill(0.0f);
        lastTarget.fill(0.0);
        frameIndex = 0;
        ticks = 0;
        historyReady = false;
        targetReady = false;
        phase = Phase::PLAYING;
        active = true;
        g_danceStartPending = false;
        g_danceStableTimerStarted = false;
        return true;
    }

    void infer()
    {
        if (frameIndex >= frames.size())
        {
            stop("motion complete");
            return;
        }
        const Frame &f = frames[frameIndex];

        std::array<float, STEP> step{};
        Eigen::Quaterniond torso = robotTorsoQuat();
        Eigen::Quaterniond delta = (torso.conjugate() * initToWorld * f.quat).normalized();
        Eigen::Matrix3d R = delta.toRotationMatrix();

        for (int i = 0; i < N; ++i)
        {
            step[0 + i] = float(f.pos[i]);
            step[21 + i] = float(f.vel[i]);
            step[51 + i] = float(propri_.jointPos[i] - defaults[i]);
            step[72 + i] = float(propri_.jointVel[i]);
            step[93 + i] = actions[i];
        }
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 2; ++c)
                step[42 + r * 2 + c] = float(R(r, c));
        for (int i = 0; i < 3; ++i)
            step[48 + i] = float(propri_.baseAngVel[i]);

        if (!historyReady)
        {
            for (int h = 0; h < HISTORY; ++h)
                std::copy(step.begin(), step.end(), obs.begin() + h * STEP);
            historyReady = true;
        }
        else
        {
            std::memmove(obs.data(), obs.data() + STEP, (OBS - STEP) * sizeof(float));
            std::copy(step.begin(), step.end(), obs.end() - STEP);
        }

        for (float x : obs)
            if (!std::isfinite(x))
                throw std::runtime_error("nonfinite observation");

        std::array<int64_t, 2> shape{{1, OBS}};
        auto input = Ort::Value::CreateTensor<float>(
            memoryInfo, obs.data(), OBS, shape.data(), 2);
        auto output = session->Run(Ort::RunOptions{nullptr},
                                   &inputName, &input, 1, &outputName, 1);

        if (output.size() != 1 || !output[0].IsTensor() ||
            output[0].GetTensorTypeAndShapeInfo().GetElementCount() != (size_t)N)
            throw std::runtime_error("invalid action output");

        const float *a = output[0].GetTensorData<float>();
        for (int i = 0; i < N; ++i)
        {
            if (!std::isfinite(a[i]))
                throw std::runtime_error("nonfinite action");
            actions[i] = std::max(-18.f, std::min(18.f, a[i]));
        }
        ++frameIndex;
    }

    void tick()
    {
        try
        {
            const bool stateOk = updateStateEstimation();
            if (!stateOk)
                throw std::runtime_error("stale/invalid hardware state");
            if (propri_.projectedGravity.z() >= kDanceTiltGuard ||
                std::abs(propri_.baseEulerXyz[0]) >= 0.45 ||
                std::abs(propri_.baseEulerXyz[1]) >= 0.45)
            {
                std::cerr << "[DANCE TILT GUARD]"
                          << " pitch=" << propri_.baseEulerXyz[1]
                          << " roll=" << propri_.baseEulerXyz[0]
                          << " gravity_z=" << propri_.projectedGravity.z()
                          << std::endl;
                throw std::runtime_error("tilt");
            }

            if (phase != Phase::PLAYING)
                return;
            if (ticks % DECIMATION == 0)
                infer();
            if (phase != Phase::PLAYING)
                return;
            std::array<MotorCmd, N> cmd{};
            for (int i = 0; i < N; ++i)
            {
                int hw = ctrl->getJointsIndex(jointNames[i]);
                if (hw < 0 || hw >= N)
                    throw std::runtime_error("motor mapping");
                double target = defaults[i] + double(actions[i]) * scale[i];
                if (!targetReady)
                    lastTarget[i] = propri_.jointPos[i];
                target = std::max(lastTarget[i] - kDanceTargetStep,
                                  std::min(lastTarget[i] + kDanceTargetStep, target));
                lastTarget[i] = target;
                cmd[hw].pos = target;
                cmd[hw].vel = 0;
                cmd[hw].tau = 0;
                cmd[hw].kp = kp[i];
                cmd[hw].kd = kd[i];
                cmd[hw].motor_id = hw;
            }
            targetReady = true;
            ctrl->set_joint(cmd);
            ++ticks;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[DANCE] fault: " << e.what() << "\n";
            active = false;
            phase = Phase::IDLE;
            mode_ = WorkMode::DEFAULT;
            g_walkResumeBlendActive = false;
            g_danceStartPending = false;
            g_danceStableTimerStarted = false;
            std::cerr << "[DANCE] fault: control handed to DEFAULT; no transition interpolation\n";
        }
    }

} // namespace dance

//  主处理
void process()
{
    static int keyflag[14] = {0};
    if (initfinish == 0)
        return;
    if (!hardwareFresh())
    {
        static bool once = false;
        if (!once)
        {
            std::cerr << "[SAFETY] hardware status missing/stale; no motor command\n";
            once = true;
        }
        return;
    }

    Command cmd;
    const joydata l_jdata = aoliondriver.getremotedata();
    {
        memcpy(remote_data.button, &l_jdata.button[0], sizeof(remote_data.button));
        memcpy(remote_data.axes, &l_jdata.axes[0], sizeof(remote_data.axes));
    }

    cmd.x = remote_data.axes[1];
    cmd.y = 0;
    cmd.yaw = remote_data.axes[0];

    if (remote_data.button[9] == 1 && keyflag[9] == 0)
    {
        if (!startcontrol)
        {
            startcontrol = true;
            standPercent = 0;
            mode_ = WorkMode::LIE;
            keyflag[9] = 1;
            auto motor_snap = getMotorSnapshot();
            for (int i = 0; i < actuatedDofNum_; i++)
            {
                int index = ctrl->getJointsIndex(jointNames[i]);
                if (index >= 0 && index < kNumJoints)
                    currentJointAngles_[i] = motor_snap[index].pos;
            }
            printf("start control\n");
        }
        else
        {
            if (dance::active)
            {
                std::cerr << "[CONTROL] stop rejected during dance; stop DANCE with LB+Y or button 11\n";
                keyflag[9] = 1;
            }
            else
            {
                startcontrol = false;
                mode_ = WorkMode::DEFAULT;
                keyflag[9] = 1;
                printf("stop control\n");
            }
        }
    }
    else if (remote_data.button[9] == 0)
    {
        keyflag[9] = 0;
    }

    if (remote_data.button[10] == 1 && remote_data.button[2] == 1 &&
        keyflag[10] == 0)
    {
        if (startcontrol)
        {
            if (mode_ != WorkMode::STAND && !dance::active)
            {
                standPercent = 0;
                mode_ = WorkMode::STAND;
                auto motor_snap = getMotorSnapshot();
                for (int i = 0; i < actuatedDofNum_; i++)
                {
                    int index = ctrl->getJointsIndex(jointNames[i]);
                    if (index >= 0 && index < kNumJoints)
                        currentJointAngles_[i] = motor_snap[index].pos;
                }
                standStartAngles_ = currentJointAngles_;
                printf("->STAND\n");
            }
            else if (mode_ == WorkMode::LIE)
            {
                standPercent = 0;
                standStartAngles_ = currentJointAngles_;
                mode_ = WorkMode::STAND;
                printf("LIE2STAND\n");
            }
        }
        keyflag[10] = 1;
    }
    else if (remote_data.button[10] == 0)
    {
        keyflag[10] = 0;
    }

    if (remote_data.button[5] == 1 && remote_data.button[2] == 1 &&
        keyflag[5] == 0)
    {
        if (mode_ == WorkMode::STAND && !dance::active)
        {
            standPercent = 0;
            isChangeMode_ = true;
            mode_ = WorkMode::USERMODE;
            g_isfirstRecObs = true;
            count = 0;
            keyflag[5] = 1;
            printf("TO WALK MODE\n");
        }
    }
    else if (remote_data.button[5] == 0)
    {
        keyflag[5] = 0;
    }

    if (remote_data.button[11] == 1 && keyflag[11] == 0)
    {
        if (dance::active)
        {
            dance::stop("remote button 11");
        }
        else if (mode_ == WorkMode::USERMODE && !dance::active)
        {
            isChangeMode_ = true;
            mode_ = WorkMode::STAND;
            if (updateStateEstimation())
                standStartAngles_ = propri_.jointPos;
            standPercent = 0;
            printf("WALK2STAND\n");
        }
        else if (mode_ == WorkMode::DEFAULT)
        {
            standPercent = 0;
            isChangeMode_ = true;
            mode_ = WorkMode::LIE;
            printf("deftolie\n");
        }
        keyflag[11] = 1;
    }
    else if (remote_data.button[11] == 0)
    {
        keyflag[11] = 0;
    }

    if (remote_data.button[2] == 1 && remote_data.button[6] == 1 &&
        keyflag[6] == 0)
    {
        keyflag[6] = 1;
        if (dance::active)
        {
            dance::stop("LB+Y remote stop");
        }
        else if (g_danceStartPending)
        {
            cancelDanceStartRequest("LB+Y pressed again");
        }
        else if (mode_ == WorkMode::USERMODE && startcontrol)
        {
            if (!dance::ready)
            {
                std::cerr << "[DANCE] request rejected: dance model/config not ready\n";
            }
            else
            {
                g_danceStartPending = true;
                g_danceRequestTime = std::chrono::steady_clock::now();
                g_danceStableTimerStarted = false;
                std::cout << "[DANCE] LB+Y received. Keep WALK policy active; zeroing joystick and waiting for stable posture.\n";
            }
        }
        else
        {
            std::cerr << "[DANCE] LB+Y ignored: robot must already be in WALK USERMODE\n";
        }
    }
    else if (remote_data.button[6] == 0)
    {
        keyflag[6] = 0;
    }

    // 等待阶段仍然执行 WALK ONNX，绝不向电机发送空指令或 DANCE 位置插值。
    if (g_danceStartPending && !dance::active)
    {
        if (!startcontrol || mode_ != WorkMode::USERMODE)
        {
            cancelDanceStartRequest("WALK mode/control no longer active");
        }
        else
        {
            cmd.x = 0.0;
            cmd.y = 0.0;
            cmd.yaw = 0.0;
            setparameter(cmd, &isChangeMode_);

            if (!handleUserMode())
            {
                cancelDanceStartRequest("WALK state/policy failure");
                mode_ = WorkMode::DEFAULT;
            }
            else
            {
                const auto now = std::chrono::steady_clock::now();
                const bool waitElapsed = now - g_danceRequestTime >= kDanceSettleWait;
                bool stable = updateStateEstimation() &&
                              propri_.projectedGravity.z() < -0.90 &&
                              std::abs(propri_.baseEulerXyz[0]) < kDanceStableTiltRad &&
                              std::abs(propri_.baseEulerXyz[1]) < kDanceStableTiltRad &&
                              propri_.baseAngVel.norm() < kDanceStableGyro &&
                              std::abs(remote_data.axes[0]) <= kWalkToDanceMaxCommand &&
                              std::abs(remote_data.axes[1]) <= kWalkToDanceMaxCommand;
                if (stable)
                {
                    for (int i = 0; i < kNumJoints; ++i)
                    {
                        if (!std::isfinite(propri_.jointVel[i]) ||
                            std::abs(propri_.jointVel[i]) > kDanceStableJointSpeed)
                        {
                            stable = false;
                            break;
                        }
                    }
                }

                if (!stable)
                {
                    g_danceStableTimerStarted = false;
                }
                else if (waitElapsed)
                {
                    if (!g_danceStableTimerStarted)
                    {
                        g_danceStableSince = now;
                        g_danceStableTimerStarted = true;
                        std::cout << "[DANCE] WALK posture stable; verifying hold duration...\n";
                    }
                    else if (now - g_danceStableSince >= kDanceStableDuration)
                    {
                        std::cout << "[DANCE] stable check passed. Directly handing control to DANCE ONNX.\n";
                        if (!dance::start())
                            cancelDanceStartRequest("DANCE start safety checks failed");
                    }
                }
                static auto lastSettleLog = std::chrono::steady_clock::time_point{};
                if (now - lastSettleLog >= std::chrono::milliseconds(500))
                {
                    lastSettleLog = now;
                    std::cout << "[DANCE SETTLE] stable=" << (stable ? "yes" : "no")
                              << " wait_elapsed=" << (waitElapsed ? "yes" : "no")
                              << " pitch=" << propri_.baseEulerXyz[1]
                              << " roll=" << propri_.baseEulerXyz[0]
                              << " gravity_z=" << propri_.projectedGravity.z()
                              << " gyro_norm=" << propri_.baseAngVel.norm() << "\n";
                }
            }
        }
        if (!dance::active)
            return;
    }

    if (dance::active)
    {
        static bool firstDanceTick = true;
        if (firstDanceTick)
        {
            std::cout << "\n========== DIRECT WALK -> DANCE ==========" << "\n";
            std::cout << "WALK policy execution stops only after settle checks pass\n";
            std::cout << "No ENTERING / EXITING position interpolation\n";
            std::cout << "DANCE ONNX now owns motor commands\n";
            firstDanceTick = false;
        }
        dance::tick();
        return;
    }

    switch (mode_)
    {
    case WorkMode::STAND:
        handleStandMode();
        break;
    case WorkMode::LIE:
        handleLieMode();
        break;
    case WorkMode::DEFAULT:
        handleDefautMode();
        break;
    case WorkMode::USERMODE:
        setparameter(cmd, &isChangeMode_);
        if (!handleUserMode())
        {
            std::cerr << "[WALK] state/policy failure; stopping policy\n";
            mode_ = WorkMode::DEFAULT;
        }
        break;
    default:
        printf("Unexpected mode: %d\n", static_cast<int>(mode_));
        break;
    }
}

//  ONNX / YAML 加载
void onnxdatainit()
{
    Ort::AllocatorWithDefaultOptions allocator;
    for (int i = 0; i < (int)policySessionPtr->GetInputCount(); i++)
    {
        auto p = policySessionPtr->GetInputNameAllocated(i, allocator);
        policyInputNodeNameAllocatedStrings.push_back(std::move(p));
        policyInputNames_.push_back(
            policyInputNodeNameAllocatedStrings.back().get());
        auto shape = policySessionPtr->GetInputTypeInfo(i)
                         .GetTensorTypeAndShapeInfo()
                         .GetShape();
        policyInputShapes_.push_back(shape);
        std::cout << "Policy input [" << i << "]: [";
        for (size_t j = 0; j < shape.size(); ++j)
            std::cout << shape[j] << (j + 1 < shape.size() ? ", " : "");
        std::cout << "]\n";
    }
    for (int i = 0; i < (int)policySessionPtr->GetOutputCount(); i++)
    {
        auto p = policySessionPtr->GetOutputNameAllocated(i, allocator);
        policyOutputNodeNameAllocatedStrings.push_back(std::move(p));
        policyOutputNames_.push_back(
            policyOutputNodeNameAllocatedStrings.back().get());
        auto shape = policySessionPtr->GetOutputTypeInfo(i)
                         .GetTensorTypeAndShapeInfo()
                         .GetShape();
        policyOutputShapes_.push_back(shape);
        std::cout << "Policy output [" << i << "]: [";
        for (size_t j = 0; j < shape.size(); ++j)
            std::cout << shape[j] << (j + 1 < shape.size() ? ", " : "");
        std::cout << "]\n";
    }
}

bool getmodelparam()
{
    try
    {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string conpath = std::string(buf);
        RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
        YAML::Node acconfig = YAML::LoadFile(conpath + "/config/bumi_ac.yaml");

        standDuration = kStandFrames;
        standPercent = 0.0;

        controlCfg.decimation = acconfig[modelname]["control"]["decimation"].as<int>();
        controlCfg.cycle_time = acconfig[modelname]["control"]["cycle_time"].as<float>();

        robotconfig.clipObs = acconfig[modelname]["normalization"]
                                      ["clip_scales"]["clip_observations"]
                                          .as<double>();
        robotconfig.clipActions = acconfig[modelname]["normalization"]
                                          ["clip_scales"]["clip_actions"]
                                              .as<double>();

        actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
        observationSize_ = acconfig[modelname]["size"]["observations_size"].as<int>();
        stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

        scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
        scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
        scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();

        actions_.resize(actionsSize_);
        actuatedDofNum_ = kNumJoints;
        policyObservations_.resize(observationSize_ * stackSize_);
        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);

        lastActions_.resize(actionsSize_);
        lastActions_.setZero();

        proprioHistoryBuffer_.resize(stackSize_ * observationSize_);
        proprioHistoryBuffer_.setZero();

        defaultJointAngles_.resize(actionsSize_);
        for (int i = 0; i < actionsSize_ && i < (int)default_joint_pos.size(); i++)
            defaultJointAngles_(i) = default_joint_pos[i];
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[loadModel] getmodelparam failed: " << e.what() << "\n";
        return false;
    }
}

bool loadModel(std::string modelpath)
{
    try
    {
        Ort::SessionOptions sessionOptions;
        onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        sessionOptions.SetInterOpNumThreads(1);
        printf("Load Onnx model from path : %s\n", modelpath.c_str());
        policySessionPtr = std::make_unique<Ort::Session>(*onnxEnvPrt_, modelpath.c_str(), sessionOptions);

        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        policyInputNodeNameAllocatedStrings.clear();
        policyOutputNodeNameAllocatedStrings.clear();

        modelname = "run";
        command_.resize(3);
        g_isfirstCompAct = true;
        g_isfirstRecObs = true;
        count = 0;

        Ort::AllocatorWithDefaultOptions allocator;
        Ort::ModelMetadata metadata = policySessionPtr->GetModelMetadata();
        auto keys = metadata.GetCustomMetadataMapKeysAllocated(allocator);
        for (size_t i = 0; i < keys.size(); i++)
        {
            auto value = metadata.LookupCustomMetadataMapAllocated(keys[i].get(), allocator);
            std::string key = std::string(keys[i].get());
            if (key == "joint_names")
            {
                found_joint_names = true;
                joint_names = SplitString(std::string(value.get()), ',');
            }
            else if (key == "joint_stiffness")
            {
                found_joint_stiffness = true;
                joint_stiffness = SplitString2Double(std::string(value.get()), ',');
            }
            else if (key == "joint_damping")
            {
                found_joint_damping = true;
                joint_damping = SplitString2Double(std::string(value.get()), ',');
            }
            else if (key == "default_joint_pos")
            {
                found_default_joint_pos = true;
                default_joint_pos = SplitString2Double(std::string(value.get()), ',');
            }
            else if (key == "action_scale")
            {
                found_action_scale = true;
                action_scale = SplitString2Double(std::string(value.get()), ',');
            }
        }

        onnxdatainit();
        if (!getmodelparam())
            return false;
        if (actionsSize_ != kNumJoints || observationSize_ != 71 || stackSize_ != 5 ||
            robotconfig.controlCfg.decimation != 10 ||
            policyInputShapes_.size() != 1 || policyOutputShapes_.size() != 1 ||
            policyInputShapes_[0].size() != 2 || policyInputShapes_[0][1] != 355 ||
            policyOutputShapes_[0].size() != 2 || policyOutputShapes_[0][1] != 21)
            throw std::runtime_error("WALK ONNX/YAML dimensions mismatch");
        if (!found_joint_names || !found_joint_stiffness || !found_joint_damping ||
            !found_default_joint_pos || !found_action_scale ||
            joint_names.size() != 21 || joint_stiffness.size() != 21 ||
            joint_damping.size() != 21 || default_joint_pos.size() != 21 ||
            action_scale.size() != 21)
            throw std::runtime_error("WALK ONNX metadata incomplete");
        for (int i = 0; i < 21; ++i)
        {
            if ((joint_names[i] != jointNames[i] &&
                 joint_names[i] != dance::expectedDanceNames[i]) ||
                !std::isfinite(action_scale[i]) || !std::isfinite(default_joint_pos[i]) ||
                !std::isfinite(joint_stiffness[i]) || !std::isfinite(joint_damping[i]))
                throw std::runtime_error("WALK metadata joint order/value invalid");
        }
        initfinish = 1;
        printf("Load Onnx run model successfully !!!\n");
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[loadModel] failed: " << e.what() << "\n";
        return false;
    }
}

//  main
int main()
{
    try
    {
        // 注册信号处理器（仅设置原子标志）
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());

        ctrl = LowController::Instance();
        ctrl->init();
        init();

        if (!loadModel(path + "/policy/policy.onnx"))
            return 1;
        if (dance::loadConfig(path + "/config/bumi_ac.yaml"))
        {
            dance::load(path + "/policy/policy_dance.onnx",
                        path + "/policy/example_mimic_dance.json");
        }
        else
        {
            std::cerr << "[DANCE] dance unavailable; walking remains available\n";
        }
        dance::hardwareEnabled = dance::ready;
        if (dance::hardwareEnabled)
            std::cout << "[DANCE] real actuation enabled by program configuration\n";
        else
            std::cerr << "[DANCE] unavailable: configuration/model load failed\n";

        while (g_running.load(std::memory_order_acquire))
        {
            auto start_time = std::chrono::steady_clock::now();
            try
            {
                process();
            }
            catch (const std::exception &e)
            {
                std::cerr << "[process] exception: " << e.what() << "\n";
            }
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
                               .count();
            if (elapsed < kLoopPeriodUs)
                std::this_thread::sleep_for(std::chrono::microseconds(kLoopPeriodUs - elapsed));
            else
                std::this_thread::yield();
        }

        // 循环下发阻尼指令，确保电机可靠进入软停
        std::cout << "[main] Shutdown requested. Damping motors...\n";
        mode_ = WorkMode::DEFAULT;
        for (int i = 0; i < kShutdownDampFrames; ++i)
        {
            handleDefautMode();
            std::this_thread::sleep_for(kShutdownDampInterval);
        }
        std::cout << "[main] Motors damped. Exit.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[main] fatal: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
