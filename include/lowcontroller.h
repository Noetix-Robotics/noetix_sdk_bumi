#ifndef LowController_H
#define LowController_H
#include "DDSWrapper.h"
#include "common.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <mutex>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <queue>
#include <shared_mutex>
#include <thread>

using namespace org::eclipse::cyclonedds;

namespace legged {
template <typename T> class DataBuffer {
      public:
        void SetData(const T &newData) {
                std::unique_lock<std::shared_mutex> lock(mutex);
                data = std::make_shared<T>(newData);
        }
        std::shared_ptr<const T> GetData() {
                std::shared_lock<std::shared_mutex> lock(mutex);
                return data ? data : nullptr;
        }
        void Clear() {
                std::unique_lock<std::shared_mutex> lock(mutex);
                data = nullptr;
        }

      private:
        std::shared_ptr<T> data;
        std::shared_mutex mutex;
};
enum class WorkMode : uint8_t { STAND, LIE, USERMODE, DEFAULT };

struct RobotCfg {
        struct ControlCfg {
                std::map<std::string, float> stiffness;
                std::map<std::string, float> damping;
                float actionScale;
                int decimation;
                float user_torque_limit;
                float user_power_limit;
                float cycle_time;
        };

        // struct InitState
        // {
        //   // default joint angles
        //   scalar_t arm_l1_joint;
        //   scalar_t arm_l2_joint;
        //   scalar_t arm_l3_joint;
        //   scalar_t arm_l4_joint;
        //   scalar_t leg_l1_joint;
        //   scalar_t leg_l2_joint;
        //   scalar_t leg_l3_joint;
        //   scalar_t leg_l4_joint;
        //   scalar_t leg_l5_joint;
        //   scalar_t arm_r1_joint;
        //   scalar_t arm_r2_joint;
        //   scalar_t arm_r3_joint;
        //   scalar_t arm_r4_joint;
        //   scalar_t leg_r1_joint;
        //   scalar_t leg_r2_joint;
        //   scalar_t leg_r3_joint;
        //   scalar_t leg_r4_joint;
        //   scalar_t leg_r5_joint;
        // };

        struct ObsScales {
                scalar_t linVel;
                scalar_t angVel;
                scalar_t dofPos;
                scalar_t dofVel;
                scalar_t quat;
                scalar_t heightMeasurements;
        };

        bool encoder_nomalize;

        scalar_t clipActions;
        scalar_t clipObs;

        // InitState initState;
        ObsScales obsScales;
        ControlCfg controlCfg;

        int loophz;
        double cycletimeerrorThreshold;
        int ThreadPriority;
};

class LowController {

      public:
        LowController()
            : memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,
                                                    OrtMemTypeDefault)) {}

        ~LowController() = default;
        static LowController *Instance() {
                static LowController lowcontrol;
                return &lowcontrol;
        }
        static LowController *instance; // 静态指针，用于调用非静态方法

        static void StaticCallback() {
                if (instance) {
                        instance->process(); // 通过实例指针调用成员函数
                }
        }
        bool init(ControlMode mode);
        bool loadModel(std::string modelpath);
        void setparameter(Command &cmd, bool *isfirst);
        void handleStandMode();
        void handleDefautMode();
        void handleLieMode();
        bool handleUserMode();

        void process();

      protected:
        void set_robotstatusdata(std::array<MotorState, 21> motorstate_data,
                                 NingImuData imudata, joydata joy_data);
        void computeActions();
        void computeObservation();
        int getJointsIndex(std::string jointname);

        void onnxdatainit();
        bool getmodelparam();
        bool updateStateEstimation();
        const std::array<MotorState, 21> get_joint_state();
        void set_joint(std::array<MotorCmd, 21> motorcmd);
        void process_thread_func();
        void send_thread_func();

      private:
        // std::vector<std::string> walkjointNames{
        //     "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
        //     "leg_r2_joint", "leg_l3_joint", "leg_r3_joint",  "arm_l1_joint",
        //     "arm_r1_joint", "leg_l4_joint", "leg_r4_joint",  "arm_l2_joint",
        //     "arm_r2_joint", "leg_l5_joint", "leg_r5_joint",  "arm_l3_joint",
        //     "arm_r3_joint", "leg_l6_joint", "leg_r6_joint",  "arm_l4_joint",
        //     "arm_r4_joint"};
        std::vector<std::string> jointNames{
            "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
            "leg_r2_joint", "arm_l1_joint", "arm_r1_joint",  "leg_l3_joint",
            "leg_r3_joint", "arm_l2_joint", "arm_r2_joint",  "leg_l4_joint",
            "leg_r4_joint", "arm_l3_joint", "arm_r3_joint",  "leg_l5_joint",
            "leg_r5_joint", "arm_l4_joint", "arm_r4_joint",  "leg_l6_joint",
            "leg_r6_joint"};

        DDSWrapper ddswrapper;
        std::string policyFilePath_;
        std::string modelname;
        int64_t count;
        RobotCfg robotconfig;
        Ort::MemoryInfo memoryInfo;
        std::shared_ptr<Ort::Env> onnxEnvPrt_;
        std::unique_ptr<Ort::Session> policySessionPtr;
        std::unique_ptr<Ort::Session> estSessionPtr;
        std::vector<const char *> policyInputNames_;
        std::vector<const char *> policyOutputNames_;
        std::vector<const char *> estInputNames_;
        std::vector<const char *> estOutputNames_;
        std::vector<Ort::AllocatedStringPtr>
            policyInputNodeNameAllocatedStrings;
        std::vector<Ort::AllocatedStringPtr>
            policyOutputNodeNameAllocatedStrings;
        std::vector<Ort::AllocatedStringPtr> estInputNodeNameAllocatedStrings;
        std::vector<Ort::AllocatedStringPtr> estOutputNodeNameAllocatedStrings;
        std::vector<std::vector<int64_t>> policyInputShapes_;
        std::vector<std::vector<int64_t>> policyOutputShapes_;
        std::vector<std::vector<int64_t>> estInputShapes_;
        std::vector<std::vector<int64_t>> estOutputShapes_;
        vector3_t baseLinVel_;
        vector3_t basePosition_;
        vector_t lastActions_;
        vector_t defaultJointAngles_;
        vector_t walkdefaultJointAngles_;
        int actuatedDofNum_;
        bool *isfirstRecObs_;
        int actionsSize_;
        int motionSize;
        int observationSize_;
        int stackSize_;
        float scalez;
        float scalex;
        float scaley;
        std::vector<tensor_element_t> actions_;
        std::vector<tensor_element_t> policyObservations_;
        std::vector<tensor_element_t> estObservations_;

        Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1>
            proprioHistoryBuffer_;
        bool isfirstCompAct_{true};
        vector_t command_;
        Proprioception propri_;
        double phase_;
        int64_t loopcount_;
        NingImuData imu_data_;
        joydata remote_data_;
        joydata remote_data;
        NingImuData imu_data;
        std::array<MotorState, 21> motorstate_;
        std::queue<std::array<MotorState, 21>> statequeue;
        std::queue<RobotMotorCmd::MotorCmdArray> cmdqueue;
        double standPercent;
        scalar_t standDuration;
        vector_t lieJointAngles;
        JointState standjointState_{
            0.0,     0.0,     0.0,     0.0, 0.0,    0.0,    0.0,
            -0.1495, -0.1495, 0.0,     0.0, 0.3215, 0.3215, 0.0000,
            0.0000,  -0.1720, -0.1720, 0.0, 0.0,    0.0,    0.0};

        JointState liejointState_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        WorkMode mode_;
        bool isChangeMode_ = false;
        bool startcontrol = false;
        int initfinish = 0;
        int model_type;
        long long statetimestamp;
        std::mutex state_mutex;
        std::array<MotorCmd, 21> usermotorcmd;

        vector_t lieJointAngles_;
        vector_t standJointAngles_;

        std::thread process_thread_;
        std::thread send_thread_;
        int new_state_arrived = false;
        bool found_joint_names{false};
        bool found_default_joint_pos{false};
        bool found_action_scale{false};
        bool found_joint_stiffness{false};
        bool found_joint_damping{false};
        std::vector<double> action_scale;
        std::vector<double> joint_stiffness;
        std::vector<double> joint_damping;
        std::vector<double> default_joint_pos;
        std::vector<std::string> joint_names;
        
        std::vector<double> stand_joint_stiffness;
        std::vector<double> stand_joint_damping;
        std::vector<double> lie_joint_stiffness;
        std::vector<double> lie_joint_damping;
};
} // namespace legged
#endif
