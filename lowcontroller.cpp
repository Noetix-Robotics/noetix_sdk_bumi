#include "lowcontroller.h"
#include "RotationTools.h"
#include "aolion_driver.h"
#include "include/common.h"
#include "yaml-cpp/yaml.h"

#include <algorithm>

#include <chrono>
#include <common.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <thread>
#include <unistd.h>

namespace legged {
// Controllerbase controllerbase;

std::vector<std::string> SplitString(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
                auto start = token.find_first_not_of(" \t\r\n");
                if (start != std::string::npos) {
                        auto end = token.find_last_not_of(" \t\r\n");
                        tokens.push_back(token.substr(start, end - start + 1));
                }
        }
        return tokens;
}

std::vector<double> SplitString2Double(const std::string &str, char delimiter) {
        std::vector<double> values;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
                if (!token.empty()) {
                        values.push_back(std::stod(token));
                }
        }
        return values;
}

// JoystickFilter filter;
DataBuffer<std::array<MotorCmd, 21>> motor_cmd_buffer_;
DataBuffer<std::array<MotorState, 21>> motor_state_buffer_;
DataBuffer<RobotControlCmd::ControlCmd> control_cmd_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;
AoLionDriver aoliondriver;

LowController *LowController::instance = nullptr;
vector_t currentJointAngles_;
float current_vel_limit_ = 1.5;
bool LowController::init(ControlMode mode) {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        // printf("cur path is %s\n",path.c_str());
        // YAML::Node acconfig = YAML::LoadFile(path +
        // "/config/ning_user.yaml");
        mode_ = WorkMode::DEFAULT;
        aoliondriver.init("/dev/input/js0", 115200);

        actuatedDofNum_ = 21;
        standDuration = 1000;
        standPercent = 0;
        lieJointAngles_.resize(actuatedDofNum_);
        standJointAngles_.resize(actuatedDofNum_);
        currentJointAngles_.resize(actuatedDofNum_);
        // lieJointAngles_ 全0 — 与 RLControllerBase.h 中的 liejointState_ 一致
        lieJointAngles_.setZero();
        // standJointAngles_ 按 jointNames 顺序赋值 — 与 RLControllerBase.h 中的
        // standjointState_ 一致 jointNames: leg_l1, leg_r1, waist_1, leg_l2,
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
        instance = this;

        RobotSetMode::SetMode cmode;
        // cmode.mode(2);

        if (mode == ControlMode::LOWMODE) {
                cmode.mode(2);
        }

        ddswrapper.publishModeData(cmode);
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 21> data;
                    joydata remote_data;
                    NingImuData imudata;
                    int i = 0;
                    for (const auto &state :
                         ddsdata.motorstatearray().motorstates()) {
                            data[i].pos = state.pos();
                            data[i].vel = state.vel();
                            data[i].tau = state.tau();
                            data[i].motor_id = state.motor_id();
                            data[i].error = state.error();
                            data[i].temperature = state.temperature();
                            // std::cout << "motorstate  id: " <<
                            // data[i].motor_id
                            //<< ";pos " << data[i].pos << ";vel "
                            //<< data[i].vel << ";tau " << data[i].tau
                            //<< ";error " << data[i].error
                            //<< ";temperature " << data[i].temperature
                            //<< std::endl;
                            i++;
                    }
                    for (int i = 0; i < 4; i++) {
                            imudata.ori[i] = ddsdata.imudata().ori()[i];
                            // printf("[DEBUG]: imudata.ori[%d]= %.2f\r\n", i,
                            // imudata.ori[i]);
                    }
                    for (int i = 0; i < 3; i++) {
                            imudata.angular_vel[i] =
                                ddsdata.imudata().angular_vel()[i];
                            imudata.linear_acc[i] =
                                ddsdata.imudata().linear_acc()[i];
                            // printf(
                            //"[DEBUG]: imuxdata.angular_vel[%d] = %.2f \t\t "
                            //"imuxdata.linear_acc[%d] = %.2f \r\n",
                            // i, imudata.angular_vel[i], i,
                            // imudata.linear_acc[i]);
                    }
                    for (int i = 0; i < 9; i++) {
                            imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
                            imudata.angular_vel_cov[i] =
                                ddsdata.imudata().angular_vel_cov()[i];
                            imudata.linear_acc_cov[i] =
                                ddsdata.imudata().linear_acc_cov()[i];
                            // printf(
                            //"[DEBUG]: imuxdata.angular_vel_cov[%d] = %.2f \t\t
                            //" "imuxdata.linear_acc_cov[%d] = %.2f \r\n", i,
                            // imudata.angular_vel_cov[i], i,
                            // imudata.linear_acc_cov[i]);
                    }
                    // DEBUG joydata
                    // for (int i = 0; i < 14; i++) {
                    // if (ddsdata.joydata().button()[i] != 0)
                    // printf("button %d =%d\n", i,
                    // ddsdata.joydata().button()[i]);
                    //}
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));

                    LowController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });

        process_thread_ =
            std::thread(&LowController::process_thread_func, this);
        send_thread_ = std::thread(&LowController::send_thread_func, this);
        sched_param ddssched{.sched_priority = 98};
        if (pthread_setschedparam(process_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }

        if (pthread_setschedparam(send_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        printf("[DEBUG]: init finish\r\n");
        initfinish = 1;
        return true;
}

void LowController::set_robotstatusdata(std::array<MotorState, 21> data,
                                        NingImuData imudata, joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void LowController::send_thread_func() {
        while (1) {
                const std::shared_ptr<const std::array<MotorCmd, 21>> mc =
                    motor_cmd_buffer_.GetData();
                if (mc) {
                        RobotMotorCmd::MotorCmdArray cmdarray;
                        cmdarray.motorcmds().resize(21);
                        for (int i = 0; i < 21; i++) {
                                auto &cmd = cmdarray.motorcmds()[i];
                                cmd.pos() = mc->at(i).pos;
                                cmd.vel() = mc->at(i).vel;
                                cmd.tau() = mc->at(i).tau;
                                cmd.kp() = mc->at(i).kp;
                                cmd.kd() = mc->at(i).kd;
                                cmd.motor_id() = mc->at(i).motor_id;
                                // std::cout
                                //<< "motorstate  id: " << cmd.motor_id()
                                //<< ";pos " << cmd.pos() << ";vel "
                                //<< cmd.vel() << ";tau " << cmd.tau()
                                //<< std::endl;
                        }

                        auto now = Clock::now();
                        long long timestamp = std::chrono::duration_cast<
                                                  std::chrono::microseconds>(
                                                  now.time_since_epoch())
                                                  .count();
                        cmdarray.timestamp() = timestamp;
                        ddswrapper.publishMotorCmdData(cmdarray);
                }
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
}

void LowController::process_thread_func() {
        while (1) {
                auto start_time = std::chrono::steady_clock::now();
                process();
                auto end_time = std::chrono::steady_clock::now();
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_time - start_time);
                std::this_thread::sleep_for(std::chrono::microseconds(2000) -
                                            elapsed);
        }
}

void LowController::set_joint(std::array<MotorCmd, 21> motorcmd) {
        motor_cmd_buffer_.SetData(motorcmd);
}
const std::array<MotorState, 21> LowController::get_joint_state() {
        std::array<MotorState, 21> motorstate;
        const std::shared_ptr<const std::array<MotorState, 21>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 21; i++) {
                        motorstate[i].pos = ms->at(i).pos;
                        motorstate[i].vel = ms->at(i).vel;
                        motorstate[i].tau = ms->at(i).tau;
                        motorstate[i].motor_id = ms->at(i).motor_id;
                        motorstate[i].error = ms->at(i).error;
                        motorstate[i].temperature = ms->at(i).temperature;
                }
        }
        return motorstate;
}

int LowController::getJointsIndex(std::string jointname) {
        int index = -1;
        if (jointname == "arm_l1_joint") {
                index = 0;
        } else if (jointname == "arm_l2_joint") {
                index = 1;
        } else if (jointname == "arm_l3_joint") {
                index = 2;
        } else if (jointname == "arm_l4_joint") {
                index = 3;
        } else if (jointname == "leg_l1_joint") {
                index = 4;
        } else if (jointname == "leg_l2_joint") {
                index = 5;
        } else if (jointname == "leg_l3_joint") {
                index = 6;
        } else if (jointname == "leg_l4_joint") {
                index = 7;
        } else if (jointname == "leg_l5_joint") {
                index = 8;
        } else if (jointname == "leg_l6_joint") {
                index = 9;
        } else if (jointname == "arm_r1_joint") {
                index = 10;
        } else if (jointname == "arm_r2_joint") {
                index = 11;
        } else if (jointname == "arm_r3_joint") {
                index = 12;
        } else if (jointname == "arm_r4_joint") {
                index = 13;
        } else if (jointname == "leg_r1_joint") {
                index = 14;
        } else if (jointname == "leg_r2_joint") {
                index = 15;
        } else if (jointname == "leg_r3_joint") {
                index = 16;
        } else if (jointname == "leg_r4_joint") {
                index = 17;
        } else if (jointname == "leg_r5_joint") {
                index = 18;
        } else if (jointname == "leg_r6_joint") {
                index = 19;
        } else if (jointname == "waist_1_joint") {
                index = 20;
        }
        return index;
}

void LowController::setparameter(Command &cmd, bool *isfirst) {
        isfirstRecObs_ = isfirst;
        isfirstCompAct_ = *isfirstRecObs_;
        command_[0] = cmd.x;
        command_[1] = cmd.y;
        command_[2] = cmd.yaw;
}

bool LowController::updateStateEstimation() {
        // 与 RLControllerBase::updateStateEstimation 对齐
        // propri_.jointPos/jointVel 按 jointNames 顺序（0-20）存储
        // 通过 getJointsIndex 从硬件电机数组中读取对应数据
        vector_t jointPos(actuatedDofNum_), jointVel(actuatedDofNum_);
        quaternion_t quat;
        vector3_t angularVel, linearAccel;

        std::array<MotorState, 21> joint_state;
        const std::shared_ptr<const std::array<MotorState, 21>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                for (int i = 0; i < 21; i++) {
                        joint_state[i].pos = ms->at(i).pos;
                        joint_state[i].vel = ms->at(i).vel;
                        joint_state[i].tau = ms->at(i).tau;
                }
        }

        std::chrono::microseconds now =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch());

        // 按 jointNames 顺序读取关节状态
        for (size_t i = 0; i < actuatedDofNum_; ++i) {
                int hwIdx = getJointsIndex(jointNames[i]);
                if (hwIdx >= 0 && hwIdx < 21) {
                        jointPos(i) = joint_state[hwIdx].pos;
                        jointVel(i) = joint_state[hwIdx].vel;
                }
        }

        NingImuData imudata;
        const std::shared_ptr<const NingImuData> idata = imu_buffer_.GetData();
        if (idata) {
                for (int i = 0; i < 4; i++) {
                        imudata.ori[i] = (*idata).ori[i];
                }
                for (int i = 0; i < 3; i++) {
                        imudata.angular_vel[i] = (*idata).angular_vel[i];
                        imudata.linear_acc[i] = (*idata).linear_acc[i];
                }
                for (int i = 0; i < 9; i++) {
                        imudata.ori_cov[i] = (*idata).ori_cov[i];
                        imudata.angular_vel_cov[i] =
                            (*idata).angular_vel_cov[i];
                        imudata.linear_acc_cov[i] = (*idata).linear_acc_cov[i];
                }

                for (size_t i = 0; i < 4; ++i) {
                        quat.coeffs()(i) = imudata.ori[i];
                }
                for (size_t i = 0; i < 3; ++i) {
                        angularVel(i) = imudata.angular_vel[i];
                        linearAccel(i) = imudata.linear_acc[i];
                }

                propri_.jointPos = jointPos;
                propri_.jointVel = jointVel;
                propri_.baseAngVel = angularVel;

                vector3_t gravityVector(0, 0, -1);
                vector3_t zyx = quatToZyx(quat);
                propri_.baseEulerXyz[0] = zyx[2];
                propri_.baseEulerXyz[1] = zyx[1];
                propri_.baseEulerXyz[2] = zyx[0];
                matrix_t inverseRot =
                    getRotationMatrixFromZyxEulerAngles(zyx).inverse();
                propri_.projectedGravity = inverseRot * gravityVector;
                // std::cout<<"gravity"<<propri_.projectedGravity<<std::endl;
                // std::cout<<"pos"<<propri_.jointPos<<std::endl;
                // std::cout<<"vel"<<propri_.jointVel<<std::endl;
                // std::cout<<"base"<<propri_.baseAngVel<<std::endl;
                // propri_.baseEulerXyz = quatToXyz(quat);
        }
        double seconds = now.count();
        phase_ = seconds / 1000000.0;

        return true;
}

void LowController::handleDefautMode() {
        // 与 RLControllerBase::handleDefautMode 对齐
        // setCommand(0, 0, 0, 0.1, 0) => pos=0, vel=0, kp=0, kd=0.1, tau=0
        std::array<MotorCmd, 21> motorcmd;
        for (int j = 0; j < actuatedDofNum_; j++) {
                int hwIdx = getJointsIndex(jointNames[j]);
                if (hwIdx >= 0 && hwIdx < 21) {
                        motorcmd[hwIdx].pos = 0;
                        motorcmd[hwIdx].vel = 0;
                        motorcmd[hwIdx].kp = 0;
                        motorcmd[hwIdx].kd = 0.1;
                        motorcmd[hwIdx].tau = 0;
                        motorcmd[hwIdx].motor_id = hwIdx;
                }
        }
        set_joint(motorcmd);
}

void LowController::handleStandMode() {
        // 与 RLControllerBase::handleStandMode 对齐
        // pos_des = lieJointAngles_[j] * (1 - percent) + standJointAngles_[j] *
        // percent setCommand(pos_des, 0, 10, 0.5, 0)
        std::array<MotorCmd, 21> motorcmd;
        if (standPercent <= 1) {
                for (int j = 0; j < actuatedDofNum_; j++) {
                        scalar_t pos_des =
                            lieJointAngles_[j] * (1 - standPercent) +
                            standJointAngles_[j] * standPercent;
                        int hwIdx = getJointsIndex(jointNames[j]);
                        if (hwIdx >= 0 && hwIdx < 21) {
                                motorcmd[hwIdx].pos = pos_des;
                                motorcmd[hwIdx].vel = 0;
                                motorcmd[hwIdx].kp = 10.0;
                                motorcmd[hwIdx].kd = 0.5;
                                motorcmd[hwIdx].tau = 0;
                                motorcmd[hwIdx].motor_id = hwIdx;
                        }
                }
                set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, scalar_t(1));
        }
}

void LowController::handleLieMode() {
        // 与 RLControllerBase::handleLieMode 对齐
        // pos_des = currentJointAngles_[j] * (1 - percent) + lieJointAngles_[j]
        // * percent setCommand(pos_des, 0, 10, 0.5, 0)
        std::array<MotorCmd, 21> motorcmd;
        if (standPercent <= 1) {
                for (int j = 0; j < actuatedDofNum_; j++) {
                        scalar_t pos_des =
                            currentJointAngles_[j] * (1 - standPercent) +
                            lieJointAngles_[j] * standPercent;
                        int hwIdx = getJointsIndex(jointNames[j]);
                        if (hwIdx >= 0 && hwIdx < 21) {
                                motorcmd[hwIdx].pos = pos_des;
                                motorcmd[hwIdx].vel = 0;
                                motorcmd[hwIdx].kp = 10.0;
                                motorcmd[hwIdx].kd = 0.5;
                                motorcmd[hwIdx].tau = 0;
                                motorcmd[hwIdx].motor_id = hwIdx;
                        }
                }
                set_joint(motorcmd);
                standPercent += 1 / standDuration;
                standPercent = std::min(standPercent, scalar_t(1));
        }
}

bool LowController::handleUserMode() {
        // 与 AcController::handleWalkMode 对齐
        if (updateStateEstimation() == false)
                return false;

        // 摔倒保护 — 与 AcController 一致
        if (propri_.projectedGravity(2) >= -0.3) {
                printf("Fall Protection Triggered (UserMode)!\n");
                mode_ = WorkMode::DEFAULT;
                return true;
        }

        if (count % robotconfig.controlCfg.decimation == 0) {
                count = 0;
                computeObservation();
                computeActions();

                // limit action range
                scalar_t actionMin = -robotconfig.clipActions;
                scalar_t actionMax = robotconfig.clipActions;
                std::transform(
                    actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x) {
                            return std::max(actionMin, std::min(actionMax, x));
                    });
        }

        // set action — 与 AcController::handleWalkMode 一致
        // 直接遍历 actionsSize_，用 joint_names[i] 通过 getJointsIndex
        // 映射到硬件
        std::array<MotorCmd, 21> motorcmd;
        for (int i = 0; i < actionsSize_; i++) {
                if (i >= (int)jointNames.size())
                        break;

                std::string partName = jointNames[i];
                std::cout << "name" << partName << std::endl;
                int hwIdx = getJointsIndex(partName);

                scalar_t pos_des =
                    actions_[i] * action_scale[i] + defaultJointAngles_(i);

                if (i >= 0 && i < 21) {
                        motorcmd[i].pos = pos_des;
                        motorcmd[i].vel = 0;
                        motorcmd[i].kp = joint_stiffness[i];
                        motorcmd[i].kd = joint_damping[i];
                        motorcmd[i].tau = 0;
                        motorcmd[i].motor_id = hwIdx;
                }
                std::cout << "i=" << i << std::endl;
                std::cout << "index" << hwIdx << std::endl;

                lastActions_(i) = actions_[i];
        }

        set_joint(motorcmd);
        count++;
        return true;
}

void LowController::process() {
        static int keyflag[14];
        if (initfinish == 0)
                return;
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();

        const std::shared_ptr<const std::array<MotorState, 21>> ms =
            motor_state_buffer_.GetData();
        if (ms) {
                // const std::shared_ptr<const joydata> jdata =
                //     joy_buffer_.GetData();
                const joydata l_jdata = aoliondriver.getremotedata();

                const joydata *jdata = &l_jdata;
                if (jdata) {
                        memcpy(remote_data.button, &(*jdata).button[0],
                               sizeof(remote_data.button));
                        memcpy(remote_data.axes, &(*jdata).axes[0],
                               sizeof(remote_data.axes));
                        cmd.x = remote_data.axes[1];
                        cmd.y = 0;
                        cmd.yaw = remote_data.axes[0];

                        if ((remote_data.button[9] == 1) && (keyflag[9] == 0)) {
                                if (!startcontrol) {
                                        startcontrol = true;
                                        standPercent = 0;
                                        mode_ = WorkMode::LIE;
                                        keyflag[9] = 1;
                                        std::array<MotorState, 21> joint_state =
                                            get_joint_state();
                                        int index = 0;
                                        for (size_t i = 0; i < actuatedDofNum_;
                                             i++) {
                                                index = getJointsIndex(
                                                    jointNames[i]);
                                                currentJointAngles_[i] =
                                                    joint_state[index].pos;
                                        }
                                        printf("start control\n");

                                } else {
                                        startcontrol = false;
                                        mode_ = WorkMode::DEFAULT;
                                        keyflag[9] = 1;
                                        printf("stop control\n");
                                }

                        } else if (remote_data.button[9] == 0)
                                keyflag[9] = 0;
                        if ((remote_data.button[10] == 1) &&
                            (remote_data.button[2] == 1) &&
                            (keyflag[10] == 0)) {
                                if (startcontrol == true) {
                                        if (mode_ != WorkMode::STAND) {
                                                standPercent = 0;
                                                mode_ = WorkMode::STAND;
                                                std::array<MotorState, 21>
                                                    joint_state =
                                                        get_joint_state();
                                                int index = 0;
                                                for (size_t i = 0;
                                                     i < actuatedDofNum_; i++) {
                                                        index = getJointsIndex(
                                                            jointNames[i]);
                                                        currentJointAngles_[i] =
                                                            joint_state[index]
                                                                .pos;
                                                }
                                                printf("STAND2LIE\n");
                                        } else if (mode_ == WorkMode::LIE) {
                                                standPercent = 0;
                                                mode_ = WorkMode::STAND;
                                                printf("LIE2STAND\n");
                                        }
                                }
                        } else if (remote_data.button[10] == 0)
                                keyflag[10] = 0;
                        if ((remote_data.button[5] == 1) &&
                            (remote_data.button[2] == 1) && (keyflag[5] == 0)) {

                                if (mode_ == WorkMode::STAND) {
                                        standPercent = 0;
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::USERMODE;
                                        keyflag[5] = 1;
                                        printf("TO USERWALK MODE\n");
                                }

                        } else if (remote_data.button[5] == 0)
                                keyflag[5] = 0;
                        if ((remote_data.button[11] == 1) &&
                            (keyflag[11] == 0)) {
                                if (mode_ == WorkMode::USERMODE) {
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::STAND;
                                        printf("WALK2STAND\n");

                                } else if (mode_ == WorkMode::DEFAULT) {
                                        standPercent = 0;
                                        printf("deftolie\n");
                                        isChangeMode_ = true;
                                        mode_ = WorkMode::LIE;
                                }
                        }
                }

                switch (mode_) {
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
                        handleUserMode();
                        break;
                default:
                        printf("Unexpected mode encountered: %d\n",
                               static_cast<int>(mode_));
                        break;
                }
        }
}

void LowController::computeActions() {
        std::vector<Ort::Value> policyInputValues;
        policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
            memoryInfo, policyObservations_.data(), policyObservations_.size(),
            policyInputShapes_[0].data(), policyInputShapes_[0].size()));
        // run inference
        Ort::RunOptions runOptions;
        std::vector<Ort::Value> outputValues = policySessionPtr->Run(
            runOptions, policyInputNames_.data(), policyInputValues.data(), 1,
            policyOutputNames_.data(), 1);
        if (isfirstCompAct_) {
                for (int i = 0; i < policyObservations_.size(); ++i) {
                        std::cout << policyObservations_[i] << " ";
                        if ((i + 1) % observationSize_ == 0) {
                                std::cout << std::endl;
                        }
                }
                isfirstCompAct_ = false;
        }

        for (int i = 0; i < actionsSize_; i++) {
                actions_[i] =
                    *(outputValues[0].GetTensorMutableData<tensor_element_t>() +
                      i);
        }
}

void LowController::onnxdatainit() {
        Ort::AllocatorWithDefaultOptions allocator;
        for (int i = 0; i < policySessionPtr->GetInputCount(); i++) {
                auto policyInputnamePtr =
                    policySessionPtr->GetInputNameAllocated(i, allocator);
                policyInputNodeNameAllocatedStrings.push_back(
                    std::move(policyInputnamePtr));
                policyInputNames_.push_back(
                    policyInputNodeNameAllocatedStrings.back().get());
                // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i,
                // allocator).get());
                policyInputShapes_.push_back(
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }

        for (int i = 0; i < policySessionPtr->GetOutputCount(); i++) {
                auto policyOutputnamePtr =
                    policySessionPtr->GetOutputNameAllocated(i, allocator);
                policyOutputNodeNameAllocatedStrings.push_back(
                    std::move(policyOutputnamePtr));
                policyOutputNames_.push_back(
                    policyOutputNodeNameAllocatedStrings.back().get());
                // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i,
                // allocator).get());
                std::cout << policySessionPtr
                                 ->GetOutputNameAllocated(i, allocator)
                                 .get()
                          << std::endl;
                policyOutputShapes_.push_back(
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape());
                std::vector<int64_t> policyShape =
                    policySessionPtr->GetOutputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                std::cerr << "Policy Shape: [";
                for (size_t j = 0; j < policyShape.size(); ++j) {
                        std::cout << policyShape[j];
                        if (j != policyShape.size() - 1) {
                                std::cerr << ", ";
                        }
                }
                std::cout << "]" << std::endl;
        }
}

bool LowController::loadModel(std::string modelpath) {
        std::string policyFilePath;
        std::string estFilePath;
        // create session
        Ort::SessionOptions sessionOptions;
        bool ret;
        onnxEnvPrt_.reset(
            new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        sessionOptions.SetInterOpNumThreads(1);
        if (onnxEnvPrt_ == NULL) {
                printf("onnxEnvPrt_  is null\n");
                return false;
        }

        policyFilePath = modelpath;
        printf("Load Onnx model from path : %s\n", policyFilePath.c_str());

        policySessionPtr = std::make_unique<Ort::Session>(
            *onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
        if (policySessionPtr == NULL) {
                printf("load run model failed\n");
                return false;
        }

        // get input and output info
        policyInputNames_.clear();
        policyOutputNames_.clear();
        policyInputShapes_.clear();
        policyOutputShapes_.clear();
        estSessionPtr = NULL;
        modelname = "run";
        command_.resize(3);
        isfirstCompAct_ = true;
        isfirstRecObs_ = NULL;
        count = 0;
        model_type = 0;

        Ort::AllocatorWithDefaultOptions allocator;
        Ort::ModelMetadata metadata = policySessionPtr->GetModelMetadata();
        auto keys = metadata.GetCustomMetadataMapKeysAllocated(allocator);
        for (size_t i = 0ul; i < keys.size(); i++) {
                auto value = metadata.LookupCustomMetadataMapAllocated(
                    keys[i].get(), allocator);
                std::string key = std::string(keys[i].get());

                if (key == "joint_names") {
                        found_joint_names = true;
                        std::string joint_names_str = std::string(value.get());
                        auto Joint_Names = SplitString(joint_names_str, ',');
                        joint_names.resize(Joint_Names.size());
                        for (size_t j = 0; j < Joint_Names.size(); j++) {
                                joint_names[j] = Joint_Names[j];
                        }
                }
                if (key == "joint_stiffness") {
                        found_joint_stiffness = true;
                        std::string joint_stiffness_str =
                            std::string(value.get());
                        auto Joint_Stiffness =
                            SplitString2Double(joint_stiffness_str, ',');
                        joint_stiffness.resize(Joint_Stiffness.size());
                        for (size_t j = 0; j < Joint_Stiffness.size(); j++) {
                                joint_stiffness[j] = Joint_Stiffness[j];
                        }
                }
                if (key == "joint_damping") {
                        found_joint_damping = true;
                        std::string joint_damping_str =
                            std::string(value.get());
                        auto Joint_Damping =
                            SplitString2Double(joint_damping_str, ',');
                        joint_damping.resize(Joint_Damping.size());
                        for (size_t j = 0; j < Joint_Damping.size(); j++) {
                                joint_damping[j] = Joint_Damping[j];
                        }
                }
                if (key == "default_joint_pos") {
                        found_default_joint_pos = true;
                        std::string default_joint_pos_str =
                            std::string(value.get());
                        auto Default_Joint_Pos =
                            SplitString2Double(default_joint_pos_str, ',');
                        default_joint_pos.resize(Default_Joint_Pos.size());
                        for (size_t j = 0; j < Default_Joint_Pos.size(); j++) {
                                default_joint_pos[j] = Default_Joint_Pos[j];
                        }
                }
                if (key == "action_scale") {
                        found_action_scale = true;
                        std::string action_scale_str = std::string(value.get());
                        auto Action_Scales =
                            SplitString2Double(action_scale_str, ',');
                        action_scale.resize(Action_Scales.size());
                        for (size_t j = 0; j < Action_Scales.size(); j++) {
                                action_scale[j] = Action_Scales[j];
                        }
                }
        }

        onnxdatainit();
        ret = getmodelparam();
        initfinish = 1;

        printf("Load Onnx run model successfully !!!\n");
        return true;
}

bool LowController::getmodelparam() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string conpath = std::string(buf);
        std::string path = modelname;
        // RobotCfg::InitState &initState = robotconfig.initState;
        RobotCfg::ControlCfg &controlCfg = robotconfig.controlCfg;
        RobotCfg::ObsScales &obsScales = robotconfig.obsScales;

        YAML::Node acconfig = YAML::LoadFile(conpath + "/config/bumi_ac.yaml");

        int error = 0;

        // removed falltostand and standtofall YAML logic to rely on native
        // parameters

        standDuration = 1000;
        standPercent = 0;
        // controlCfg.actionScale =
        // acconfig[modelname]["control"]["action_scale"].as<float>();
        controlCfg.decimation =
            acconfig[modelname]["control"]["decimation"].as<int>();
        controlCfg.cycle_time =
            acconfig[modelname]["control"]["cycle_time"].as<float>();

        robotconfig.clipObs = acconfig[modelname]["normalization"]
                                      ["clip_scales"]["clip_observations"]
                                          .as<double>();
        robotconfig.clipActions =
            acconfig[modelname]["normalization"]["clip_scales"]["clip_actions"]
                .as<double>();

        actionsSize_ = acconfig[modelname]["size"]["actions_size"].as<int>();
        observationSize_ =
            acconfig[modelname]["size"]["observations_size"].as<int>();

        stackSize_ = acconfig[modelname]["size"]["stack_size"].as<int>();

        scalez = acconfig[modelname]["axis_mappings"]["scalez"].as<float>();
        scaley = acconfig[modelname]["axis_mappings"]["scaley"].as<float>();
        scalex = acconfig[modelname]["axis_mappings"]["scalex"].as<float>();

        actions_.resize(actionsSize_);

        actuatedDofNum_ = 21;

        policyObservations_.resize(observationSize_ * stackSize_);

        std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
        lastActions_.resize(actionsSize_);
        lastActions_.setZero();
        const int inputSize = stackSize_ * observationSize_;
        proprioHistoryBuffer_.resize(inputSize);
        defaultJointAngles_.resize(actionsSize_);
        walkdefaultJointAngles_.resize(actionsSize_);
        for (int i = 0; i < actionsSize_; i++) {
                defaultJointAngles_(i) = default_joint_pos[i];
                // printf("defaultJointAngles[%d]
                // %f\n",i,modelcfg->defaultJointAngles_(i));
        }

        return true;
}

void LowController::computeObservation() {
        // 与 AcController::computeObservation 对齐
        vector_t command(3);

        // 绝对值小于0.3的都置0
        double cmd_x = command_[0] * scalex;
        double cmd_y = command_[1] * scaley;
        double cmd_z = command_[2] * scalez;

        if (cmd_x < 0.3)
                cmd_x = 0.0;
        if (abs(cmd_z) < 0.3)
                cmd_z = 0.0;

        // x的command限定
        if (cmd_x > current_vel_limit_)
                cmd_x = current_vel_limit_;

        // 速度大于 0.6, 限制原地转向的速度
        if (cmd_x > 0.6 && cmd_z > 1.0)
                cmd_z = 1.0;
        if (cmd_x > 0.6 && cmd_z < -1.0)
                cmd_z = -1.0;

        command[0] = cmd_x;
        command[1] = cmd_y;
        command[2] = cmd_z;

        // actions
        vector_t actions(lastActions_);

        // propri_.jointPos/jointVel 已按 jointNames 顺序存储
        // 直接使用，与 AcController::computeObservation 一致
        vector_t proprioObs(observationSize_);

        proprioObs << command,                        // 3
            propri_.baseAngVel,                       // 3
            propri_.baseEulerXyz(0),                  // 1
            propri_.baseEulerXyz(1),                  // 1
            (propri_.jointPos - defaultJointAngles_), // actionsSize_
            propri_.jointVel,                         // actionsSize_
            actions;                                  // actionsSize_

        if (*isfirstRecObs_) {
                for (int i = observationSize_ - actionsSize_;
                     i < observationSize_; i++) {
                        proprioObs(i, 0) = 0.0;
                }

                for (size_t i = 0; i < stackSize_; i++) {
                        proprioHistoryBuffer_.segment(i * observationSize_,
                                                      observationSize_) =
                            proprioObs.cast<tensor_element_t>();
                }
                *isfirstRecObs_ = false;
                std::fill(policyObservations_.begin(),
                          policyObservations_.end(), 0.0f);
        }

        proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() -
                                   observationSize_) =
            proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() -
                                       observationSize_);
        proprioHistoryBuffer_.tail(observationSize_) =
            proprioObs.cast<tensor_element_t>();

        for (size_t i = 0; i < (observationSize_ * stackSize_); i++) {
                policyObservations_[i] =
                    static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
        }

        // Limit observation range
        scalar_t obsMin = -robotconfig.clipObs;
        scalar_t obsMax = robotconfig.clipObs;
        std::transform(policyObservations_.begin(), policyObservations_.end(),
                       policyObservations_.begin(),
                       [obsMin, obsMax](scalar_t x) {
                               return std::max(obsMin, std::min(obsMax, x));
                       });
}

} // namespace legged

int main() {
        // setenv("CYCLONEDDS_URI","file:///home/oem/test/dds-test/config/dds.xml",1);
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        legged::LowController lowcontroller;
        lowcontroller.init(legged::ControlMode::LOWMODE);
        lowcontroller.loadModel(path + "/policy/policy.onnx");
        while (1) {
                usleep(10);
        }
        return 0;
}
