#include "highcontroller.h"
#include <Robot_status.hpp>
#include <chrono>
#include <common.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>
using namespace org::eclipse::cyclonedds;

namespace legged {
DataBuffer<std::array<MotorState, 21>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

HighController *HighController::instance = nullptr;
bool HighController::init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());

        instance = this;

        RobotSetMode::SetMode cmode;

        cmode.mode(1);
        ddswrapper.publishModeData(cmode);
        ddswrapper.subscribeRobotStatus(
            [](const RobotStatus::StatusData &ddsdata) {
                    std::array<MotorState, 21> data;
                    joydata remote_data;
                    NingImuData imudata;
                    static unsigned short premode = 200;
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
                            // printf("[DEBUG]: imudata.ori[%d]= %.f\r\n", i,
                            // imudata.ori[i]);
                    }
                    for (int i = 0; i < 3; i++) {
                            imudata.angular_vel[i] =
                                ddsdata.imudata().angular_vel()[i];
                            imudata.linear_acc[i] =
                                ddsdata.imudata().linear_acc()[i];
                    }
                    for (int i = 0; i < 9; i++) {
                            imudata.ori_cov[i] = ddsdata.imudata().ori_cov()[i];
                            imudata.angular_vel_cov[i] =
                                ddsdata.imudata().angular_vel_cov()[i];
                            imudata.linear_acc_cov[i] =
                                ddsdata.imudata().linear_acc_cov()[i];
                    }

                    RobotStatus::BmsState state = ddsdata.bmsdata();
                    unsigned short curmode = ddsdata.workmode();
                    if (premode == 200) {
                            premode = curmode;
                    }
                    if (premode != curmode) {
                            premode = curmode;
                            printf("[DEBUG]: current mode is %d\r\n", curmode);
                    }
                    // for (int i = 0; i < 14; i++) {
                    //  if (ddsdata.joydata().button()[i] != 0)
                    //  printf("button %d =%d\n", i,
                    //  ddsdata.joydata().button()[i]);
                    //}
                    memcpy(remote_data.button, &ddsdata.joydata().button(),
                           sizeof(remote_data.button));
                    memcpy(remote_data.axes, &ddsdata.joydata().axes(),
                           sizeof(remote_data.axes));
                    HighController::Instance()->set_robotstatusdata(
                        data, imudata, remote_data);
            });
        process_thread_ =
            std::thread(&HighController::process_thread_func, this);
        sched_param ddssched{.sched_priority = 98};
        if (pthread_setschedparam(process_thread_.native_handle(), SCHED_FIFO,
                                  &ddssched) != 0) {
                printf(" failed to set threads priority\n");
        }
        printf("[DEBUG]: init finish \r\n");
        return true;
}


void HighController::set_robotstatusdata(std::array<MotorState, 21> data,
                                         NingImuData imudata,
                                         joydata joy_data) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
}

void HighController::process_thread_func() {
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

const NingImuData HighController::get_imu_data() {
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
        }
	return imudata;
}


const std::array<MotorState, 21> HighController::get_joint_state() {
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

uint16_t fileindex = 0;

void HighController::process() {
        static int keyflag[14];
        joydata remote_data;
        static char key_updown[14], key_inuse[14];
        Command cmd;
        auto now = Clock::now();
        long starttimestamp =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch())
                .count();
        const std::shared_ptr<const joydata> mc = joy_buffer_.GetData();
        if (mc) {
                for (int i = 0; i < 14; i++) {
                        remote_data.button[i] = mc->button[i];
                }
                remote_data.axes[0] = mc->axes[0];
                remote_data.axes[1] = mc->axes[1];
        }

        for (int i = 0; i < 14; i++) {
                if (remote_data.button[i] == 0) {
                        key_updown[i] = 0;
                        key_inuse[i] = 0;
                } else if (remote_data.button[i] == 1) {
                        key_updown[i] = 1;
                }
        }
        ControlCmd action = ControlCmd::DEFAULT;
        cmd.x = remote_data.axes[1];
        cmd.yaw = remote_data.axes[0];
        if (key_updown[Key2] == 0 && key_updown[Key5] == 1 &&
            key_updown[Key1] == 0 && (key_inuse[Key5] == 0)) {
                action = ControlCmd::STARTTEACH;
                key_inuse[Key5] = 1;
                printf("[DEBUG]: STARTTEACH \n"); //                         X键
        } else if ((key_updown[Key6] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key6] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::SWING;
                key_inuse[Key6] = 1;
                printf("[DEBUG]: SWING \r\n"); //                            Y键
        } else if ((key_updown[Key7] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key7] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::SHAKE;
                key_inuse[Key7] = 1;
                printf("[DEBUG]: SHAKE \r\n"); //                            B键
        } else if ((key_updown[Key8] == 1) && key_updown[Key2] == 0 &&
                   (key_inuse[Key8] == 0) && key_updown[Key1] == 0) {
                action = ControlCmd::CHEER;
                key_inuse[Key8] = 1;
                printf("[DEBUG]: CHEER \r\n"); //                            A键
        } else if ((key_updown[Key9] == 1) && (key_inuse[Key9] == 0)) {
                key_inuse[Key9] = 1;
                action = ControlCmd::START;
                printf("[DEBUG]: START \r\n"); //			     +键
        } else if (key_updown[Key10] == 1 && (key_inuse[Key10] == 0)) {
                action = ControlCmd::SWITCH;
                key_inuse[Key10] = 1;
                printf("[DEBUG]: SWITCH \n"); //			     -键
        } else if (key_updown[Key2] == 1 && key_updown[Key5] == 1 &&
                   (key_inuse[Key5] == 0)) {
                action = ControlCmd::WALK;
                key_inuse[Key5] = 1;
                printf("[DEBUG]: WALK \n"); //                              LB+X
        } else if (key_updown[Key1] == 1 && key_updown[Key6] == 1 &&
                   (key_inuse[Key6] == 0)) {
                action = ControlCmd::SAVETEACH;
                key_inuse[Key6] = 1;
                fileindex++;
                printf("[DEBUG]: SAVETEACH \n"); //                        RB+Y
        } else if (key_updown[Key1] == 1 && key_updown[Key7] == 1 &&
                   (key_inuse[Key7] == 0)) {
                action = ControlCmd::ENDTEACH;
                key_inuse[Key7] = 1;
                printf("[DEBUG]: ENDTEACH \n"); //                         RB+B
        } else if (key_updown[Key1] == 1 && key_updown[Key8] == 1 &&
                   (key_inuse[Key8] == 0)) {
                action = ControlCmd::PLAYTEACH;
                key_inuse[Key8] = 1;
                fileindex = 1;
                printf("[DEBUG]: PLAYTEACH \n"); //                       RB+A
        } else if (key_updown[Key1] == 1 && key_updown[Key5] == 1 &&
                   key_inuse[Key5] == 0) {
                action = ControlCmd::FALLTOSTAND;
                printf("[DEBUG]: FALL2STAND \r\n"); //                    RB+X
                key_inuse[Key5] = 1;
        } else if (key_updown[Key2] == 1 && key_updown[Key6] == 1 &&
                   key_inuse[Key6] == 0) {
                action = ControlCmd::DANCE;
                printf("[DEBUG]: DANCE \r\n"); //                         LB+Y
                key_inuse[Key6] = 1;
        } else if (key_updown[Key2] == 1 && key_updown[Key7] == 1 &&
                   key_inuse[Key7] == 0) {
                action = ControlCmd::DANCE1;
                printf("[DEBUG]: DANCE1 \r\n"); //                         LB+B
                key_inuse[Key7] = 1;
        } else if (key_updown[Key2] == 1 && key_updown[Key8] == 1 &&
                   key_inuse[Key8] == 0) {
                action = ControlCmd::DANCE2;
                printf("[DEBUG]: DANCE2 \r\n"); //                         LB+A
                key_inuse[Key8] = 1;
        } else if (key_updown[Key12] == 1 && key_updown[Key8] == 0 &&
                   key_inuse[Key12] == 0) {
                action = ControlCmd::STANDTOFALL;
                printf(
                    "[DEBUG]: STANDTOFALL \r\n"); //                         左摇杆按下
                key_inuse[Key12] = 1;
        }

        RobotControlCmd::ControlCmd controlcmd;
        controlcmd.axes()[0] = cmd.yaw;
        controlcmd.axes()[1] = cmd.x;
        controlcmd.action() = (int)action;
        controlcmd.data() = fileindex;
        ddswrapper.publishControlCmdData(controlcmd);
}

} // namespace legged

int main() {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        legged::HighController highcontroller;
        highcontroller.init();

        while (1) {
                usleep(10);
        }
        return 0;
}
