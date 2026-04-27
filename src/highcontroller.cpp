#include "highcontroller.h"
#include <Robot_status.hpp>
#include <common.h>
#include <cstdint>
#include <stdio.h>
#include <unistd.h>
using namespace org::eclipse::cyclonedds;

namespace legged {
DataBuffer<std::array<MotorState, 21>> motor_state_buffer_;
DataBuffer<joydata> joy_buffer_;
DataBuffer<NingImuData> imu_buffer_;

bool HighController::init() {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        printf("cur path is %s\n", path.c_str());

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
                    int curmode = ddsdata.workmode();
                    if (premode == 200) {
                            premode = curmode;
                    }
                    if (premode != curmode) {
                            premode = curmode;
                            printf("[DEBUG]: callback: current mode is %d\r\n",
                                   curmode);
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
                        data, imudata, remote_data, curmode);
            });
        printf("[DEBUG]: init finish \r\n");
        return true;
}

void HighController::publish_cmd(double ver, double hor, ControlCmd action,
                                 uint16_t index) {
        RobotControlCmd::ControlCmd controlcmd;
        controlcmd.axes()[0] = hor;
        controlcmd.axes()[1] = ver;
        controlcmd.action() = (int32_t)action;
        if (controlcmd.action() == 8 || controlcmd.action() == 10) {
                controlcmd.data() = index; // teach dance index
        }
        if (controlcmd.action() == 11) {
                controlcmd.data() = index;
        }
        ddswrapper.publishControlCmdData(controlcmd);
}

int HighController::get_mode() { return curmode_.load(); }

void HighController::set_robotstatusdata(std::array<MotorState, 21> data,
                                         NingImuData imudata, joydata joy_data,
                                         int curmode) {
        motor_state_buffer_.SetData(data);
        imu_buffer_.SetData(imudata);
        joy_buffer_.SetData(joy_data);
        curmode_.store(curmode);
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

joydata HighController::from_dds_get_joydata() {
        joydata jsdata;
        const std::shared_ptr<const joydata> jdata = joy_buffer_.GetData();
        if (jdata) {
                memcpy(jsdata.button, &(*jdata).button[0],
                       sizeof(jsdata.button));
                memcpy(jsdata.axes, &(*jdata).axes[0], sizeof(jsdata.axes));
        }
        return jsdata;
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

} // namespace legged
