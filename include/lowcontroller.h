#ifndef LowController_H
#define LowController_H
#include "common.h"

namespace noetix {

class DDSWrapper;

enum class WorkMode : uint8_t { STAND, LIE, USERMODE, DEFAULT };

class LowController {

      public:
        ~LowController();
        static LowController *Instance();

        bool init();
        const std::array<MotorState, 21> get_joint_state();
        void set_joint(std::array<MotorCmd, 21> motorcmd);
        NingImuData get_imu_data();
        joydata from_dds_get_joydata();
        int getJointsIndex(std::string jointname);
        const RobotBmsData get_robot_bms_data();

      protected:
        void set_robotstatusdata(std::array<MotorState, 21> motorstate_data,
                                 NingImuData imudata, joydata joy_data,
                                 RobotBmsData bms_data);

        void send_thread_func();

      private:
        std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
