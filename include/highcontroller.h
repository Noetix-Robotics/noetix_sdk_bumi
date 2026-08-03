#ifndef HighController_H
#define HighController_H
#include "common.h"

namespace noetix {

class DDSWrapper;

class HighController {

      public:
        ~HighController();
        static HighController *Instance();

        bool init();

        void publish_cmd(double x, double y, double z, ControlCmd action,
                         uint16_t index);

        int get_mode();

        joydata from_dds_get_joydata();

        const NingImuData get_imu_data();

        const std::array<MotorState, 21> get_joint_state();

        const RobotBmsData get_robot_bms_data();

      protected:
        void set_robotstatusdata(std::array<MotorState, 21> motorstate_data,
                                 NingImuData imudata, joydata joy_data,
                                 int curmode, RobotBmsData data);

      private:
	std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
