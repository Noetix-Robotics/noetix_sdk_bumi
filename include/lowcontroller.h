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

        void set_joint(std::array<MotorCmd, 21> motorcmd);

	int getJointsIndex(const std::string jointname);

        void
        subscribe_robot_hardware_status(RobotHardwareStatusCallback callback);

      protected:
        void send_thread_func();

      private:
        std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
