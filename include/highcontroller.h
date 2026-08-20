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

        void
        subscribe_robot_hardware_status(RobotHardwareStatusCallback callback);

      private:
        std::unique_ptr<DDSWrapper> ddswrapper;
};
} // namespace noetix
#endif
