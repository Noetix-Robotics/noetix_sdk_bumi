#include "lowcontroller.h"
#include <array>
#include <common.h>
using namespace legged;

int main(int argc, char *argv[]) {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        LowController *ctrl = LowController::Instance();
        ctrl->init();
        std::array<MotorCmd, 21> motorcmd;
        for (int i = 0; i < 21; i++) {
                motorcmd[i].motor_id = i;
                motorcmd[i].kd = 0;
                motorcmd[i].kp = 0;
                motorcmd[i].pos = 0;
                motorcmd[i].tau = 0;
                motorcmd[i].vel = 0;
        }
        motorcmd[0].kp = 5;
        motorcmd[0].kd = 5;
        motorcmd[0].pos = -2;

        ctrl->set_joint(motorcmd);
        while (true)
                ;

        return 0;
}
