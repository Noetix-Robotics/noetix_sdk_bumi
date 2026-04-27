#include "common.h"
#include "highcontroller.h"
#include <algorithm>
using namespace legged;

int main(int argc, char *argv[]) {
        char buf[256];
        bool ret = true;
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());
        HighController *ctrl = HighController::Instance();
        ctrl->init();

        joydata remote_data;

        int key_updown[14], key_inuse[14];
        float x, yaw;
        int fileindex;
        ControlCmd action;

        while (true) {
                remote_data = ctrl->from_dds_get_joydata();
                for (int i = 0; i < 14; i++) {
                        if (remote_data.button[i] == 0) {
                                key_updown[i] = 0;
                                key_inuse[i] = 0;
                        } else if (remote_data.button[i] == 1) {
                                key_updown[i] = 1;
                        }
                }
                for (int i = 0; i < 14; i++) {
                        if (remote_data.button[i] == 0) {
                                key_updown[i] = 0;
                                key_inuse[i] = 0;
                        } else if (remote_data.button[i] == 1) {
                                key_updown[i] = 1;
                        }
                }
                action = ControlCmd::DEFAULT;
                x = remote_data.axes[1];
                yaw = remote_data.axes[0];
                if (key_updown[Key2] == 0 && key_updown[Key5] == 1 &&
                    key_updown[Key1] == 0 && (key_inuse[Key5] == 0)) {
                        action = ControlCmd::STARTTEACH;
                        key_inuse[Key5] = 1;
                        printf(
                            "[DEBUG]: STARTTEACH \n"); //                         X键
                } else if ((key_updown[Key6] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key6] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::SWING;
                        key_inuse[Key6] = 1;
                        printf(
                            "[DEBUG]: SWING \r\n"); //                            Y键
                } else if ((key_updown[Key7] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key7] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::SHAKE;
                        key_inuse[Key7] = 1;
                        printf(
                            "[DEBUG]: SHAKE \r\n"); //                            B键
                } else if ((key_updown[Key8] == 1) && key_updown[Key2] == 0 &&
                           (key_inuse[Key8] == 0) && key_updown[Key1] == 0) {
                        action = ControlCmd::CHEER;
                        key_inuse[Key8] = 1;
                        printf(
                            "[DEBUG]: CHEER \r\n"); //                            A键
                } else if ((key_updown[Key9] == 1) && (key_inuse[Key9] == 0)) {
                        key_inuse[Key9] = 1;
                        action = ControlCmd::START;
                        printf(
                            "[DEBUG]: START \r\n"); //			     +键
                } else if (key_updown[Key10] == 1 && (key_inuse[Key10] == 0)) {
                        action = ControlCmd::SWITCH;
                        key_inuse[Key10] = 1;
                        printf(
                            "[DEBUG]: SWITCH \n"); //			     -键
                } else if (key_updown[Key2] == 1 && key_updown[Key5] == 1 &&
                           (key_inuse[Key5] == 0)) {
                        action = ControlCmd::WALK;
                        key_inuse[Key5] = 1;
                        printf(
                            "[DEBUG]: WALK \n"); //                              LB+X
                } else if (key_updown[Key1] == 1 && key_updown[Key6] == 1 &&
                           (key_inuse[Key6] == 0)) {
                        action = ControlCmd::SAVETEACH;
                        key_inuse[Key6] = 1;
                        fileindex++;
                        printf(
                            "[DEBUG]: SAVETEACH \n"); //                        RB+Y
                } else if (key_updown[Key1] == 1 && key_updown[Key7] == 1 &&
                           (key_inuse[Key7] == 0)) {
                        action = ControlCmd::ENDTEACH;
                        key_inuse[Key7] = 1;
                        printf(
                            "[DEBUG]: ENDTEACH \n"); //                         RB+B
                } else if (key_updown[Key1] == 1 && key_updown[Key8] == 1 &&
                           (key_inuse[Key8] == 0)) {
                        action = ControlCmd::PLAYTEACH;
                        key_inuse[Key8] = 1;
                        fileindex = 1;
                        printf(
                            "[DEBUG]: PLAYTEACH \n"); //                       RB+A
                } else if (key_updown[Key1] == 1 && key_updown[Key5] == 1 &&
                           key_inuse[Key5] == 0) {
                        action = ControlCmd::FALLTOSTAND;
                        printf(
                            "[DEBUG]: FALL2STAND \r\n"); //                    RB+X
                        key_inuse[Key5] = 1;
                } else if (key_updown[Key2] == 1 && key_updown[Key6] == 1 &&
                           key_inuse[Key6] == 0) {
                        action = ControlCmd::DANCE;
                        printf(
                            "[DEBUG]: DANCE \r\n"); //                         LB+Y
                        key_inuse[Key6] = 1;
                } else if (key_updown[Key2] == 1 && key_updown[Key7] == 1 &&
                           key_inuse[Key7] == 0) {
                        action = ControlCmd::DANCE1;
                        printf(
                            "[DEBUG]: DANCE1 \r\n"); //                         LB+B
                        key_inuse[Key7] = 1;
                } else if (key_updown[Key2] == 1 && key_updown[Key8] == 1 &&
                           key_inuse[Key8] == 0) {
                        action = ControlCmd::DANCE2;
                        printf(
                            "[DEBUG]: DANCE2 \r\n"); //                         LB+A
                        key_inuse[Key8] = 1;
                } else if (key_updown[Key12] == 1 && key_updown[Key8] == 0 &&
                           key_inuse[Key12] == 0) {
                        action = ControlCmd::STANDTOFALL;
                        printf(
                            "[DEBUG]: STANDTOFALL \r\n"); //                         左摇杆按下
                        key_inuse[Key12] = 1;
                }

                ctrl->publish_cmd(x, yaw, action, fileindex);
                sleep_ms(2);
        }

        return 0;
}
