#ifndef HighController_H
#define HighController_H

#include "DDSWrapper.h"
#include "common.h"
#include <atomic>
#include <chrono>
#include <eigen3/Eigen/Dense>

#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>

//#ifdef RK3588
//#include "rknn_api.h"
//#endif
using namespace org::eclipse::cyclonedds;
#define Key1 1   // 上扳机键
#define Key2 2   // 下扳机键
#define Key5 5   // 拇指上左
#define Key6 6   // 拇指上右
#define Key7 7   // 拇指下左
#define Key8 8   // 拇指下右
#define Key9 9   // 左侧上
#define Key10 10 // 左侧下
#define Key11 11 // 右侧上
#define Key12 12 //  右侧下
namespace legged {

enum class WorkMode : uint8_t { STAND, LIE, USERWALK, DEFAULT };

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

        //struct InitState {
                //// default joint angles
                //scalar_t arm_l1_joint;
                //scalar_t arm_l2_joint;
                //scalar_t arm_l3_joint;
                //scalar_t arm_l4_joint;
                //scalar_t arm_l5_joint;
                //scalar_t arm_l6_joint;
                //scalar_t leg_l1_joint;
                //scalar_t leg_l2_joint;
                //scalar_t leg_l3_joint;
                //scalar_t leg_l4_joint;
                //scalar_t leg_l5_joint;
                //scalar_t arm_r1_joint;
                //scalar_t arm_r2_joint;
                //scalar_t arm_r3_joint;
                //scalar_t arm_r4_joint;
                //scalar_t arm_r5_joint;
                //scalar_t arm_r6_joint;
                //scalar_t leg_r1_joint;
                //scalar_t leg_r2_joint;
                //scalar_t leg_r3_joint;
                //scalar_t leg_r4_joint;
                //scalar_t leg_r5_joint;
                //scalar_t waist_1_joint;
        //};

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

        //InitState initState;
        ObsScales obsScales;
        ControlCfg controlCfg;

        int loophz;
        double cycletimeerrorThreshold;
        int ThreadPriority;
};
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
class HighController {

      public:
        ~HighController() = default;
        static HighController *Instance() {
                static HighController highcontrol;
                return &highcontrol;
        }
        static HighController *instance; // 静态指针，用于调用非静态方法

        bool init(ControlMode mode);

        void process();

      protected:
        void onnxdatainit();
        bool getmodelparam();
        bool updateStateEstimation();
        const std::array<MotorState, 21> get_joint_state();
        void set_axes(double ver, double hor, int action, uint16_t index);
        const joydata get_jsdata();
        const NingImuData get_imu_data();
        void process_thread_func();
        void set_imudata(NingImuData imu_data);
        void set_joydata(joydata joy_data);
        void set_robotstatusdata(std::array<MotorState, 21> motorstate_data,
                                 NingImuData imudata, joydata joy_data);

      private:
        int64_t count;

        DDSWrapper ddswrapper;
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
        // RemoteDriver remotedriver;

        vector_t command_;

        int64_t loopcount_;
        NingImuData imu_data_;
        joydata remote_data_;

        // NingImuData imu_data;
        std::array<MotorState, 21> motorstate_;
        std::queue<std::array<MotorState, 21>> statequeue;
        std::queue<RobotMotorCmd::MotorCmdArray> cmdqueue;

        WorkMode mode_;
        bool isChangeMode_ = false;
        bool startcontrol = false;
        int initfinish = 0;
        int model_type;
        long long statetimestamp;
        std::mutex state_mutex;
        std::array<MotorCmd, 21> usermotorcmd;

        // DataBuffer<RobotMotorCmd::MotorCmdArray> motor_cmd_buffer_;
        // DataBuffer<std::array<MotorState,21>> motor_state_buffer_;
        // DataBuffer<joydata> joy_buffer_;
        // DataBuffer<NingImuData> imu_buffer_;

        std::thread process_thread_;
};
} // namespace legged
#endif
