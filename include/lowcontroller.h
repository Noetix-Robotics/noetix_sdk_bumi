#ifndef LowController_H
#define LowController_H
#include "DDSWrapper.h"
#include "common.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>

using namespace org::eclipse::cyclonedds;

namespace legged {
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
enum class WorkMode : uint8_t { STAND, LIE, USERMODE, DEFAULT };

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

        // InitState initState;
        ObsScales obsScales;
        ControlCfg controlCfg;

        int loophz;
        double cycletimeerrorThreshold;
        int ThreadPriority;
};

class LowController {

      public:
        ~LowController() = default;
        static LowController *Instance() {
                static LowController lowcontrol;
                return &lowcontrol;
        }

        bool init();
        const std::array<MotorState, 21> get_joint_state();
        void set_joint(std::array<MotorCmd, 21> motorcmd);
	NingImuData get_imu_data();
	joydata from_dds_get_joydata();
        int getJointsIndex(std::string jointname);

      protected:
        void set_robotstatusdata(std::array<MotorState, 21> motorstate_data,
                                 NingImuData imudata, joydata joy_data);

        void send_thread_func();

      private:
        DDSWrapper ddswrapper;

        std::thread send_thread_;
};
} // namespace legged
#endif
