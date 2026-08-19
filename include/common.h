#ifndef COMMON_H
#define COMMON_H

#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
namespace noetix {
#define SDK_VERSION "3.1.0"

// ==================== MOTION ====================

using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;
using Vecjoint = Eigen::VectorXd;
using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
using tensor_element_t = float;

template <typename T> using feet_array_t = std::array<T, 4>;

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

using contact_flag_t = feet_array_t<bool>;

enum class ControlMode : uint8_t { LOWMODE, HIGHMODE, USERMODE, DEFAULT };

enum class ControlCmd : uint32_t {
        WALK,
        SWING,
        SHAKE,
        CHEER,
        RUN,
        START,
        SWITCH,
        STARTTEACH,
        SAVETEACH,
        ENDTEACH,
        PLAYTEACH,
        DANCE,
        FALLTOSTAND,
        STANDTOFALL,
        DANCE1,
        DANCE2,
        TEAR,
        DEFAULT
};

struct RobotBmsData {
        uint16_t battery_temp_ = 0;
        uint16_t battery_alarm_ = 0;
        uint16_t battery_soc_ = 0;
        uint16_t battery_soh_ = 0;
};

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

struct RLRobotCfg {
        struct ControlCfg {
                std::map<std::string, float> stiffness;
                std::map<std::string, float> damping;
                float actionScale;
                int decimation;
                float user_torque_limit;
                float user_power_limit;
                float cycle_time;
                float bf_cycle_time;
        };

        struct ObsScales {
                scalar_t linVel;
                scalar_t angVel;
                scalar_t dofPos;
                scalar_t dofVel;
                scalar_t quat;
                scalar_t gravity;
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

struct Proprioception {
        vector_t jointPos;
        vector_t jointVel;
        vector3_t baseAngVel;
        vector3_t baseEulerXyz;
        vector3_t projectedGravity;
        quaternion_t robot_quat_;
};

struct Command {
        std::atomic<double> x;
        std::atomic<double> y;
        std::atomic<double> yaw;
};
struct MotorState {
        double pos;
        double vel;
        double tau;
        uint16_t motor_id;
        uint8_t error;
        int temperature;
};
struct MotorCmd {
        double pos;
        double vel;
        double tau;
        double kp;
        double kd;
        uint16_t motor_id;
};
struct NingImuData {
        double ori[4];
        double ori_cov[9];
        double angular_vel[3];
        double angular_vel_cov[9];
        double linear_acc[3];
        double linear_acc_cov[9];
};
struct joydata {
        double axes[2]; // horz,vert;
        int button[14];
};

// ==================== MEDIA ====================
namespace media {

struct Header {
        uint64_t message_id = 0;
        uint64_t timestamp_us = 0;
        std::string sn;
};

enum class WorkStatus { READY, SLEEPED, WAKEUPED, EXIT };

enum class StatusChangeReason {
        SYSTEM_LAUNCH,
        CMD_RESET,
        AUDIO_WAKEUPED,
        CMD_WAKEUPED,
        AUDIO_SLEEPED,
        CMD_SLEEPED,
        TIMEOUT_SLEEPED,
        ERROR_SLEEPED
};

enum class SystemControlType { TO_WAKEUP, TO_SLEEP, TO_RESET };

struct SystemStatus {
        Header header;
        WorkStatus value = WorkStatus::READY;
        StatusChangeReason reason = StatusChangeReason::SYSTEM_LAUNCH;
};

struct SystemError {
        Header header;
        int32_t code = 0;
        std::string message;
};

struct AudioStream {
        Header header;
        uint64_t timestamp_us = 0;
        uint32_t channels = 0;
        uint32_t sample_rate = 0;
        uint32_t format = 0;
        uint32_t duration_ms = 0;
        std::vector<int16_t> audio_data;
};

struct VideoStream {
        Header header;
        uint64_t timestamp_us = 0;
        uint32_t format = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t fps = 0;
        std::vector<uint8_t> video_data;
};

} // namespace media

} // namespace noetix
#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us()                                                          \
        std::chrono::duration_cast<std::chrono::microseconds>(                 \
            std::chrono::steady_clock::now().time_since_epoch())               \
            .count()
#endif
