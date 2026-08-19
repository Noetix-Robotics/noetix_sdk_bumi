#ifndef MEDIA_CONTROLLER
#define MEDIA_CONTROLLER
#include "common.h"

namespace noetix {

class MediaDDSWrapper;

class MediaController {
      public:
        static MediaController *Instance();

        ~MediaController();

        bool init();
        //=========================================================
        // System Control
        //=========================================================

        void set_system_control(media::SystemControlType type,
                                bool is_need_reply);

        //=========================================================
        // System State
        //=========================================================

        // 获取当前系统状态
        using SystemstatusCallback =
            std::function<void(const media::SystemStatus &)>;

        void subscribe_system_status(SystemstatusCallback callback);

        // 获取最近一次系统错误
        using SystemerrorCallback =
            std::function<void(const media::SystemError &)>;

        void subscribe_system_error(SystemerrorCallback callback);

        //=========================================================
        // 音量控制
        //=========================================================

        // 获取当前音量
        int get_volume();

        // 设置当前音量
        void set_volume(int value);

        //=========================================================
        // 通用配置
        //=========================================================

        // 获取超时时间（ms）
        int get_timeout();

        // 设置超时时间（ms）
        void set_timeout(int timeout_ms);

        // 获取提示音开关状态
        bool get_audio_cue_enable();

        // 设置提示音开关
        void set_audio_cue_enable(bool enable);

        // 获取内部麦克风音频是否发送给大模型
        bool get_internal_capture_audio_data_to_agent_enable();

        // 设置内部麦克风音频是否发送给大模型
        void set_internal_capture_audio_data_to_agent_enable(bool enable);

        // 获取外部音频是否发送给大模型
        bool get_external_custom_audio_data_to_agent_enable();

        // 设置外部音频是否发送给大模型
        void set_external_custom_audio_data_to_agent_enable(bool enable);

        // 获取大模型回复是否输出到扬声器
        bool get_internal_agent_audio_data_to_playback_enable();

        // 设置大模型回复是否输出到扬声器
        void set_internal_agent_audio_data_to_playback_enable(bool enable);

        // 获取外部音频是否允许输出到扬声器
        bool get_external_custom_audio_data_to_playback_enable();

        // 设置外部音频是否允许输出到扬声器
        void set_external_custom_audio_data_to_playback_enable(bool enable);

        // 获取内部视频是否发送给大模型
        bool get_internal_capture_video_data_to_agent_enable();

        // 设置内部视频是否发送给大模型
        void set_internal_capture_video_data_to_agent_enable(bool enable);

        // 获取外部视频是否发送给大模型
        bool get_external_custom_video_data_to_agent_enable();

        // 设置外部视频是否发送给大模型
        void set_external_custom_video_data_to_agent_enable(bool enable);

        // 获取外部音频是否使用机器人内部 3A 算法
        bool get_external_custom_audio_data_to_agent_use_internal_3a();

        // 设置外部音频是否使用机器人内部 3A 算法
        void
        set_external_custom_audio_data_to_agent_use_internal_3a(bool enable);

        //=========================================================
        // 唤醒配置
        //=========================================================

        // 获取唤醒回复词
        std::string get_wakeup_response_words();

        // 设置唤醒回复词
        void set_wakeup_response_words(const std::string &words);

        // 获取休眠回复词
        std::string get_sleep_response_words();

        // 设置休眠回复词
        void set_sleep_response_words(const std::string &words);

        // 获取当前所有唤醒词
        std::string get_wakeup_words();

        // 追加自定义唤醒词（不会覆盖系统默认唤醒词）
        bool add_wakeup_words(const std::string &words);

        // 恢复默认唤醒词（清除用户追加内容）
        void reset_wakeup_words();

        //=========================================================
        // Audio Stream
        //=========================================================

        using AudiostreamCallback =
            std::function<void(const media::AudioStream)>;

        // 获取机器人内部麦克风采集到的音频流
        void subscribe_internal_audio_capture(AudiostreamCallback callback);

        // 获取机器人内部播放的音频流
        void subscribe_internal_audio_playback(AudiostreamCallback callback);

        //=========================================================
        // Video Stream
        //=========================================================

        using VideostreamCallback =
            std::function<void(const media::VideoStream)>;

        // 获取机器人内部摄像头采集的视频流
        void subscribe_internal_video_capture(VideostreamCallback callback);

        // 获取脱敏后的视频流
        void subscribe_internal_video_desensed(VideostreamCallback callback);

        //=========================================================
        // Video Control
        //=========================================================

        // 暂停机器人内部摄像头采集
        void pause_video_capture();

        // 恢复机器人内部摄像头采集
        void resume_video_capture();

        //=========================================================
        // External Stream
        //=========================================================

        // 发布外部视频流给语音程序
        void publish_external_video_stream(const media::VideoStream &stream);

        // 发布外部音频流给语音程序
        void publish_external_audio_stream(const media::AudioStream &stream);

        // 发布外部音频流到机器人扬声器（external_playback 主题）
        void publish_external_audio_playback_stream(
            const media::AudioStream &stream);

        //=========================================================
        // Audio Control
        //=========================================================

        // 暂停内部音频采集
        void pause_audio_capture();

        // 恢复内部音频采集
        void resume_audio_capture();

        // 暂停音频播放
        void pause_audio_playback();

        // 恢复音频播放
        void resume_audio_playback();

      private:
        std::unique_ptr<MediaDDSWrapper> ddswrapper;
};

} // namespace noetix

#endif // !MEDIA_CONTROLLER
