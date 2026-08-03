/*
 * Example Media Demo — 集成音视频测试工具 (C++ 版, 无Qt)
 * 适用于 Jetson Orin Nano + RealSense D435i (aarch64 算力板)
 *
 * 用法:
 *   ./example_media capture_video              内部摄像头 → 抓一帧存 out/
 *           ⚠ 算力板无机器人内部相机, 需运控板相机才能用
 *   ./example_media external_video              外接USB摄像头 → 自动检测 +
 * 连续推流
 *   ./example_media capture_audio               内部麦克风 → 录 WAV 存 out/
 *   ./example_media playback_audio              内部扬声器 → 录 WAV 存 out/
 *   ./example_media external_audio_speaker [wav] 本地WAV/实时麦克风 →
 * 机器人扬声器
 *   ./example_media external_audio_ai <wav>      本地WAV → AI识别回答
 *   ./example_media desensed_video              脱敏视频 → 抓一帧存 out/
 *
 * 所有输出文件保存在 SDK 根目录的 out/ 文件夹下。
 * 依赖: ffmpeg (外部摄像头 & 格式转换), 无 Qt 依赖。
 */

#include "MediaController.h"
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace noetix;

// ═══════════════════════════════════════════════════════
// 信号处理 (Ctrl+C 优雅退出)
// ═══════════════════════════════════════════════════════

volatile bool g_running = true;

static void sigint_handler(int) { g_running = false; }

// ═══════════════════════════════════════════════════════
// 路径工具
// ═══════════════════════════════════════════════════════

static std::filesystem::path find_sdk_root() {
        auto dir = std::filesystem::current_path();
        for (int lv = 0; lv < 5; ++lv) {
                auto cfg = dir / "config" / "dds.xml";
                if (std::filesystem::is_regular_file(cfg))
                        return std::filesystem::absolute(dir);
                if (dir == dir.root_path())
                        break;
                dir = dir.parent_path();
        }
        return std::filesystem::current_path();
}

static std::string out_path(const std::string &filename) {
        static auto root = find_sdk_root();
        auto out_dir = root / "out";
        std::filesystem::create_directories(out_dir);
        return (out_dir / filename).string();
}

// ═══════════════════════════════════════════════════════
// WAV writer (简单, 无额外chunk)
// ═══════════════════════════════════════════════════════

static void write_wav(const std::string &path,
                      const std::vector<int16_t> &samples, uint32_t sample_rate,
                      uint16_t channels) {
        std::ofstream f(path, std::ios::binary);
        uint32_t dsz = samples.size() * 2;
        f.write("RIFF", 4);
        uint32_t fsz = 36 + dsz;
        f.write((char *)&fsz, 4);
        f.write("WAVE", 4);
        f.write("fmt ", 4);
        uint32_t ff = 16;
        f.write((char *)&ff, 4);
        uint16_t a = 1;
        f.write((char *)&a, 2);
        f.write((char *)&channels, 2);
        f.write((char *)&sample_rate, 4);
        uint32_t br = sample_rate * channels * 2;
        f.write((char *)&br, 4);
        uint16_t ba = channels * 2;
        f.write((char *)&ba, 2);
        uint16_t bp = 16;
        f.write((char *)&bp, 2);
        f.write("data", 4);
        f.write((char *)&dsz, 4);
        f.write((char *)samples.data(), dsz);
}

// ═══════════════════════════════════════════════════════
// WAV reader (扫描chunk, 兼容fact等额外chunk)
// ═══════════════════════════════════════════════════════

static bool read_audio_file(const std::string &path,
                            std::vector<int16_t> &samples,
                            uint32_t &sample_rate, uint16_t &channels) {
        std::ifstream f(path, std::ios::binary);
        if (!f)
                return false;
        f.seekg(12);
        int32_t data_size = -1;
        while (f.good()) {
                char ckid[5] = {};
                int32_t cksz;
                f.read(ckid, 4);
                f.read(reinterpret_cast<char *>(&cksz), 4);
                if (!f.good())
                        break;
                std::string id(ckid, 4);
                if (id == "fmt ") {
                        int16_t afmt, nch;
                        int32_t srate;
                        f.read(reinterpret_cast<char *>(&afmt), 2);
                        f.read(reinterpret_cast<char *>(&nch), 2);
                        f.read(reinterpret_cast<char *>(&srate), 4);
                        sample_rate = srate;
                        channels = nch;
                        f.seekg(cksz - 8, std::ios::cur);
                } else if (id == "data") {
                        data_size = cksz;
                        break;
                } else {
                        f.seekg(cksz, std::ios::cur);
                }
        }
        if (data_size <= 0 || sample_rate == 0)
                return false;
        samples.resize(data_size / 2);
        f.read(reinterpret_cast<char *>(samples.data()), data_size);
        return true;
}

// ═══════════════════════════════════════════════════════
// YUV422 → RGB
// ═══════════════════════════════════════════════════════

static void yuv422_to_rgb(const uint8_t *yuv, uint8_t *rgb, int width,
                          int height) {
        for (int i = 0; i < width * height; i += 2) {
                int y0 = yuv[i * 2];
                int u = yuv[i * 2 + 1] - 128;
                int y1 = yuv[i * 2 + 2];
                int v = yuv[i * 2 + 3] - 128;

                auto clamp = [](int v) {
                        return std::max(0, std::min(255, v));
                };

                int r0 = y0 + ((v * 359) >> 8);
                int g0 = y0 - ((u * 88) >> 8) - ((v * 183) >> 8);
                int b0 = y0 + ((u * 454) >> 8);

                int r1 = y1 + ((v * 359) >> 8);
                int g1 = y1 - ((u * 88) >> 8) - ((v * 183) >> 8);
                int b1 = y1 + ((u * 454) >> 8);

                int idx = i * 3;
                rgb[idx + 0] = clamp(r0);
                rgb[idx + 1] = clamp(g0);
                rgb[idx + 2] = clamp(b0);
                rgb[idx + 3] = clamp(r1);
                rgb[idx + 4] = clamp(g1);
                rgb[idx + 5] = clamp(b1);
        }
}

// ═══════════════════════════════════════════════════════
// PPM writer (P6 binary format, 无需任何第三方库)
// ═══════════════════════════════════════════════════════

static void write_ppm(const std::string &path, const uint8_t *rgb, int width,
                      int height) {
        std::ofstream f(path, std::ios::binary);
        f << "P6\n" << width << " " << height << "\n255\n";
        f.write(reinterpret_cast<const char *>(rgb), width * height * 3);
}

// 用 ffmpeg 把 PPM 转 PNG（更通用）
static void ppm_to_png(const std::string &ppm_path,
                       const std::string &png_path) {
        std::string cmd = "ffmpeg -y -i \"" + ppm_path + "\" \"" + png_path +
                          "\" 2>/dev/null";
        if (system(cmd.c_str()) == 0) {
                std::filesystem::remove(ppm_path);
        }
}

// ═══════════════════════════════════════════════════════
// 通用: 从机器人摄像头抓一帧并保存
// ═══════════════════════════════════════════════════════

static bool decode_and_save(const media::VideoStream &vs,
                            const std::string &out_png) {
        auto &data = vs.video_data;
        int w = vs.width, h = vs.height;
        if (data.empty() || w == 0 || h == 0)
                return false;

        std::vector<uint8_t> rgb(w * h * 3);

        if (data.size() == (size_t)(w * h * 2)) {
                // YUV422
                yuv422_to_rgb(data.data(), rgb.data(), w, h);
        } else if (data.size() == (size_t)(w * h * 3)) {
                // 已经是 RGB
                std::copy(data.begin(), data.end(), rgb.begin());
        } else {
                return false;
        }

        // 先写 PPM, 再用 ffmpeg 转 PNG
        std::string ppm = out_png;
        ppm.replace(ppm.rfind(".png"), 4, ".ppm");
        write_ppm(ppm, rgb.data(), w, h);
        ppm_to_png(ppm, out_png);
        return true;
}

static media::VideoStream grab_frame(MediaController *media,
                                     bool desensed = false) {
        // 尝试抓一帧, 最多等 5 秒
        for (int i = 0; i < 50; ++i) {
                auto vs = desensed ? media->get_video_capture_desensed_data()
                                   : media->get_video_capture_data();
                if (!vs.video_data.empty() && vs.width > 0)
                        return vs;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return media::VideoStream{};
}

// ═══════════════════════════════════════════════════════
// V4L2 Camera — 自动分辨率 + 曝光控制
//   (通过 ffmpeg v4l2 直接抓帧, 适配 RealSense D435i YUYV)
// ═══════════════════════════════════════════════════════

class V4L2Camera {
      public:
        // 打开摄像头: 指定设备路径, w/h=0 自动选择最佳分辨率
        bool open(const char *dev, int w = 0, int h = 0) {
                dev_ = dev;
                // 如果指定了分辨率, 直接尝试
                if (w > 0 && h > 0) {
                        return try_open(dev, w, h);
                }
                // 否则按 RealSense D435i 原生 YUYV 分辨率依次尝试
                static const int res[][2] = {
                    {640, 480}, {640, 360}, {424, 240}, {320, 240}, {320, 180}};
                for (auto &r : res) {
                        std::cout << "  尝试 " << r[0] << "x" << r[1]
                                  << "...\n";
                        if (try_open(dev, r[0], r[1]))
                                return true;
                }
                return false;
        }

        // 打开 ffmpeg pipe, 重试读首帧 (ffmpeg 需要启动时间)
        bool try_open(const char *dev, int w, int h) {
                width_ = w;
                height_ = h;
                std::string cmd =
                    "ffmpeg -fflags nobuffer -flags low_delay -probesize 32 "
                    "-f v4l2 -input_format yuyv422 -video_size " +
                    std::to_string(w) + "x" + std::to_string(h) + " -i " + dev +
                    " -f rawvideo -pix_fmt yuyv422 pipe:1 2>/dev/null";
                pipe_ = popen(cmd.c_str(), "r");
                if (!pipe_) {
                        std::cerr << "  ffmpeg v4l2 启动失败\n";
                        return false;
                }

                // 重试读取首帧, 最多 40 次, 每次 50ms
                for (int i = 0; i < 40; ++i) {
                        std::vector<uint8_t> test_frame;
                        size_t sz = w * h * 2;
                        test_frame.resize(sz);
                        if (fread(test_frame.data(), 1, sz, pipe_) == sz) {
                                std::cout << "  分辨率: " << w << "x" << h
                                          << "\n";
                                // 设置曝光: 自动曝光 + 固定曝光时间 500
                                std::string exp_cmd =
                                    "v4l2-ctl -d " + dev_ +
                                    " -c auto_exposure=1 -c "
                                    "exposure_time_absolute=90 2>/dev/null";
                                system(exp_cmd.c_str());
                                return true;
                        }
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(50));
                }
                // 超时, 关闭 pipe
                pclose(pipe_);
                pipe_ = nullptr;
                return false;
        }

        bool read(std::vector<uint8_t> &frame) {
                if (!pipe_)
                        return false;
                size_t sz = width_ * height_ * 2;
                frame.resize(sz);
                size_t n = fread(frame.data(), 1, sz, pipe_);
                return n == sz;
        }

        int width() const { return width_; }
        int height() const { return height_; }

        ~V4L2Camera() {
                if (pipe_)
                        pclose(pipe_);
        }

      private:
        FILE *pipe_ = nullptr;
        int width_ = 0, height_ = 0;
        std::string dev_;
};

// ═══════════════════════════════════════════════════════
// 命令实现
// ═══════════════════════════════════════════════════════

static int cmd_capture_video(MediaController *media) {
        std::cout << "[capture_video] 从内部摄像头抓帧 (最多等 5 秒)...\n";
        std::cout << "⚠ 算力板无机器人内部相机, 需运控板相机才能用\n";
        media->set_internal_capture_video_data_to_agent_enable(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        auto vs = grab_frame(media, false);
        if (vs.video_data.empty()) {
                std::cerr
                    << "  ✗ 未获取到视频帧!\n"
                    << "    可能原因: 机器人摄像头服务未启动 / 未连接机器人\n";
                return 1;
        }
        std::string out = out_path("capture_video.png");
        if (!decode_and_save(vs, out)) {
                std::cerr << "  解码失败!\n";
                return 1;
        }
        printf("[✓] 已保存: %s (%dx%d, %zuB)\n", out.c_str(), vs.width,
               vs.height, vs.video_data.size());
        return 0;
}

static int cmd_desensed_video(MediaController *media) {
        std::cout << "[desensed_video] 从脱敏视频抓帧 (最多等 5 秒)...\n";
        media->set_internal_capture_video_data_to_agent_enable(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        auto vs = grab_frame(media, true);
        if (vs.video_data.empty()) {
                std::cerr << "  ✗ 未获取到脱敏视频帧!\n"
                          << "    可能原因: 机器人脱敏功能未开启 / "
                             "机器人端未启用 desensed "
                             "topic\n";
                return 1;
        }
        std::string out = out_path("desensed_video.png");
        if (!decode_and_save(vs, out)) {
                std::cerr << "  解码失败!\n";
                return 1;
        }
        printf("[✓] 已保存: %s (%dx%d, %zuB)\n", out.c_str(), vs.width,
               vs.height, vs.video_data.size());
        return 0;
}

static int cmd_external_video(MediaController *media,
                              const std::string &dev_in) {
        std::string devpath =
            dev_in.empty()
                ? ""
                : (dev_in.find("/") == std::string::npos ? "/dev/video" + dev_in
                                                         : dev_in);

        // 自动检测: 依次尝试 /dev/video4 → video2 → video0
        if (devpath.empty()) {
                static const char *devs[] = {"/dev/video4", "/dev/video2",
                                             "/dev/video0"};
                bool found = false;
                for (auto d : devs) {
                        std::cout << "[external_video] 检测 " << d << "...\n";
                        V4L2Camera test_cam;
                        if (test_cam.try_open(d, 640, 360)) {
                                devpath = d;
                                found = true;
                                break;
                        }
                }
                if (!found) {
                        std::cerr << "  ✗ 未检测到可用摄像头 (尝试了 "
                                     "/dev/video4, video2, video0)\n";
                        return 1;
                }
                std::cout << "[external_video] 使用 " << devpath << "\n";
        }

        V4L2Camera cam;
        if (!cam.open(devpath.c_str(), 0, 0)) {
                std::cerr << "  打开摄像头失败!\n";
                return 1;
        }

        // 切换为外部视频源
        media->set_external_custom_video_data_to_agent_enable(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // 连续推流, Ctrl+C 停止
        std::cout << "  连续推流中... 按 Ctrl+C 停止\n";
        std::vector<uint8_t> last_frame;
        int fc = 0;
        auto start = std::chrono::steady_clock::now();
        while (g_running) {
                std::vector<uint8_t> frm;
                if (!cam.read(frm))
                        break;
                last_frame = frm;
                media::VideoStream vs;
                vs.width = cam.width();
                vs.height = cam.height();
                vs.format = 0;
                vs.fps = 30;
                vs.timestamp_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                vs.video_data = frm;
                media->publish_external_video_stream(vs);
                ++fc;
                if (fc % 30 == 0) {
                        double sec =
                            std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - start)
                                .count();
                        printf("\r  已推送 %d 帧 (%.1f fps)", fc, fc / sec);
                        fflush(stdout);
                }
        }
        printf("\n  推送完成, 共 %d 帧\n", fc);

        // 保存最后一帧为 PNG
        if (!last_frame.empty()) {
                std::vector<uint8_t> rgb(cam.width() * cam.height() * 3);
                yuv422_to_rgb(last_frame.data(), rgb.data(), cam.width(),
                              cam.height());
                std::string ppm = out_path("external_video.ppm");
                write_ppm(ppm, rgb.data(), cam.width(), cam.height());
                std::string png = out_path("external_video.png");
                ppm_to_png(ppm, png);
                printf("[✓] 最后一帧已保存: %s (%dx%d)\n", png.c_str(),
                       cam.width(), cam.height());
        }

        return 0;
}

static int record_audio(MediaController *media, const std::string &filename,
                        bool is_playback) {
        std::cout << "[" << (is_playback ? "playback_audio" : "capture_audio")
                  << "] 录制 10 秒...\n";

        // playback 需要等机器人恢复, capture 直接录
        if (is_playback) {
                std::cout << "  等待机器人恢复语音..." << std::endl;
                auto wait_t0 = std::chrono::steady_clock::now();
                while (std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - wait_t0)
                           .count() < 10.0) {
                        auto check = media->get_audio_playback_data();
                        if (!check.audio_data.empty())
                                break;
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(100));
                }
        }
        std::vector<int16_t> samples;
        uint32_t sr = 16000;
        uint16_t ch = 1;
        auto start = std::chrono::steady_clock::now();

        while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             start)
                   .count() < 10.0) {
                auto as = is_playback ? media->get_audio_playback_data()
                                      : media->get_audio_capture_data();
                auto &d = as.audio_data;
                if (!d.empty()) {
                        samples.insert(samples.end(), d.begin(), d.end());
                        sr = as.sample_rate;
                        ch = as.channels;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (samples.empty()) {
                std::cerr << "没录到音频!\n";
                return 1;
        }

        std::string out = out_path(filename);
        write_wav(out, samples, sr, ch);
        double sec = samples.size() / (double)(sr * ch);
        printf("[✓] 保存: %s (%.1f 秒, %u Hz, %u ch)\n", out.c_str(), sec, sr,
               ch);
        return 0;
}

static int cmd_capture_audio(MediaController *media) {
        return record_audio(media, "example_media_mic.wav", false);
}

static int cmd_playback_audio(MediaController *media) {
        return record_audio(media, "example_media_speaker.wav", true);
}

static std::string ensure_wav(const std::string &path,
                              int target_channels = -1) {
        // always convert to ensure correct channels
        std::string tmp =
            "/tmp/example_media_convert_" +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) +
            ".wav";
        std::string ac = target_channels > 0
                             ? " -ac " + std::to_string(target_channels)
                             : "";
        std::string cmd = "ffmpeg -y -i \"" + path +
                          "\" -acodec pcm_s16le -ar 16000" + ac + " \"" + tmp +
                          "\" 2>/dev/null";
        if (system(cmd.c_str()) != 0) {
                std::cerr << "ffmpeg 转换失败: " << path << "\n";
                return "";
        }
        return tmp;
}

static int cmd_external_audio_speaker(MediaController *media,
                                      const std::string &path) {
        // 无参数 = 实时麦克风
        if (path.empty()) {
                std::cout
                    << "[external_audio_speaker] 实时麦克风 → 机器人扬声器\n";
                std::cout << "  说话测试... 按 Ctrl+C 退出\n";

                const int ch = 2, sr = 16000;
                const int frame_n = sr * ch / 100;
                std::string tmp = "/tmp/example_media_live.wav";
                while (true) {
                        if (system(("arecord -D pulse -f S16_LE -r 16000 -c 2 "
                                    "-d 1 " +
                                    tmp + " 2>/dev/null")
                                       .c_str()) != 0)
                                break;
                        std::vector<int16_t> samples;
                        uint32_t rsr;
                        uint16_t rch;
                        if (!read_audio_file(tmp, samples, rsr, rch) ||
                            samples.empty())
                                continue;
                        for (size_t off = 0; off < samples.size();
                             off += frame_n) {
                                media::AudioStream as;
                                as.channels = rch;
                                as.sample_rate = rsr;
                                as.format = 2;
                                as.duration_ms = 10;
                                as.timestamp_us =
                                    std::chrono::duration_cast<
                                        std::chrono::microseconds>(
                                        std::chrono::system_clock::now()
                                            .time_since_epoch())
                                        .count();
                                size_t n = std::min<size_t>(
                                    frame_n, samples.size() - off);
                                as.audio_data.clear();
                                for (size_t j = 0; j < n; ++j) {
                                        int v = samples[off + j] * 3;
                                        as.audio_data.push_back(
                                            (int16_t)std::max(
                                                -32768, std::min(32767, v)));
                                }
                                media->publish_external_audio_playback_stream(
                                    as);
                                std::this_thread::sleep_for(
                                    std::chrono::milliseconds(10));
                        }
                }
                printf("\n  [✓] 完成\n");
                return 0;
        }

        // 播放文件
        std::cout << "[external_audio_speaker] 播放: " << path << "\n";
        std::string wav = ensure_wav(path, 2);
        if (wav.empty())
                return 1;
        std::vector<int16_t> samples;
        uint32_t sr;
        uint16_t ch;
        if (!read_audio_file(wav, samples, sr, ch)) {
                std::cerr << "读取 WAV 失败: " << wav << "\n";
                return 1;
        }

        int frame_samples = sr * ch / 100;
        for (size_t off = 0; off < samples.size(); off += frame_samples) {
                media::AudioStream as;
                as.channels = ch;
                as.sample_rate = sr;
                as.format = 2;
                as.duration_ms = 10;
                as.timestamp_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                size_t n =
                    std::min<size_t>(frame_samples, samples.size() - off);
                as.audio_data.assign(samples.begin() + off,
                                     samples.begin() + off + n);
                media->publish_external_audio_playback_stream(as);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        double dur = samples.size() / (double)(sr * ch);
        printf("[✓] 推送完成 (%.1f 秒)\n", dur);
        return 0;
}

static int cmd_external_audio_ai(MediaController *media,
                                 const std::string &path) {
        std::cout << "[external_audio_ai] 发送给 AI: " << path << "\n";
        std::string wav = ensure_wav(path, 8);
        if (wav.empty())
                return 1;
        std::vector<int16_t> samples;
        uint32_t sr;
        uint16_t ch;
        if (!read_audio_file(wav, samples, sr, ch)) {
                std::cerr << "读取 WAV 失败: " << wav << "\n";
                return 1;
        }

        media->wakeup();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        int frame_samples = sr * ch / 100;
        for (size_t off = 0; off < samples.size(); off += frame_samples) {
                media::AudioStream as;
                as.channels = ch;
                as.sample_rate = sr;
                as.format = 2;
                as.duration_ms = 10;
                as.timestamp_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                size_t n =
                    std::min<size_t>(frame_samples, samples.size() - off);
                as.audio_data.assign(samples.begin() + off,
                                     samples.begin() + off + n);
                media->publish_external_audio_stream(as);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        double dur = samples.size() / (double)(sr * ch);
        printf("[✓] 音频推送完成 (%.1f 秒), 等待 AI 响应...\n", dur);

        // 监控扬声器输出, 录 AI 回答
        std::cout << "  录制 AI 回答 (最长 10 秒)...\n";
        std::vector<int16_t> reply;
        uint32_t reply_sr = 0;
        uint16_t reply_ch = 0;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             start)
                   .count() < 10.0) {
                auto as = media->get_audio_playback_data();
                auto &d = as.audio_data;
                if (!d.empty()) {
                        reply.insert(reply.end(), d.begin(), d.end());
                        reply_sr = as.sample_rate;
                        reply_ch = as.channels;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (reply.empty()) {
                std::cout << "  未检测到 AI 语音回复\n";
        } else {
                if (reply_sr == 0)
                        reply_sr = 16000;
                if (reply_ch == 0)
                        reply_ch = 1;
                std::string fn = out_path("example_media_ai_reply.wav");
                write_wav(fn, reply, reply_sr, reply_ch);
                printf("[✓] AI 回复已保存: %s (%.1f 秒, %uHz %uch)\n",
                       fn.c_str(), reply.size() / (double)(reply_sr * reply_ch),
                       reply_sr, reply_ch);
        }

        media->sleep();
        return 0;
}

// ═══════════════════════════════════════════════════════
// Usage
// ═══════════════════════════════════════════════════════

static void usage() {
        printf(
            "Example Media Demo (C++, 无Qt) — Jetson Orin Nano + RealSense "
            "D435i\n\n"
            "用法: ./example_media <command> [args]\n\n"
            "命令:\n"
            "  capture_video              内部摄像头 → 抓一帧存 out/\n"
            "          ⚠ 算力板无机器人内部相机, 需运控板相机才能用\n"
            "  external_video [dev]        外接USB摄像头 → 自动检测 + "
            "连续推流\n"
            "          自动检测顺序: /dev/video4 → video2 → video0\n"
            "          按 Ctrl+C 停止, 自动保存最后一帧为 PNG\n"
            "  capture_audio               内部麦克风 → 录 WAV 存 out/\n"
            "  playback_audio              内部扬声器 → 录 WAV 存 out/\n"
            "  external_audio_speaker [wav] 本地WAV/实时麦克风 → 机器人扬声器\n"
            "  external_audio_ai <wav>      本地WAV → AI识别回答\n"
            "  desensed_video              脱敏视频 → 抓一帧存 out/\n\n"
            "所有输出文件保存在 out/ 目录下。\n");
}

// ═══════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════

int main(int argc, char **argv) {
        if (argc < 2) {
                usage();
                return 1;
        }

        signal(SIGINT, sigint_handler);

        auto sdk_root = find_sdk_root();
        auto config = sdk_root / "config" / "dds.xml";
        if (!std::filesystem::is_regular_file(config)) {
                std::cerr << "找不到 config/dds.xml\n";
                return 1;
        }
        setenv("CYCLONEDDS_URI",
               ("file://" + std::filesystem::absolute(config).string()).c_str(),
               1);

        auto *media = MediaController::Instance();
        if (!media->init()) {
                std::cerr << "MediaController init 失败\n";
                return 1;
        }

        std::string cmd = argv[1];

        // 仅在需要音频的命令中开启音频模块
        if (cmd == "external_audio_speaker" || cmd == "external_audio_ai") {
                media->set_internal_capture_audio_data_to_agent_enable(true);
                media->set_external_custom_audio_data_to_agent_enable(true);
                media->resume_audio_capture();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        if (cmd == "capture_video")
                return cmd_capture_video(media);
        if (cmd == "internal_video")
                return cmd_capture_video(media);
        if (cmd == "external_video") {
                std::string dev = argc > 2 ? argv[2] : "";
                return cmd_external_video(media, dev);
        }
        if (cmd == "capture_audio")
                return cmd_capture_audio(media);
        if (cmd == "playback_audio")
                return cmd_playback_audio(media);
        if (cmd == "external_audio_speaker") {
                std::string path = argc > 2 ? argv[2] : "";
                return cmd_external_audio_speaker(media, path);
        }
        if (cmd == "external_audio_ai") {
                if (argc < 3) {
                        printf("用法: %s external_audio_ai <wav>\n", argv[0]);
                        return 1;
                }
                return cmd_external_audio_ai(media, argv[2]);
        }
        if (cmd == "desensed_video")
                return cmd_desensed_video(media);

        usage();
        return 1;
}
