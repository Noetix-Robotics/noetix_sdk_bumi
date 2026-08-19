#include "MediaController.h"
#include <csignal>
#include <fstream>
#include <unistd.h>

using namespace noetix;

static volatile sig_atomic_t g_running = 1;

static void sigint_handler(int) { g_running = 0; }

// WAV file helpers
static void write_wav_header(std::ofstream &f, uint32_t channels,
                             uint32_t sample_rate, uint32_t bits_per_sample,
                             uint32_t data_bytes) {
        uint32_t byte_rate = sample_rate * channels * bits_per_sample / 8;
        uint16_t block_align = channels * bits_per_sample / 8;
        uint32_t chunk_size = 36 + data_bytes;

        f.write("RIFF", 4);
        f.write(reinterpret_cast<const char *>(&chunk_size), 4);
        f.write("WAVE", 4);
        f.write("fmt ", 4);

        uint32_t subchunk1_size = 16;
        uint16_t audio_format = 1; // PCM
        f.write(reinterpret_cast<const char *>(&subchunk1_size), 4);
        f.write(reinterpret_cast<const char *>(&audio_format), 2);
        f.write(reinterpret_cast<const char *>(&channels), 2);
        f.write(reinterpret_cast<const char *>(&sample_rate), 4);
        f.write(reinterpret_cast<const char *>(&byte_rate), 4);
        f.write(reinterpret_cast<const char *>(&block_align), 2);
        f.write(reinterpret_cast<const char *>(&bits_per_sample), 2);
        f.write("data", 4);
        f.write(reinterpret_cast<const char *>(&data_bytes), 4);
}

static void save_audio_to_wav(const std::string &path,
                              const std::vector<int16_t> &pcm, uint32_t channels,
                              uint32_t sample_rate) {
        uint32_t data_bytes = pcm.size() * sizeof(int16_t);
        std::ofstream f(path, std::ios::binary);
        if (!f.is_open()) {
                printf("[ERROR] cannot open %s\n", path.c_str());
                return;
        }
        write_wav_header(f, channels, sample_rate, 16, data_bytes);
        f.write(reinterpret_cast<const char *>(pcm.data()), data_bytes);
        f.close();
        printf("[OK] saved %s (%u samples, %u ch, %u Hz)\n", path.c_str(),
               (unsigned)pcm.size(), channels, sample_rate);
}

// Parse WAV file, return raw PCM samples. Sets channels/sample_rate on output.
static bool load_wav(const std::string &path, std::vector<int16_t> &pcm,
                     uint32_t &channels, uint32_t &sample_rate) {
        std::ifstream f(path, std::ios::binary);
        if (!f.is_open()) {
                printf("[ERROR] cannot open %s\n", path.c_str());
                return false;
        }

        // Read RIFF header
        char tag[4];
        f.read(tag, 4);
        if (memcmp(tag, "RIFF", 4) != 0) {
                printf("[ERROR] not a WAV file\n");
                return false;
        }
        f.ignore(4); // file size
        f.read(tag, 4);
        if (memcmp(tag, "WAVE", 4) != 0) {
                printf("[ERROR] not a WAV file\n");
                return false;
        }

        uint32_t data_bytes = 0;
        bool found_fmt = false, found_data = false;

        while (f.good() && !found_data) {
                f.read(tag, 4);
                uint32_t chunk_size;
                f.read(reinterpret_cast<char *>(&chunk_size), 4);

                if (memcmp(tag, "fmt ", 4) == 0) {
                        uint16_t audio_format, ch, bits;
                        uint32_t sr;
                        f.read(reinterpret_cast<char *>(&audio_format), 2);
                        f.read(reinterpret_cast<char *>(&ch), 2);
                        f.read(reinterpret_cast<char *>(&sr), 4);
                        f.ignore(4); // byte_rate
                        f.ignore(2); // block_align
                        f.read(reinterpret_cast<char *>(&bits), 2);
                        if (chunk_size > 16)
                                f.ignore(chunk_size - 16);
                        if (audio_format != 1) {
                                printf("[ERROR] unsupported format %u (need PCM)\n",
                                       audio_format);
                                return false;
                        }
                        channels = ch;
                        sample_rate = sr;
                        found_fmt = true;
                } else if (memcmp(tag, "data", 4) == 0) {
                        data_bytes = chunk_size;
                        found_data = true;
                } else {
                        f.ignore(chunk_size);
                }
        }

        if (!found_fmt || !found_data) {
                printf("[ERROR] invalid WAV file\n");
                return false;
        }

        size_t num_samples = data_bytes / sizeof(int16_t);
        pcm.resize(num_samples);
        f.read(reinterpret_cast<char *>(pcm.data()), data_bytes);
        return true;
}

// Convert multi-channel to 2-channel interleaved (take first 2 channels)
static void convert_to_2ch_interleaved(const std::vector<int16_t> &in,
                                       uint32_t in_channels,
                                       std::vector<int16_t> &out) {
        if (in_channels == 2) {
                out = in;
                return;
        }
        size_t frames = in.size() / in_channels;
        out.resize(frames * 2);
        for (size_t i = 0; i < frames; i++) {
                out[i * 2] = in[i * in_channels];
                out[i * 2 + 1] = in[i * in_channels + 1];
        }
}

// Print all non-stream configs
static void print_all_config(MediaController *ctrl) {
        printf("\n========== Media Config ==========\n");
        printf("[Volume] %d\n", ctrl->get_volume());
        printf("[Timeout] %d ms\n", ctrl->get_timeout());
        printf("[AudioCueEnable] %d\n", ctrl->get_audio_cue_enable());
        printf("[InternalCaptureAudio->Agent] %d\n",
               ctrl->get_internal_capture_audio_data_to_agent_enable());
        printf("[ExternalCustomAudio->Agent] %d\n",
               ctrl->get_external_custom_audio_data_to_agent_enable());
        printf("[InternalAgentAudio->Playback] %d\n",
               ctrl->get_internal_agent_audio_data_to_playback_enable());
        printf("[ExternalCustomAudio->Playback] %d\n",
               ctrl->get_external_custom_audio_data_to_playback_enable());
        printf("[InternalCaptureVideo->Agent] %d\n",
               ctrl->get_internal_capture_video_data_to_agent_enable());
        printf("[ExternalCustomVideo->Agent] %d\n",
               ctrl->get_external_custom_video_data_to_agent_enable());
        printf("[ExternalAudioUseInternal3A] %d\n",
               ctrl->get_external_custom_audio_data_to_agent_use_internal_3a());
        printf("[WakeupResponseWords] %s\n",
               ctrl->get_wakeup_response_words().c_str());
        printf("[SleepResponseWords] %s\n",
               ctrl->get_sleep_response_words().c_str());
        printf("[WakeupWords]\n%s\n", ctrl->get_wakeup_words().c_str());
        printf("==================================\n\n");
}

// --- 2. internal_audio_capture: record 10s internal mic ---
static void cmd_internal_audio_capture(MediaController *ctrl) {
        printf("[CMD] internal_audio_capture: recording 10s...\n");
        std::mutex mtx;
        std::vector<int16_t> pcm;
        uint32_t channels = 0, sample_rate = 0;
        std::atomic<bool> done{false};

        ctrl->subscribe_internal_audio_capture(
            [&](const media::AudioStream &stream) {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (channels == 0) {
                            channels = stream.channels;
                            sample_rate = stream.sample_rate;
                    }
                    pcm.insert(pcm.end(), stream.audio_data.begin(),
                               stream.audio_data.end());
            });

        // collect for 10 seconds
        for (int i = 0; i < 20; i++)
                sleep_ms(500);

        std::lock_guard<std::mutex> lk(mtx);
        printf("[CMD] captured %zu samples (%u ch, %u Hz)\n", pcm.size(),
               channels, sample_rate);
        system("mkdir -p out");
        save_audio_to_wav("out/internal_audio_capture.wav", pcm, channels,
                          sample_rate);
}

// --- 3. internal_audio_playback: record 10s internal speaker ---
static void cmd_internal_audio_playback(MediaController *ctrl) {
        printf("[CMD] internal_audio_playback: recording 10s...\n");
        std::mutex mtx;
        std::vector<int16_t> pcm;
        uint32_t channels = 0, sample_rate = 0;

        ctrl->subscribe_internal_audio_playback(
            [&](const media::AudioStream &stream) {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (channels == 0) {
                            channels = stream.channels;
                            sample_rate = stream.sample_rate;
                    }
                    pcm.insert(pcm.end(), stream.audio_data.begin(),
                               stream.audio_data.end());
            });

        for (int i = 0; i < 20; i++)
                sleep_ms(500);

        std::lock_guard<std::mutex> lk(mtx);
        printf("[CMD] captured %zu samples (%u ch, %u Hz)\n", pcm.size(),
               channels, sample_rate);
        system("mkdir -p out");
        save_audio_to_wav("out/internal_audio_playback.wav", pcm, channels,
                          sample_rate);
}

// --- 4. external_audio_agent: play wav to agent (wakeup first) ---
static void cmd_external_audio_agent(MediaController *ctrl,
                                     const std::string &wav_path) {
	ctrl->set_external_custom_audio_data_to_agent_enable(true);
	sleep_ms(100);
	ctrl->set_internal_capture_audio_data_to_agent_enable(false);
	sleep_ms(100);
        printf("[CMD] external_audio_agent: %s\n", wav_path.c_str());

        std::vector<int16_t> pcm;
        uint32_t channels = 0, sample_rate = 0;
        if (!load_wav(wav_path, pcm, channels, sample_rate))
                return;
        printf("[INFO] loaded %s: %zu samples, %u ch, %u Hz\n",
               wav_path.c_str(), pcm.size(), channels, sample_rate);

        // wakeup agent first
        ctrl->set_system_control(media::SystemControlType::TO_WAKEUP, true);
        sleep_ms(2000);

        // send in chunks (~20ms per frame at 16kHz = 320 samples/ch)
        uint32_t frame_samples = sample_rate / 50 * channels; // 20ms
        if (frame_samples == 0)
                frame_samples = 640;

        size_t offset = 0;
        while (offset < pcm.size()) {
                size_t chunk = std::min((size_t)frame_samples, pcm.size() - offset);
                media::AudioStream stream;
                stream.timestamp_us = get_time_us();
                stream.channels = channels;
                stream.sample_rate = sample_rate;
                stream.format = 2; // 16-bit LE
                stream.duration_ms = chunk / channels * 1000 / sample_rate;
                stream.audio_data.assign(pcm.begin() + offset,
                                         pcm.begin() + offset + chunk);
                ctrl->publish_external_audio_stream(stream);
                offset += chunk;
                sleep_ms(20);
        }
        printf("[CMD] external_audio_agent done\n");
	ctrl->set_internal_capture_audio_data_to_agent_enable(true);
	sleep_ms(100);
	ctrl->set_external_custom_audio_data_to_agent_enable(false);
}

// --- 5. external_audio_playback: play wav to internal speaker (2ch) ---
static void cmd_external_audio_playback(MediaController *ctrl,
                                        const std::string &wav_path) {
        printf("[CMD] external_audio_playback: %s\n", wav_path.c_str());

        std::vector<int16_t> pcm;
        uint32_t channels = 0, sample_rate = 0;
        if (!load_wav(wav_path, pcm, channels, sample_rate))
                return;

        // convert to 2-channel interleaved
        std::vector<int16_t> pcm_2ch;
        convert_to_2ch_interleaved(pcm, channels, pcm_2ch);
        printf("[INFO] converted to 2ch: %zu samples\n", pcm_2ch.size());

        uint32_t frame_samples = sample_rate / 50 * 2; // 20ms, 2ch
        size_t offset = 0;
        while (offset < pcm_2ch.size()) {
                size_t chunk =
                    std::min((size_t)frame_samples, pcm_2ch.size() - offset);
                media::AudioStream stream;
                stream.timestamp_us = get_time_us();
                stream.channels = 2;
                stream.sample_rate = sample_rate;
                stream.format = 2;
                stream.duration_ms = chunk / 2 * 1000 / sample_rate;
                stream.audio_data.assign(pcm_2ch.begin() + offset,
                                         pcm_2ch.begin() + offset + chunk);
                ctrl->publish_external_audio_playback_stream(stream);
                offset += chunk;
                sleep_ms(20);
        }
        printf("[CMD] external_audio_playback done\n");
}

// YUV422 (YUYV) to RGB conversion
static void yuv422_to_rgb(const uint8_t *yuv, uint8_t *rgb, int w, int h) {
        for (int i = 0; i < w * h / 2; i++) {
                int y0 = yuv[4 * i + 0];
                int u  = yuv[4 * i + 1];
                int y1 = yuv[4 * i + 2];
                int v  = yuv[4 * i + 3];
                int c0 = y0 - 16, c1 = y1 - 16;
                int d = u - 128, e = v - 128;
                auto clamp = [](int val) -> uint8_t {
                        return (uint8_t)(val < 0 ? 0 : (val > 255 ? 255 : val));
                };
                rgb[6 * i + 0] = clamp((298 * c0 + 409 * e + 128) >> 8);
                rgb[6 * i + 1] = clamp((298 * c0 - 100 * d - 208 * e + 128) >> 8);
                rgb[6 * i + 2] = clamp((298 * c0 + 516 * d + 128) >> 8);
                rgb[6 * i + 3] = clamp((298 * c1 + 409 * e + 128) >> 8);
                rgb[6 * i + 4] = clamp((298 * c1 - 100 * d - 208 * e + 128) >> 8);
                rgb[6 * i + 5] = clamp((298 * c1 + 516 * d + 128) >> 8);
        }
}

// Write PPM (P6) file
static bool write_ppm(const std::string &path, const uint8_t *rgb, int w,
                      int h) {
        std::ofstream f(path, std::ios::binary);
        if (!f.is_open())
                return false;
        f << "P6\n" << w << " " << h << "\n255\n";
        f.write(reinterpret_cast<const char *>(rgb), w * h * 3);
        f.close();
        return true;
}

// Decode video stream to RGB, then save as PNG via PPM+ffmpeg
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
                // already RGB
                std::copy(data.begin(), data.end(), rgb.begin());
        } else {
                printf("[WARN] unsupported video data size: %zu "
                       "(expected %d for YUV422 or %d for RGB)\n",
                       data.size(), w * h * 2, w * h * 3);
                return false;
        }

        // write PPM, then convert to PNG via ffmpeg
        std::string ppm = out_png;
        ppm.replace(ppm.rfind(".png"), 4, ".ppm");
        if (!write_ppm(ppm, rgb.data(), w, h))
                return false;

        std::string cmd = "ffmpeg -y -i " + ppm + " " + out_png +
                          " 2>/dev/null";
        if (system(cmd.c_str()) == 0) {
                remove(ppm.c_str());
                return true;
        }
        // ffmpeg failed, keep PPM
        printf("[OK] saved %s (PPM, ffmpeg not available)\n", ppm.c_str());
        return true;
}

static void save_video_frame(const std::string &base,
                             const media::VideoStream &stream) {
        system("mkdir -p out");
        std::string png_path = "out/" + base + ".png";
        if (decode_and_save(stream, png_path)) {
                printf("[OK] saved %s (%ux%u)\n", png_path.c_str(),
                       stream.width, stream.height);
        } else {
                printf("[ERROR] failed to save frame %s\n", base.c_str());
        }
}

// --- 6. internal_video_capture: capture one frame ---
static void cmd_internal_video_capture(MediaController *ctrl) {
        printf("[CMD] internal_video_capture: waiting for frame...\n");
        std::atomic<bool> got{false};

        ctrl->subscribe_internal_video_capture(
            [&](const media::VideoStream &stream) {
                    printf("[DEBUG] video frame: %ux%u format=%u data=%zu bytes\n",
                           stream.width, stream.height, stream.format,
                           stream.video_data.size());
                    if (got.exchange(true))
                            return;
                    save_video_frame("internal_video_capture", stream);
            });

        for (int i = 0; i < 20 && !got; i++)
                sleep_ms(500);
        if (!got)
                printf("[WARN] no video frame received\n");
}

// --- 7. internal_video_desensed: capture one desensed frame ---
static void cmd_internal_video_desensed(MediaController *ctrl) {
        printf("[CMD] internal_video_desensed: waiting for frame...\n");
        std::atomic<bool> got{false};

        ctrl->subscribe_internal_video_desensed(
            [&](const media::VideoStream &stream) {
                    printf("[DEBUG] desensed frame: %ux%u format=%u data=%zu bytes\n",
                           stream.width, stream.height, stream.format,
                           stream.video_data.size());
                    if (got.exchange(true))
                            return;
                    save_video_frame("internal_video_desensed", stream);
            });

        for (int i = 0; i < 20 && !got; i++)
                sleep_ms(500);
        if (!got)
                printf("[WARN] no video frame received\n");
}

// --- 8. external_video_publish: capture /dev/video4 via ffmpeg, publish ---
static void cmd_external_video_publish(MediaController *ctrl) {
        // enable external video, disable internal video
        ctrl->set_external_custom_video_data_to_agent_enable(true);
        sleep_ms(500);
        ctrl->set_internal_capture_video_data_to_agent_enable(false);
        sleep_ms(500);

        // restore config on exit (normal or Ctrl+C)
        auto restore = [&]() {
                ctrl->set_external_custom_video_data_to_agent_enable(false);
                sleep_ms(500);
                ctrl->set_internal_capture_video_data_to_agent_enable(true);
                sleep_ms(500);
                printf("[CMD] config restored\n");
        };

        signal(SIGINT, sigint_handler);
        g_running = 1;

        // first check if device exists
        if (access("/dev/video4", F_OK) != 0) {
                printf("[ERROR] /dev/video4 not found\n");
                restore();
                return;
        }

        printf("[CMD] external_video_publish: capturing /dev/video4...\n");
        printf("[INFO] press Ctrl+C to stop\n");

        const int W = 640, H = 480;
        const int frame_size = W * H * 2; // YUYV: 2 bytes per pixel

        // ffmpeg: capture YUYV from device, output raw YUYV to pipe
        const char *cmd =
            "ffmpeg -f v4l2 -input_format yuyv422 "
            "-video_size 640x480 -framerate 30 "
            "-i /dev/video4 -f rawvideo -pix_fmt yuyv422 pipe:1";

        FILE *pipe = popen(cmd, "r");
        if (!pipe) {
                printf("[ERROR] failed to start ffmpeg\n");
                restore();
                return;
        }

        std::vector<uint8_t> buf(frame_size);
        int count = 0;

        while (g_running) {
                size_t n = fread(buf.data(), 1, frame_size, pipe);
                if (n != (size_t)frame_size)
                        break;
                media::VideoStream stream;
                stream.timestamp_us = get_time_us();
                stream.format = 1; // YUYV
                stream.width = W;
                stream.height = H;
                stream.fps = 30;
                stream.video_data = buf;
                ctrl->publish_external_video_stream(stream);
                count++;
                if (count % 30 == 0)
                        printf("[INFO] published %d frames\n", count);
        }

        pclose(pipe);
        printf("[CMD] external_video_publish done (%d frames)\n", count);
        restore();
}

static void print_usage(const char *prog) {
        printf("Usage: %s [command] [args]\n", prog);
        printf("  (no args)                  - print all config\n");
        printf("  internal_audio_capture     - record 10s internal mic\n");
        printf("  internal_audio_playback    - record 10s internal speaker\n");
        printf("  external_audio_agent <wav> - send wav to agent\n");
        printf("  external_audio_playback <wav> - play wav to speaker\n");
        printf("  internal_video_capture     - capture one video frame\n");
        printf("  internal_video_desensed    - capture one desensed frame\n");
        printf("  external_video_publish     - publish /dev/video4 via ffmpeg\n");
}

int main(int argc, char *argv[]) {
        char buf[256];
        getcwd(buf, sizeof(buf));
        std::string path = std::string(buf);
        std::string ddsxml = "file://" + path + "/config/dds.xml";
        setenv("CYCLONEDDS_URI", ddsxml.c_str(), 1);
        printf("cur path is %s\n", path.c_str());

        MediaController *ctrl = MediaController::Instance();
        ctrl->init();

        if (argc < 2) {
                print_all_config(ctrl);
                return 0;
        }

        std::string cmd = argv[1];

        if (cmd == "internal_audio_capture") {
                cmd_internal_audio_capture(ctrl);
        } else if (cmd == "internal_audio_playback") {
                cmd_internal_audio_playback(ctrl);
        } else if (cmd == "external_audio_agent") {
                if (argc < 3) {
                        printf("[ERROR] missing wav file\n");
                        print_usage(argv[0]);
                        return 1;
                }
                cmd_external_audio_agent(ctrl, argv[2]);
        } else if (cmd == "external_audio_playback") {
                if (argc < 3) {
                        printf("[ERROR] missing wav file\n");
                        print_usage(argv[0]);
                        return 1;
                }
                cmd_external_audio_playback(ctrl, argv[2]);
        } else if (cmd == "internal_video_capture") {
                cmd_internal_video_capture(ctrl);
        } else if (cmd == "internal_video_desensed") {
                cmd_internal_video_desensed(ctrl);
        } else if (cmd == "external_video_publish") {
                cmd_external_video_publish(ctrl);
        } else {
                printf("[ERROR] unknown command: %s\n", cmd.c_str());
                print_usage(argv[0]);
                return 1;
        }

        return 0;
}
