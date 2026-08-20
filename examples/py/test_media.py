import sys
import os
import struct
import subprocess
import threading
from time import sleep

import numpy as np

# DDS 环境配置
os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"

sys.path.append(os.path.abspath("./build"))
from mediacontrol_py import *


# ========== WAV helpers ==========


def read_wav(path):
    """Read a WAV file, return (pcm_int16_array, channels, sample_rate)."""
    import wave

    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        sample_rate = wf.getframerate()
        sampwidth = wf.getsampwidth()
        nframes = wf.getnframes()
        raw = wf.readframes(nframes)

    if sampwidth == 2:
        pcm = np.frombuffer(raw, dtype=np.int16)
    elif sampwidth == 1:
        pcm = (np.frombuffer(raw, dtype=np.uint8).astype(np.int16) - 128) * 256
    else:
        raise ValueError(f"unsupported sample width: {sampwidth}")

    return pcm, channels, sample_rate


def write_wav(path, pcm, channels, sample_rate):
    """Write PCM int16 data to WAV file."""
    import wave

    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(pcm.tobytes())
    print(f"[OK] saved {path} ({len(pcm)} samples, {channels}ch, {sample_rate}Hz)")


def convert_to_2ch_interleaved(pcm, in_channels):
    """Convert multi-channel PCM to 2-channel interleaved (take first 2 ch)."""
    if in_channels == 2:
        return pcm
    frames = len(pcm) // in_channels
    pcm_reshaped = pcm[: frames * in_channels].reshape(frames, in_channels)
    return pcm_reshaped[:, :2].flatten()


# ========== Commands ==========


def print_all_config(ctrl):
    """Print all non-stream config."""
    print("\n========== Media Config ==========")
    print(f"[Volume] {ctrl.get_volume()}")
    print(f"[Timeout] {ctrl.get_timeout()} ms")
    print(f"[AudioCueEnable] {ctrl.get_audio_cue_enable()}")
    print(
        f"[InternalCaptureAudio->Agent] {ctrl.get_internal_capture_audio_data_to_agent_enable()}"
    )
    print(
        f"[ExternalCustomAudio->Agent] {ctrl.get_external_custom_audio_data_to_agent_enable()}"
    )
    print(
        f"[InternalAgentAudio->Playback] {ctrl.get_internal_agent_audio_data_to_playback_enable()}"
    )
    print(
        f"[ExternalCustomAudio->Playback] {ctrl.get_external_custom_audio_data_to_playback_enable()}"
    )
    print(
        f"[InternalCaptureVideo->Agent] {ctrl.get_internal_capture_video_data_to_agent_enable()}"
    )
    print(
        f"[ExternalCustomVideo->Agent] {ctrl.get_external_custom_video_data_to_agent_enable()}"
    )
    print(
        f"[ExternalAudioUseInternal3A] {ctrl.get_external_custom_audio_data_to_agent_use_internal_3a()}"
    )
    print(f"[WakeupResponseWords] {ctrl.get_wakeup_response_words()}")
    print(f"[SleepResponseWords] {ctrl.get_sleep_response_words()}")
    print(f"[WakeupWords] {ctrl.get_wakeup_words()}")
    print("==================================\n")


def cmd_internal_audio_capture(ctrl):
    """Record 10s internal mic audio."""
    print("[CMD] internal_audio_capture: recording 10s...")
    lock = threading.Lock()
    pcm_all = []
    channels_ref = [0]
    sample_rate_ref = [0]

    def on_audio(stream):
        with lock:
            if channels_ref[0] == 0:
                channels_ref[0] = stream.channels
                sample_rate_ref[0] = stream.sample_rate
            pcm_all.extend(stream.audio_data)

    ctrl.subscribe_internal_audio_capture(on_audio)
    sleep(10)

    with lock:
        pcm = np.array(pcm_all, dtype=np.int16)
        ch = channels_ref[0]
        sr = sample_rate_ref[0]

    print(f"[INFO] captured {len(pcm)} samples ({ch}ch, {sr}Hz)")
    write_wav("out/internal_audio_capture.wav", pcm, ch, sr)


def cmd_internal_audio_playback(ctrl):
    """Record 10s internal speaker audio."""
    print("[CMD] internal_audio_playback: recording 10s...")
    lock = threading.Lock()
    pcm_all = []
    channels_ref = [0]
    sample_rate_ref = [0]

    def on_audio(stream):
        with lock:
            if channels_ref[0] == 0:
                channels_ref[0] = stream.channels
                sample_rate_ref[0] = stream.sample_rate
            pcm_all.extend(stream.audio_data)

    ctrl.subscribe_internal_audio_playback(on_audio)
    sleep(10)

    with lock:
        pcm = np.array(pcm_all, dtype=np.int16)
        ch = channels_ref[0]
        sr = sample_rate_ref[0]

    print(f"[INFO] captured {len(pcm)} samples ({ch}ch, {sr}Hz)")
    write_wav("out/internal_audio_playback.wav", pcm, ch, sr)


def cmd_external_audio_agent(ctrl, wav_path):
    """Send WAV audio to agent (wakeup first)."""
    ctrl.set_external_custom_audio_data_to_agent_enable(True)
    sleep(0.1)
    ctrl.set_internal_capture_audio_data_to_agent_enable(False)
    sleep(0.1)
    print(f"[CMD] external_audio_agent: {wav_path}")
    pcm, channels, sample_rate = read_wav(wav_path)
    print(
        f"[INFO] loaded {wav_path}: {len(pcm)} samples, {channels}ch, {sample_rate}Hz"
    )

    # wakeup agent first
    ctrl.set_system_control(SystemControlType.TO_WAKEUP, True)
    sleep(2)

    # send in ~20ms chunks
    frame_samples = sample_rate // 50 * channels  # 20ms
    if frame_samples == 0:
        frame_samples = 640

    offset = 0
    while offset < len(pcm):
        chunk = pcm[offset : offset + frame_samples]
        stream = AudioStream()
        stream.timestamp_us = int(time_us())
        stream.channels = channels
        stream.sample_rate = sample_rate
        stream.format = 2
        stream.duration_ms = len(chunk) // channels * 1000 // sample_rate
        stream.audio_data = chunk.tolist()
        ctrl.publish_external_audio_stream(stream)
        offset += frame_samples
        sleep(0.02)

    print("[CMD] external_audio_agent done")
    ctrl.set_external_custom_audio_data_to_agent_enable(False)
    sleep(0.1)
    ctrl.set_internal_capture_audio_data_to_agent_enable(True)


def cmd_external_audio_playback(ctrl, wav_path):
    """Play WAV to internal speaker (2-channel interleaved)."""
    print(f"[CMD] external_audio_playback: {wav_path}")
    pcm, channels, sample_rate = read_wav(wav_path)

    # convert to 2-channel interleaved
    pcm_2ch = convert_to_2ch_interleaved(pcm, channels)
    print(f"[INFO] converted to 2ch: {len(pcm_2ch)} samples")

    frame_samples = sample_rate // 50 * 2  # 20ms, 2ch
    offset = 0
    while offset < len(pcm_2ch):
        chunk = pcm_2ch[offset : offset + frame_samples]
        stream = AudioStream()
        stream.timestamp_us = int(time_us())
        stream.channels = 2
        stream.sample_rate = sample_rate
        stream.format = 2
        stream.duration_ms = len(chunk) // 2 * 1000 // sample_rate
        stream.audio_data = chunk.tolist()
        ctrl.publish_external_audio_playback_stream(stream)
        offset += frame_samples
        sleep(0.02)

    print("[CMD] external_audio_playback done")


def yuv422_to_rgb(yuv, w, h):
    """Convert YUV422 (YUYV) bytes to RGB numpy array."""
    yuv = np.frombuffer(yuv, dtype=np.uint8).reshape(-1, 4)
    y0 = yuv[:, 0].astype(np.int32)
    u = yuv[:, 1].astype(np.int32)
    y1 = yuv[:, 2].astype(np.int32)
    v = yuv[:, 3].astype(np.int32)

    c0 = y0 - 16
    c1 = y1 - 16
    d = u - 128
    e = v - 128

    def clamp(arr):
        return np.clip(arr, 0, 255).astype(np.uint8)

    r0 = clamp((298 * c0 + 409 * e + 128) >> 8)
    g0 = clamp((298 * c0 - 100 * d - 208 * e + 128) >> 8)
    b0 = clamp((298 * c0 + 516 * d + 128) >> 8)
    r1 = clamp((298 * c1 + 409 * e + 128) >> 8)
    g1 = clamp((298 * c1 - 100 * d - 208 * e + 128) >> 8)
    b1 = clamp((298 * c1 + 516 * d + 128) >> 8)

    rgb = np.empty((len(yuv), 6), dtype=np.uint8)
    rgb[:, 0] = r0
    rgb[:, 1] = g0
    rgb[:, 2] = b0
    rgb[:, 3] = r1
    rgb[:, 4] = g1
    rgb[:, 5] = b1
    return rgb.flatten()


def decode_and_save(stream, out_path):
    """Decode video stream to RGB, save as PNG via PIL or PPM+ffmpeg."""
    data = bytes(stream.video_data)
    w, h = stream.width, stream.height
    if not data or w == 0 or h == 0:
        return False

    expected_yuv422 = w * h * 2
    expected_rgb = w * h * 3

    if len(data) == expected_yuv422:
        rgb = yuv422_to_rgb(data, w, h)
    elif len(data) == expected_rgb:
        rgb = np.frombuffer(data, dtype=np.uint8)
    else:
        print(
            f"[WARN] unsupported video data size: {len(data)} "
            f"(expected {expected_yuv422} for YUV422 or {expected_rgb} for RGB)"
        )
        return False

    # try PIL/Pillow -> PNG
    try:
        from PIL import Image

        img = Image.frombytes("RGB", (w, h), rgb.tobytes())
        img.save(out_path)
        return True
    except ImportError:
        pass

    # fallback: write PPM, then ffmpeg -> PNG
    ppm = out_path.replace(".png", ".ppm")
    with open(ppm, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode())
        f.write(rgb.tobytes())

    ret = os.system(f"ffmpeg -y -i {ppm} {out_path} 2>/dev/null")
    if ret == 0:
        os.remove(ppm)
        return True
    print(f"[OK] saved {ppm} (PPM, ffmpeg not available)")
    return True


def save_video_frame(base, stream):
    """Save video frame as PNG."""
    os.makedirs("out", exist_ok=True)
    path = f"out/{base}.png"
    if decode_and_save(stream, path):
        print(f"[OK] saved {path} ({stream.width}x{stream.height})")
    else:
        print(f"[ERROR] failed to save frame {base}")


def cmd_internal_video_capture(ctrl):
    """Capture one internal video frame."""
    print("[CMD] internal_video_capture: waiting for frame...")
    got = threading.Event()
    result = [None]

    def on_video(stream):
        if got.is_set():
            return
        result[0] = stream
        got.set()

    ctrl.subscribe_internal_video_capture(on_video)
    got.wait(timeout=10)

    if result[0] is None:
        print("[WARN] no video frame received")
        return
    save_video_frame("internal_video_capture", result[0])


def cmd_internal_video_desensed(ctrl):
    """Capture one desensed video frame."""
    print("[CMD] internal_video_desensed: waiting for frame...")
    got = threading.Event()
    result = [None]

    def on_video(stream):
        if got.is_set():
            return
        result[0] = stream
        got.set()

    ctrl.subscribe_internal_video_desensed(on_video)
    got.wait(timeout=10)

    if result[0] is None:
        print("[WARN] no video frame received")
        return
    save_video_frame("internal_video_desensed", result[0])


def cmd_external_video_publish(ctrl):
    """Capture /dev/video4 via ffmpeg and publish."""
    # enable external video, disable internal video
    ctrl.set_external_custom_video_data_to_agent_enable(True)
    sleep(0.5)
    ctrl.set_internal_capture_video_data_to_agent_enable(False)
    sleep(0.5)

    def restore():
        ctrl.set_external_custom_video_data_to_agent_enable(False)
        sleep(0.5)
        ctrl.set_internal_capture_video_data_to_agent_enable(True)
        sleep(0.5)
        print("[CMD] config restored")

    if not os.path.exists("/dev/video4"):
        print("[ERROR] /dev/video4 not found")
        restore()
        return

    print("[CMD] external_video_publish: capturing /dev/video4...")
    print("[INFO] press Ctrl+C to stop")

    W, H, FPS = 640, 480, 30
    frame_size = W * H * 2  # YUYV: 2 bytes per pixel

    proc = subprocess.Popen(
        [
            "ffmpeg",
            "-f",
            "v4l2",
            "-input_format",
            "yuyv422",
            "-video_size",
            f"{W}x{H}",
            "-framerate",
            str(FPS),
            "-i",
            "/dev/video4",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "yuyv422",
            "pipe:1",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    count = 0
    try:
        while True:
            raw = proc.stdout.read(frame_size)
            if len(raw) < frame_size:
                break
            stream = VideoStream()
            stream.timestamp_us = int(time_us())
            stream.format = 1  # YUYV
            stream.width = W
            stream.height = H
            stream.fps = FPS
            stream.video_data = list(raw)
            ctrl.publish_external_video_stream(stream)
            count += 1
            if count % 30 == 0:
                print(f"[INFO] published {count} frames")
    except KeyboardInterrupt:
        pass
    finally:
        proc.stdout.close()
        proc.terminate()
        proc.wait()
        print(f"[CMD] external_video_publish done ({count} frames)")
        restore()


def time_us():
    import time

    return time.time() * 1e6


# ========== Main ==========


def print_usage(prog):
    print(f"Usage: {prog} [command] [args]")
    print("  (no args)                  - print all config")
    print("  internal_audio_capture     - record 10s internal mic")
    print("  internal_audio_playback    - record 10s internal speaker")
    print("  external_audio_agent <wav> - send wav to agent")
    print("  external_audio_playback <wav> - play wav to speaker")
    print("  internal_video_capture     - capture one video frame")
    print("  internal_video_desensed    - capture one desensed frame")
    print("  external_video_publish     - publish /dev/video4 via ffmpeg")


def main():
    ctrl = MediaController.instance()
    ctrl.init()

    args = sys.argv[1:]

    if len(args) == 0:
        print_all_config(ctrl)
        return

    cmd = args[0]

    if cmd == "internal_audio_capture":
        cmd_internal_audio_capture(ctrl)
    elif cmd == "internal_audio_playback":
        cmd_internal_audio_playback(ctrl)
    elif cmd == "external_audio_agent":
        if len(args) < 2:
            print("[ERROR] missing wav file")
            print_usage(sys.argv[0])
            sys.exit(1)
        cmd_external_audio_agent(ctrl, args[1])
    elif cmd == "external_audio_playback":
        if len(args) < 2:
            print("[ERROR] missing wav file")
            print_usage(sys.argv[0])
            sys.exit(1)
        cmd_external_audio_playback(ctrl, args[1])
    elif cmd == "internal_video_capture":
        cmd_internal_video_capture(ctrl)
    elif cmd == "internal_video_desensed":
        cmd_internal_video_desensed(ctrl)
    elif cmd == "external_video_publish":
        cmd_external_video_publish(ctrl)
    else:
        print(f"[ERROR] unknown command: {cmd}")
        print_usage(sys.argv[0])
        sys.exit(1)


if __name__ == "__main__":
    main()
