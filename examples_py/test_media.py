#!/usr/bin/env python3
import sys, os, time, struct, wave, numpy as np
from io import BytesIO
from pathlib import Path
from PIL import Image

SDK_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = SDK_ROOT / "out"
OUT_DIR.mkdir(exist_ok=True)
os.environ["CYCLONEDDS_URI"] = "file://" + str(SDK_ROOT / "config" / "dds.xml")
sys.path.insert(0, str(SDK_ROOT / "build"))
from mediacontrol_py import MediaController, VideoStream, AudioStream


def decode_video_frame(data, width, height):
    if len(data) < 100:
        return None
    try:
        return np.array(Image.open(BytesIO(data)).convert("RGB"))
    except:
        pass
    if len(data) == width * height * 2:
        try:
            return yuv422_to_rgb(data, width, height)
        except:
            pass
    if len(data) == width * height * 3:
        try:
            return np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        except:
            pass
    return None


def yuv422_to_rgb(data, width, height):
    yuv = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 2))
    y = yuv[:, :, 0].astype(np.float32)
    u = yuv[:, ::2, 1].astype(np.float32)
    v = yuv[:, 1::2, 1].astype(np.float32)
    u = np.repeat(np.repeat(u, 2, axis=1), 1, axis=0)[:height, :width]
    v = np.repeat(np.repeat(v, 2, axis=1), 1, axis=0)[:height, :width]
    y -= 16
    u -= 128
    v -= 128
    r = (y + 1.402 * v).clip(0, 255).astype(np.uint8)
    g = (y - 0.344136 * u - 0.714136 * v).clip(0, 255).astype(np.uint8)
    b = (y + 1.772 * u).clip(0, 255).astype(np.uint8)
    return np.stack([r, g, b], axis=2)


def out_path(fn):
    return str(OUT_DIR / fn)


def frame_to_yuyv(frame):
    h, w = frame.shape[:2]
    ch = frame.shape[2] if len(frame.shape) > 2 else 1
    out = np.zeros((h, w, 2), dtype=np.uint8)
    if ch == 1:
        out[:, :, 0] = frame
        out[:, :, 1] = 128
    else:
        import cv2

        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        out[:, 0::2, 0] = yuv[:, 0::2, 0]
        out[:, 0::2, 1] = yuv[:, 0::2, 1]
        out[:, 1::2, 0] = yuv[:, 1::2, 0]
        out[:, 1::2, 1] = yuv[:, 1::2, 2]
    return out.tobytes()


def publish_video(media, camera_id):
    import cv2

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"[!] Cannot open camera {camera_id}")
        for d in range(10):
            p = f"/dev/video{d}"
            if os.path.exists(p):
                print(f"    {p}")
        return 1
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[external_video] {w}x{h}")
    media.set_external_custom_video_data_to_agent_enable(True)
    time.sleep(0.2)
    print(f"\n  Streaming {w}x{h} ... Ctrl+C to stop\n")
    fc = 0
    last = None
    t0 = time.time()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if fc == 0:
                    time.sleep(0.5)
                    continue
                break
            last = frame.copy()
            yuv = frame_to_yuyv(frame)
            vs = VideoStream()
            vs.width = w
            vs.height = h
            vs.format = 0
            vs.fps = 30
            vs.timestamp_us = int(time.time() * 1e6)
            vs.video_data = list(yuv)
            media.publish_external_video_stream(vs)
            fc += 1
            if fc % 30 == 0:
                s = time.time() - t0
                print(f"\r  {fc} frames ({fc / s:.1f} fps)", end="", flush=True)
    except KeyboardInterrupt:
        pass
    cap.release()
    print("\n\n  Stopped.")
    if last is not None:
        path = out_path("external_video.png")
        rgb = cv2.cvtColor(last, cv2.COLOR_BGR2RGB)
        Image.fromarray(rgb).save(path)
        print(f"  [OK] {path} ({w}x{h})")
    print(f"  Done: {fc} frames")
    return 0


def record_audio(media, is_playback):
    what = "扬声器" if is_playback else "麦克风"
    fn = "bumi_speaker.wav" if is_playback else "bumi_mic.wav"
    print(f"[{what}] 等待机器人恢复语音 (最长 10 秒)...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        a = (
            media.get_audio_playback_data()
            if is_playback
            else media.get_audio_capture_data()
        )
        if a.audio_data:
            break
        time.sleep(0.1)
    print(f"[{what}] 录制 10 秒...")
    samples = []
    sr, ch = 16000, 8
    t0 = time.time()
    while time.time() - t0 < 10.0:
        a = (
            media.get_audio_playback_data()
            if is_playback
            else media.get_audio_capture_data()
        )
        if a.audio_data:
            samples.extend(a.audio_data)
            sr, ch = a.sample_rate, a.channels
        time.sleep(0.005)
    if not samples:
        print(f"[!] 没录到音频")
        return False
    path = out_path(fn)
    with wave.open(path, "w") as wf:
        wf.setnchannels(ch)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(struct.pack(f"<{len(samples)}h", *samples))
    dur = len(samples) / (sr * ch)
    print(f"[OK] {path} ({dur:.1f}s, {sr}Hz, {ch}ch)")
    return True


def internal_video(media):
    """Capture from control board internal camera via DDS"""
    print("[internal_video] capture from DDS...")
    media.set_internal_capture_video_data_to_agent_enable(True)
    time.sleep(2)
    t0 = time.time()
    while time.time() - t0 < 5:
        vs = media.get_video_capture_data()
        data = bytes(vs.video_data)
        if data and vs.width > 0:
            frame = decode_video_frame(data, vs.width, vs.height)
            if frame is not None:
                path = out_path("internal_video.png")
                Image.fromarray(frame).save(path)
                print(f"  [OK] {path} ({vs.width}x{vs.height})")
                return True
        time.sleep(0.1)
    print("  [!] No frame. Does control board have a camera?")
    return False


def desensed_video(media):
    """Read desensed frames from DDS topic (requires external_video running)"""
    print("[desensed_video] read from DDS desensed topic...")
    media.set_internal_capture_video_data_to_agent_enable(True)
    time.sleep(2)
    t0 = time.time()
    while time.time() - t0 < 5:
        vs = media.get_video_capture_desensed_data()
        data = bytes(vs.video_data)
        if data and vs.width > 0:
            frame = decode_video_frame(data, vs.width, vs.height)
            if frame is not None:
                path = out_path("desensed_video.png")
                Image.fromarray(frame).save(path)
                print(f"  [OK] {path} ({vs.width}x{vs.height})")
                return True
        time.sleep(0.1)
    print("  [!] No desensed frame. Is external_video streaming?")
    return False


def publish_audio_speaker(media, path):
    """Play WAV through robot speaker"""
    if not path:
        print("[external_audio_speaker] needs a WAV file argument")
        return 1
    print(f"[external_audio_speaker] {path}")
    sr, ch = 16000, 2
    samples = []
    # Always convert to 2ch 16kHz via ffmpeg
    import tempfile, subprocess

    wav = tempfile.mktemp(suffix=".wav")
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            path,
            "-acodec",
            "pcm_s16le",
            "-ar",
            "16000",
            "-ac",
            "2",
            wav,
        ],
        capture_output=True,
    )
    with open(wav, "rb") as f2:
        data = f2.read()
    fmt_off = data.find(b"fmt ")
    data_off = data.find(b"data")
    import struct

    ch = struct.unpack_from("<H", data, fmt_off + 10)[0]
    sr = struct.unpack_from("<I", data, fmt_off + 12)[0]
    dsz = struct.unpack_from("<I", data, data_off + 4)[0]
    raw = data[data_off + 8 : data_off + 8 + dsz]
    samples = list(struct.unpack(f"<{len(raw) // 2}h", raw))
    media.wakeup()
    time.sleep(1)
    frame_n = sr * ch // 100
    for off in range(0, len(samples), frame_n):
        chunk = samples[off : off + frame_n]
        s = AudioStream()
        s.channels = ch
        s.sample_rate = sr
        s.format = 2
        s.duration_ms = 10
        s.timestamp_us = int(time.time() * 1e6)
        s.audio_data = chunk
        media.publish_external_audio_playback_stream(s)
        time.sleep(0.01)
    dur = len(samples) / (sr * ch)
    print(f"[OK] played {dur:.1f}s")
    return 0


def publish_audio_ai(media, path):
    import tempfile, subprocess

    print(f"[external_audio_ai] {path}")
    # Convert to WAV if needed
    wav = path
    if not path.lower().endswith(".wav"):
        wav = tempfile.mktemp(suffix=".wav")
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-i",
                path,
                "-acodec",
                "pcm_s16le",
                "-ar",
                "16000",
                "-ac",
                "8",
                wav,
            ],
            capture_output=True,
        )
    # Read WAV
    with open(wav, "rb") as f:
        data = f.read()
    fmt_off = data.find(b"fmt ")
    data_off = data.find(b"data")
    if fmt_off < 0 or data_off < 0:
        print("Invalid WAV")
        return 1
    import struct

    nch = struct.unpack_from("<H", data, fmt_off + 10)[0]
    sr = struct.unpack_from("<I", data, fmt_off + 12)[0]
    dsz = struct.unpack_from("<I", data, data_off + 4)[0]
    raw = data[data_off + 8 : data_off + 8 + dsz]
    samples = list(struct.unpack(f"<{len(raw) // 2}h", raw))
    # Wake up and push
    media.wakeup()
    time.sleep(1.0)
    frame_n = sr * nch // 100
    for off in range(0, len(samples), frame_n):
        chunk = samples[off : off + frame_n]
        s = AudioStream()
        s.channels = nch
        s.sample_rate = sr
        s.format = 2
        s.duration_ms = 10
        s.timestamp_us = int(time.time() * 1e6)
        s.audio_data = chunk
        media.publish_external_audio_stream(s)
        time.sleep(0.01)
    dur = len(samples) / (sr * nch)
    print(f"[OK] pushed {dur:.1f}s, waiting AI reply (10s)...")
    # Record reply
    reply = []
    reply_sr = 0
    reply_ch = 0
    t0 = time.time()
    while time.time() - t0 < 10:
        a = media.get_audio_playback_data()
        if a.audio_data:
            reply.extend(a.audio_data)
            reply_sr = a.sample_rate
            reply_ch = a.channels
        time.sleep(0.01)
    if not reply:
        print("  No AI reply")
    else:
        if reply_sr == 0:
            reply_sr = 16000
        if reply_ch == 0:
            reply_ch = 1
        fn = out_path("bumi_ai_reply.wav")
        with wave.open(fn, "w") as wf:
            wf.setnchannels(reply_ch)
            wf.setsampwidth(2)
            wf.setframerate(reply_sr)
            wf.writeframes(struct.pack(f"<{len(reply)}h", *reply))
        print(f"[OK] {fn} ({len(reply) / (reply_sr * reply_ch):.1f}s)")
    media.sleep()
    return 0


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 test_media.py external_video [cam_id]")
        print("  python3 test_media.py desensed_video")
        print("  python3 test_media.py playback_audio")
        print("  python3 test_media.py internal_video")
        print("  python3 test_media.py capture_audio")
        print("  python3 test_media.py external_audio_ai <wav>")
        print("  python3 test_media.py external_audio_speaker <wav>")
        return 1
    cmd = sys.argv[1]
    media = MediaController.instance()
    if not media.init():
        print("init failed")
        return 1

    if cmd in ("external_audio_speaker", "external_audio_ai"):
        media.set_internal_capture_audio_data_to_agent_enable(True)
        media.set_external_custom_audio_data_to_agent_enable(True)
        media.resume_audio_capture()
        time.sleep(0.2)

    if cmd == "external_video":
        cam = int(sys.argv[2]) if len(sys.argv) > 2 else 4
        return publish_video(media, cam)
    elif cmd == "playback_audio":
        return 0 if record_audio(media, True) else 1
    elif cmd == "capture_audio":
        return 0 if record_audio(media, False) else 1

    elif cmd == "internal_video":
        return internal_video(media)
    elif cmd == "desensed_video":
        return desensed_video(media)
    elif cmd == "external_audio_speaker":
        return publish_audio_speaker(media, sys.argv[2] if len(sys.argv) > 2 else "")
    elif cmd == "external_audio_ai":
        return publish_audio_ai(media, sys.argv[2])
    print(f"Unknown: {cmd}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
