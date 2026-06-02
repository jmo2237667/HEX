# video_stream.py
# ============================================================================
# Thread-Safe Flask FPV MJPEG Video Server
# ============================================================================
#
# Architecture:
#   - A dedicated daemon thread captures frames from the USB camera and
#     writes JPEG-encoded bytes into a shared buffer protected by a Lock.
#   - The Flask MJPEG generator reads from that buffer under the same Lock.
#   - The Flask/Werkzeug server itself runs on a separate daemon thread.
#
# This design ensures the camera is touched by exactly one thread, and
# the frame buffer access is serialised via threading.Lock.
# ============================================================================

import time
import threading
import cv2
from flask import Flask, Response
import config

app = Flask(__name__)

_frame_lock = threading.Lock()
_latest_frame = None
_capture_started = False


def _camera_capture_loop():
    """
    Background camera capture loop.  Runs on its own daemon thread.
    Grabs frames from the USB camera, encodes to JPEG, and stores
    in a thread-safe buffer that the Flask generator reads from.
    """
    global _latest_frame

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 15)

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 70]

    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(0.05)
            continue

        ret, buf = cv2.imencode('.jpg', frame, encode_params)
        if ret:
            with _frame_lock:
                _latest_frame = buf.tobytes()


def _ensure_capture_started():
    """Start the camera capture thread if not already running."""
    global _capture_started
    if not _capture_started:
        _capture_started = True
        t = threading.Thread(target=_camera_capture_loop, daemon=True)
        t.start()


def _generate_frames():
    """MJPEG frame generator consumed by Flask's Response streamer."""
    _ensure_capture_started()

    while True:
        with _frame_lock:
            frame_bytes = _latest_frame

        if frame_bytes is None:
            time.sleep(0.05)
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        # Cap output rate to ~30 FPS to avoid saturating the WiFi link
        time.sleep(0.033)


@app.route('/video_feed')
def video_feed():
    return Response(_generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return ("<html><body><h1>Hexapod FPV Stream</h1>"
            "<img src='/video_feed'></body></html>")


def run_flask_app():
    """Run Flask in production-ish mode (no reloader, no debug)."""
    app.run(host=config.FLASK_HOST, port=config.FLASK_PORT,
            debug=False, use_reloader=False)


def start_stream_thread():
    """Spawn the Flask FPV server on a daemon thread."""
    print(f"[FPV] Starting Flask MJPEG stream on port {config.FLASK_PORT}...")
    thread = threading.Thread(target=run_flask_app, daemon=True)
    thread.start()
