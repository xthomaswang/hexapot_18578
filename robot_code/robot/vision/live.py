#!/usr/bin/env python3
import argparse
import atexit
import io
import os
import time
from threading import Lock
from types import SimpleNamespace

import numpy as np
from flask import Flask, Response, jsonify
from PIL import Image, ImageDraw

from camera import PiCameraStream
from detector import build_detector


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Robot vision live server",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python live.py camera\n"
            "  python live.py detect\n"
            "  python live.py detect --port 8090\n"
            "  python live.py camera --host 0.0.0.0 --port 8080"
        ),
    )
    parser.add_argument(
        "mode",
        nargs="?",
        choices=("camera", "detect"),
        default=os.environ.get("VISION_MODE", "camera").lower(),
        help="camera: raw camera stream, detect: AprilTag overlay stream",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("VISION_HOST", "0.0.0.0"),
        help="host interface to bind",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("VISION_PORT", "8080")),
        help="HTTP port",
    )
    parser.add_argument(
        "--detector",
        default=None,
        help="detector backend name, defaults to apriltag in detect mode",
    )
    args = parser.parse_args()
    if args.detector is None:
        args.detector = "apriltag" if args.mode == "detect" else "none"
    args.detector = args.detector.lower()
    return args


def _default_args():
    mode = os.environ.get("VISION_MODE", "camera").lower()
    detector = os.environ.get(
        "VISION_DETECTOR",
        "apriltag" if mode == "detect" else "none",
    ).lower()
    return SimpleNamespace(
        mode=mode,
        host=os.environ.get("VISION_HOST", "0.0.0.0"),
        port=int(os.environ.get("VISION_PORT", "8080")),
        detector=detector,
    )


RUNTIME_ARGS = _parse_args() if __name__ == "__main__" else _default_args()

HOST = RUNTIME_ARGS.host
PORT = RUNTIME_ARGS.port
MODE = RUNTIME_ARGS.mode
DETECTOR_NAME = RUNTIME_ARGS.detector

app = Flask(__name__)
camera = PiCameraStream()
latest_detection_lock = Lock()
latest_detection_payload = None

_detector_error = None
try:
    detector = build_detector(DETECTOR_NAME)
except Exception as exc:
    detector = build_detector("none")
    _detector_error = str(exc)


@atexit.register
def stop_camera():
    try:
        camera.stop()
    except Exception:
        pass


def _serialize_detection(item):
    return {
        "kind": item.kind,
        "family": item.family,
        "tag_id": item.tag_id,
        "hamming": item.hamming,
        "center": item.center,
        "confidence": item.confidence,
        "corners": item.corners,
    }


def _detector_enabled():
    return MODE == "detect" and detector.name != "none" and _detector_error is None


def _build_detection_payload(items=None, error=None):
    return {
        "mode": MODE,
        "detector": detector.info(),
        "error": error,
        "count": len(items or []),
        "detections": items or [],
        "updated_at": time.time(),
    }


def _set_latest_detection_payload(payload):
    global latest_detection_payload
    with latest_detection_lock:
        latest_detection_payload = payload


def _get_latest_detection_payload():
    with latest_detection_lock:
        return latest_detection_payload


def _run_detection(frame):
    items = [_serialize_detection(item) for item in detector.detect(frame)]
    payload = _build_detection_payload(items=items)
    _set_latest_detection_payload(payload)
    return payload


def _frame_to_image(frame):
    array = np.asarray(frame)
    if array.ndim == 2:
        return Image.fromarray(array.astype(np.uint8), mode="L").convert("RGB")
    if array.ndim == 3 and array.shape[2] >= 3:
        rgb = array[..., :3][:, :, ::-1].astype(np.uint8)
        return Image.fromarray(rgb, mode="RGB")
    raise ValueError("Unsupported frame format for live overlay")


def _render_detection_jpeg(frame, payload):
    image = _frame_to_image(frame)
    draw = ImageDraw.Draw(image)
    for index, item in enumerate(payload["detections"], start=1):
        corners = item.get("corners") or []
        if corners:
            points = [tuple(point) for point in corners]
            if len(points) >= 2:
                draw.line(points + [points[0]], fill=(57, 255, 20), width=4)

        center = item.get("center")
        if center:
            cx, cy = center
            draw.ellipse((cx - 6, cy - 6, cx + 6, cy + 6), fill=(255, 64, 64))

        label_anchor = corners[0] if corners else center or (20, 30 * index)
        label = (
            f"tag {item.get('tag_id', '?')} | "
            f"margin {float(item.get('confidence') or 0.0):.1f}"
        )
        tx, ty = int(label_anchor[0]), max(int(label_anchor[1]) - 22, 10)
        draw.rectangle((tx - 4, ty - 4, tx + 220, ty + 18), fill=(0, 0, 0))
        draw.text((tx, ty), label, fill=(255, 230, 80))

    draw.rectangle((12, 12, 260, 34), fill=(0, 0, 0))
    draw.text(
        (18, 16),
        f"mode={MODE} detector={detector.info().get('name')}",
        fill=(255, 255, 255),
    )

    encoded = io.BytesIO()
    image.save(encoded, format="JPEG", quality=85)
    return encoded.getvalue()


def _iter_detect_mjpeg():
    while True:
        frame = camera.capture_array()
        try:
            payload = _run_detection(frame)
            jpeg = _render_detection_jpeg(frame, payload)
        except Exception as exc:
            payload = _build_detection_payload(error=str(exc))
            _set_latest_detection_payload(payload)
            jpeg = _render_detection_jpeg(frame, payload)

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        )


def _latest_or_fresh_detections():
    latest = _get_latest_detection_payload()
    if latest is not None:
        return latest

    frame = camera.capture_array()
    return _run_detection(frame)


@app.route("/")
def index():
    detector_text = detector.name if _detector_error is None else f"unavailable ({_detector_error})"
    return (
        "<!doctype html>"
        "<html><head><title>Robot Vision</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{margin:0;font-family:system-ui;background:#111;color:#f5f5f5;min-height:100vh}"
        ".wrap{width:min(96vw,1280px);margin:0 auto;padding:24px}"
        ".grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(280px,1fr);gap:20px;align-items:start}"
        ".panel{background:#181818;border:1px solid #303030;border-radius:16px;padding:16px}"
        ".stream{width:100%;height:auto;border:1px solid #333;border-radius:12px;background:#000}"
        ".muted{opacity:.8}"
        ".pill{display:inline-block;background:#1d1d1d;padding:.15rem .45rem;border-radius:999px;margin-right:.35rem}"
        "pre{white-space:pre-wrap;word-break:break-word;background:#0f0f0f;border:1px solid #262626;padding:12px;border-radius:12px;min-height:220px;overflow:auto}"
        "@media (max-width:900px){.grid{grid-template-columns:1fr}}"
        "</style></head>"
        "<body><div class='wrap'>"
        "<h1>Robot Vision Live</h1>"
        f"<p class='muted'><span class='pill'>mode={MODE}</span><span class='pill'>detector={detector_text}</span></p>"
        "<div class='grid'>"
        "<div class='panel'>"
        "<img class='stream' src='/stream.mjpg' alt='Live camera stream' />"
        "<p class='muted'>"
        "Use <code>python live.py camera</code> for raw camera hosting, "
        "or <code>python live.py detect</code> for AprilTag overlay hosting."
        "</p>"
        "<p class='muted'>Endpoints: <code>/stream.mjpg</code> <code>/raw.mjpg</code> <code>/detections</code> <code>/config</code> <code>/health</code></p>"
        "</div>"
        "<div class='panel'>"
        "<h2>Detection Output</h2>"
        "<pre id='detections'>Loading...</pre>"
        "</div>"
        "</div>"
        "<script>"
        "async function refreshDetections(){"
        "  try {"
        "    const res = await fetch('/detections', {cache:'no-store'});"
        "    const data = await res.json();"
        "    document.getElementById('detections').textContent = JSON.stringify(data, null, 2);"
        "  } catch (err) {"
        "    document.getElementById('detections').textContent = 'Failed to fetch detections: ' + err;"
        "  }"
        "}"
        "refreshDetections();"
        "setInterval(refreshDetections, 700);"
        "</script>"
        "</div></body></html>"
    )


@app.route("/stream.mjpg")
def stream():
    stream_iter = _iter_detect_mjpeg() if _detector_enabled() else camera.iter_mjpeg()
    return Response(stream_iter, mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/raw.mjpg")
def raw_stream():
    return Response(
        camera.iter_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/detections")
def detections():
    if not _detector_enabled():
        return jsonify(_build_detection_payload(error=_detector_error or "detector disabled"))

    try:
        payload = _latest_or_fresh_detections()
    except Exception as exc:
        payload = _build_detection_payload(error=str(exc))
        _set_latest_detection_payload(payload)
        return jsonify(payload), 500

    return jsonify(payload)


@app.route("/config")
def config():
    return jsonify(
        {
            "mode": MODE,
            "port": PORT,
            "detector": detector.info(),
            "detector_error": _detector_error,
        }
    )


@app.route("/health")
def health():
    return jsonify(
        {
            "status": "ok",
            "mode": MODE,
            "detector": detector.info(),
            "detector_error": _detector_error,
            "port": PORT,
        }
    )


if __name__ == "__main__":
    app.run(host=HOST, port=PORT, threaded=True)
