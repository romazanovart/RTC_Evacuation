#!/usr/bin/env python3
"""Laptop-only YOLO worker: one fresh frame in flight, no motor API access."""
import argparse
import base64
import json
import logging
import os
from pathlib import Path
import socket
import sys
import time
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
os.environ.setdefault('YOLO_OFFLINE', 'true')
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('YOLO_CONFIG_DIR', str(Path.home() / '.local/share/frob/yolo-settings'))
ROOT = Path(__file__).resolve().parents[1]
ROVER_PACKAGE = ROOT / 'ros2/src/ros2_ws/src/rover_web'
sys.path.insert(0, str(ROVER_PACKAGE))
from rover_web.lan_peer import LANPeer
from rover_web.neural_overlay import LABELS

EXPECTED_NAMES = {0: 'rtk-benzovoz', 1: 'rtk-derevo', 2: 'rtk-dtp', 3: 'rtk-qr'}
COLORS = {0: (255, 154, 50), 1: (60, 190, 255), 2: (255, 90, 130),
          3: (80, 220, 100)}


def load_detector(path):
    """Load only the reviewed YOLOv8 classes, never arbitrary pickle globals."""
    import torch
    from torch import nn
    Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True, exist_ok=True)
    from ultralytics import YOLO, settings
    from ultralytics.nn.tasks import DetectionModel
    from ultralytics.nn.modules import Bottleneck, C2f, DFL, SPPF, Concat, Conv, Detect
    allowed = [set, nn.ModuleList, nn.Sequential, nn.Conv2d, nn.BatchNorm2d,
               nn.SiLU, nn.MaxPool2d, nn.Upsample, DetectionModel,
               Bottleneck, C2f, DFL, SPPF, Concat, Conv, Detect]
    settings.update({'sync': False})
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    with torch.serialization.safe_globals(allowed):
        checkpoint = torch.load(path, map_location='cpu', weights_only=True)
    model = checkpoint.get('model')
    if not isinstance(model, DetectionModel) or model.names != EXPECTED_NAMES:
        raise ValueError('Expected the supplied RTK YOLOv8 model and its four classes')
    detector = YOLO('yolov8n.yaml', task='detect', verbose=False)
    detector.model = model.float().eval()
    detector.overrides = {'task': 'detect', 'mode': 'predict', 'imgsz': 640}
    return detector


class Detector:
    def __init__(self, model, confidence=.45):
        import cv2
        from PIL import ImageFont
        cv2.setNumThreads(1)
        self.model = load_detector(model)
        self.confidence = confidence
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
        # Warm up outside the live-frame deadline.
        import numpy as np
        self.model.predict(np.zeros((360, 640, 3), dtype=np.uint8),
                           imgsz=640, conf=confidence, device='cpu', verbose=False, max_det=30)

    def infer(self, jpeg):
        import cv2
        import numpy as np
        from PIL import Image, ImageDraw
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape[1] > 1920 or frame.shape[0] > 1080:
            raise ValueError('Invalid camera image')
        started = time.monotonic()
        result = self.model.predict(frame, imgsz=640, conf=self.confidence,
                                    device='cpu', verbose=False, max_det=30)[0]
        inference_ms = (time.monotonic() - started) * 1000
        height, width = frame.shape[:2]
        rows = []
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image)
        for box in result.boxes:
            cls = int(box.cls.item())
            score = float(box.conf.item())
            if cls not in LABELS:
                continue
            coords = box.xyxy[0].tolist()
            x1, y1, x2, y2 = [max(0, min(limit, float(v))) for v, limit in
                              zip(coords, (width - 1, height - 1, width - 1, height - 1))]
            if x1 >= x2 or y1 >= y2:
                continue
            rows.append({'class_id': cls, 'confidence': score,
                         'box': [x1 / width, y1 / height, x2 / width, y2 / height]})
            color = COLORS[cls]
            draw.rectangle((x1, y1, x2, y2), outline=color, width=2)
            label = f'{LABELS[cls]} {score:.0%}'
            bounds = draw.textbbox((0, 0), label, font=self.font)
            tw, th = bounds[2] - bounds[0] + 8, bounds[3] - bounds[1] + 8
            tx, ty = min(x1, max(0, width - tw)), max(0, y1 - th)
            draw.rectangle((tx, ty, tx + tw, ty + th), fill=color)
            draw.text((tx + 4, ty + 4 - bounds[1]), label, font=self.font, fill=(10, 20, 30))
        annotated = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError('Cannot encode neural preview')
        return {'image_b64': base64.b64encode(encoded).decode('ascii'),
                'inference_ms': inference_ms, 'detections': rows}


def notify(message):
    address = os.environ.get('NOTIFY_SOCKET')
    if not address:
        return
    if address.startswith('@'):
        address = '\0' + address[1:]
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
        sock.sendto(message.encode(), address)


def run(detector, fps):
    robot = LANPeer(os.environ.get('FROB_ROBOT_MAC', '00:00:00:00:00:00'), 8765, 'rover_web')
    http = build_opener(ProxyHandler({}))
    last_frame = None
    last_log = 0.
    completed = 0
    notify('READY=1\nSTATUS=Модель готова; поиск робота')
    while True:
        started = time.monotonic()
        notify('WATCHDOG=1')
        delay = 1 / fps
        try:
            endpoint = robot.resolve()
            with http.open(endpoint + '/api/neural/input', timeout=1.2) as response:
                data = json.loads(response.read(800_000))
            frame_id = (endpoint, data['frame_id'])
            if frame_id != last_frame:
                last_frame = frame_id
                result = detector.infer(base64.b64decode(data['image_b64'], validate=True))
                result['token'] = data['token']
                request = Request(endpoint + '/api/neural/result',
                                  data=json.dumps(result).encode(),
                                  headers={'Content-Type': 'application/json'})
                with http.open(request, timeout=1.2) as response:
                    accepted = json.loads(response.read(4096)).get('accepted')
                completed += int(bool(accepted))
                if time.monotonic() - last_log >= 30:
                    logging.info('robot=%s accepted=%s inference=%.1fms objects=%s frames=%s',
                                 endpoint, accepted, result['inference_ms'],
                                 [LABELS[row['class_id']] for row in result['detections']], completed)
                    last_log = time.monotonic()
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            if not isinstance(exc, HTTPError):
                robot.invalidate()
            delay = 3.
            if time.monotonic() - last_log >= 30:
                logging.warning('Ожидание камеры/робота: %s', exc)
                last_log = time.monotonic()
        notify('WATCHDOG=1')
        time.sleep(max(0., delay - (time.monotonic() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT / 'models/rtk-best.pt')
    parser.add_argument('--fps', type=float, default=4.)
    parser.add_argument('--confidence', type=float, default=.45)
    parser.add_argument('--image', type=Path, help='Offline single-image test; never contacts robot')
    parser.add_argument('--output', type=Path, help='Annotated JPEG for offline test')
    args = parser.parse_args()
    if not 0 < args.fps <= 5 or not 0 < args.confidence <= 1:
        parser.error('Expected fps in (0,5] and confidence in (0,1]')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    detector = Detector(args.model, args.confidence)
    logging.info('Модель загружена: %s; классы: %s', args.model.name, list(LABELS.values()))
    if args.image:
        result = detector.infer(args.image.read_bytes())
        jpeg = base64.b64decode(result.pop('image_b64'))
        if args.output:
            args.output.write_bytes(jpeg)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return
    run(detector, args.fps)


if __name__ == '__main__':
    main()
