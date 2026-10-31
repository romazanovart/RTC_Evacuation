"""Bounded QR work in a killable process; never block live video on inference."""
import multiprocessing as mp
import os
import queue
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from .qr_geometry import locate_qr


class Decoder:
    def __init__(self):
        dependency_root = Path(
            os.environ.get('RTC_EVACUATION_ROOT', Path.home() / 'RTC_Evacuation')
        )
        sys.path.insert(0, str(dependency_root / 'vision-lite/qr-deps'))
        import zxingcpp
        self.zx = zxingcpp
        self.index = 0
        self.roi = None
        self.roi_time = 0.0
        self.clahe = cv2.createCLAHE(2.0, (8, 8))
        self.locator = cv2.QRCodeDetector()
        self.roi_variant = 0
        self.last_stats = {}

    def decode(self, frame, budget=0.65, roi_hint=None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        started = time.monotonic()
        self.last_stats = {'attempts': 0, 'method': None}

        def scan(image, x=0, y=0, sx=1., sy=1., method='full', binary=False, transform=None):
            if time.monotonic() - started >= budget:
                return []
            self.last_stats['attempts'] += 1
            found = []
            for code in self.zx.read_barcodes(
                np.ascontiguousarray(image), formats=self.zx.BarcodeFormat.QRCode,
                try_rotate=True, try_downscale=True, try_invert=True,
                binarizer=self.zx.Binarizer.BoolCast if binary else self.zx.Binarizer.LocalAverage,
            ):
                if code.valid and code.text:
                    p = code.position
                    points = np.array([[v.x / sx + x, v.y / sy + y] for v in
                                       (p.top_left, p.top_right, p.bottom_right, p.bottom_left)], np.float32)
                    if transform is not None:
                        points = cv2.perspectiveTransform(points.reshape(1, 4, 2), transform)[0]
                    found.append((code.text, points))
            if found:
                self.roi = found[0][1]
                self.roi_time = time.monotonic()
                self.last_stats['method'] = method
            return found

        def crop_bounds(points):
            if points is None:
                return None
            try:
                points = np.asarray(points, np.float32).reshape(4, 2)
            except (ValueError, TypeError):
                return None
            if not np.isfinite(points).all():
                return None
            lo, hi = points.min(axis=0), points.max(axis=0)
            if min(hi-lo) < 8:
                return None
            pad = max(12, int(max(hi-lo)*.18))
            x, y = np.maximum(0, np.floor(lo).astype(int)-pad)
            r, b = np.minimum([width,height], np.ceil(hi).astype(int)+pad)
            return (int(x),int(y),int(r),int(b)) if r>x and b>y else None

        # Use coordinates from this raw frame, never the yellow/green preview.
        hint = crop_bounds(roi_hint)
        if hint is None and time.monotonic()-started < budget*.15:
            hint = crop_bounds(locate_qr(frame))
        if hint is not None:
            x,y,r,b = hint
            tile = gray[y:b,x:r]
            roi_deadline = started + budget * .55
            phase = self.roi_variant
            self.roi_variant = (phase+1) % 4
            for variant in (0, 1, 2+phase):
                if time.monotonic() >= roi_deadline:
                    break
                scale = min(4., max(1., 640./max(tile.shape)))
                interpolation = cv2.INTER_NEAREST if variant == 2 else cv2.INTER_CUBIC
                processed = tile if variant == 0 else cv2.resize(tile,None,fx=scale,fy=scale,interpolation=interpolation)
                if variant == 3:
                    processed = self.clahe.apply(processed)
                elif variant == 4:
                    processed = cv2.addWeighted(processed,1.6,cv2.GaussianBlur(processed,(0,0),1),-.6,0)
                elif variant == 5:
                    processed = cv2.adaptiveThreshold(processed,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,31,3)
                result = scan(processed,x,y,processed.shape[1]/(r-x),processed.shape[0]/(b-y),
                              method=f'candidate-{variant}',binary=variant==5)
                if result:
                    return result

        # Native pixels first: interpolation never invents missing QR modules.
        result = scan(gray)
        if result:
            return result
        # Locate an actual quadrilateral before rectification. A stand's bounding
        # rectangle is not a perspective estimate. Native decoder stays primary.
        if hint is not None and time.monotonic()-started < budget*.7:
            x,y,r,b = hint
            ok, polygon = self.locator.detect(gray[y:b,x:r])
            if ok and polygon is not None:
                polygon = polygon.reshape(4,2).astype(np.float32) + [x,y]
                polygon = polygon.astype(np.float32)
                if np.isfinite(polygon).all() and cv2.isContourConvex(polygon) and abs(cv2.contourArea(polygon)) > 64:
                    size, border = 512, 40
                    destination = np.array([[border,border],[size-border,border],
                                            [size-border,size-border],[border,size-border]],np.float32)
                    matrix = cv2.getPerspectiveTransform(polygon,destination)
                    if abs(np.linalg.det(matrix)) > 1e-10:
                        rectified = cv2.warpPerspective(gray,matrix,(size,size),flags=cv2.INTER_CUBIC,
                                                       borderMode=cv2.BORDER_CONSTANT,borderValue=255)
                        result = scan(rectified,method='perspective',transform=np.linalg.inv(matrix))
                        if result:
                            return result
        crops = []
        if self.roi is not None and started - self.roi_time < 2.:
            lo, hi = self.roi.min(axis=0), self.roi.max(axis=0)
            pad = max(24, int(max(hi - lo) * .5))
            x, y = np.maximum(0, lo.astype(int) - pad)
            r, b = np.minimum([width, height], hi.astype(int) + pad)
            crops.append((int(x), int(y), int(r-x), int(b-y), self.index % 3))
        # Rotate through overlapping tiles and preprocessing on successive frames.
        # Advance before processing, so expensive tiles cannot starve the rest.
        for _ in range(3):
            idx = self.index
            self.index = (self.index + 1) % 54
            tw, th = max(16, width // 2), max(16, height // 2)
            x = (0, (width-tw)//2, width-tw)[idx % 3]
            y = (0, (height-th)//2, height-th)[(idx//3) % 3]
            crops.append((x, y, tw, th, (idx//9) % 3))
        for x, y, tw, th, variant in crops:
            if time.monotonic() - started >= budget:
                break
            tile = gray[y:y+th, x:x+tw]
            if not tile.size:
                continue
            if variant == 1:
                tile = self.clahe.apply(tile)
            elif variant == 2:
                tile = cv2.addWeighted(tile, 1.6, cv2.GaussianBlur(tile, (0, 0), .8), -.6, 0)
            scale = min(3., 1600. / max(tile.shape))
            enlarged = cv2.resize(tile, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            result = scan(enlarged, x, y, enlarged.shape[1]/tw, enlarged.shape[0]/th)
            if result:
                return result
        return []


def run_worker(inbox, outbox, pixels):
    cv2.setNumThreads(1)
    decoder = Decoder()
    while True:
        shape, roi_hint = inbox.get()
        frame = np.frombuffer(pixels, dtype=np.uint8, count=int(np.prod(shape))).reshape(shape).copy()
        try:
            outbox.put((decoder.decode(frame, roi_hint=roi_hint), None))
        except Exception as exc:
            outbox.put(([], str(exc)))


class QRWorker:
    def __init__(self, timeout=3.0, target=run_worker):
        self.context = mp.get_context('spawn')
        self.timeout = timeout
        self.target = target
        self.process = None
        self.pending = False
        self.started = 0.
        self.restarts = 0
        self.start()

    def start(self):
        self.pixels = self.context.RawArray('B', 4096 * 4096 * 3)
        self.inbox = self.context.Queue(maxsize=1)
        self.outbox = self.context.Queue(maxsize=1)
        self.process = self.context.Process(target=self.target, args=(self.inbox, self.outbox, self.pixels), daemon=True)
        self.process.start()

    def submit(self, frame, roi_hint=None):
        if self.pending:
            return False
        if frame.size > len(self.pixels):
            raise ValueError('QR input exceeds 4096x4096 pixels')
        np.copyto(np.frombuffer(self.pixels, dtype=np.uint8, count=frame.size), frame.reshape(-1))
        self.inbox.put_nowait((frame.shape, None if roi_hint is None else np.asarray(roi_hint).tolist()))
        self.pending = True
        self.started = time.monotonic()
        return True

    def poll(self):
        try:
            result = self.outbox.get_nowait()
            self.pending = False
            return result
        except queue.Empty:
            pass
        if not self.process.is_alive() or (self.pending and time.monotonic() - self.started > self.timeout):
            self.close()
            self.restarts += 1
            self.start()
            return ([], 'QR worker restarted after timeout or exit')
        return None

    def close(self):
        if self.process is not None:
            if self.process.is_alive():
                self.process.kill()
            self.process.join(timeout=.2)
        for q in (self.inbox, self.outbox):
            q.cancel_join_thread()
            q.close()
        self.pending = False
