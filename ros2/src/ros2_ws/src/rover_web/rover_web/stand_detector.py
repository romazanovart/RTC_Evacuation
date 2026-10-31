"""Field-specific white QR placard detector. Never supplies decoded text."""
import cv2
import numpy as np
from .qr_geometry import locate_qr


def find_stand(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    edges = cv2.Canny(gray, 35, 95)
    contours = []
    for threshold in (140, 155, 170, 185, 200, 215, 230):
        mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)[1]
        found, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours.extend(found)
    choices = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if not (28 <= w <= width * .4 and 28 <= h <= height * .6 and .78 < w/h < 1.25):
            continue
        if y < height*.2 or y+h*1.65 >= height:
            continue
        margin = max(3, int(w*.09))
        inner = gray[y+margin:y+h-margin, x+margin:x+w-margin]
        texture = float(np.mean(edges[y+margin:y+h-margin, x+margin:x+w-margin] > 0))
        if inner.std() < 22 or texture < .18:
            continue
        # Both white legs and the crossbar must be brighter than the space between.
        top, bottom = y+h+int(h*.12), y+h+int(h*.55)
        region = gray[top:bottom, max(0,x-int(w*.1)):min(width,x+int(w*1.1))]
        if region.size == 0:
            continue
        profile = np.mean(region, axis=0)
        n = len(profile)
        left = float(np.max(profile[int(n*.08):int(n*.35)]))
        right = float(np.max(profile[int(n*.65):int(n*.92)]))
        center = float(np.median(profile[int(n*.38):int(n*.62)]))
        contrast = min(left, right)-center
        if contrast < 18:
            continue
        # Contours can bound the printed code rather than the outer plate.
        # The foot then lies about two code heights below its top, not 1.5.
        base = gray[y+int(h*1.50):min(height,y+int(h*2.25)), max(0,x-int(w*.2)):min(width,x+int(w*1.2))]
        if not base.size:
            continue
        bright_rows = np.mean(base, axis=1) >= center+18
        # Require a band, not one bright/noisy scanline.
        band = max(2, int(h*.025))
        if len(bright_rows) < band or np.max(np.convolve(bright_rows.astype(int), np.ones(band, dtype=int), mode='valid')) < band:
            continue
        corners = np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],np.float32)
        choices.append((contrast+texture*100, corners))
    if not choices:
        return None
    choices.sort(key=lambda c:c[0], reverse=True)
    # Ambiguous separated targets require operator selection instead of a jump.
    if len(choices)>1 and np.linalg.norm(choices[0][1].mean(0)-choices[1][1].mean(0)) > width*.15:
        return None
    return choices[0][1]


class StandTracker:
    def __init__(self):
        self.previous = None
        self.count = 0
        self.source = 'white_stand'

    def detect(self, frame):
        scale = min(1.0, 800.0 / max(1, frame.shape[1]))
        working = frame if scale == 1.0 else cv2.resize(
            frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
        )
        geometry_status = {}
        points = locate_qr(working, status=geometry_status)
        if geometry_status.get('ambiguous'):
            self.previous = None
            self.count = 0
            return None
        self.source = 'qr_geometry' if points is not None else 'white_stand'
        if points is None:
            points = find_stand(working)
        if points is None:
            self.previous = None
            self.count = 0
            return None
        if scale != 1.0:
            points = points / scale
        same = self.previous is not None and np.linalg.norm(points.mean(0)-self.previous.mean(0)) < max(20., np.ptp(points[:,0])*.3)
        self.count = self.count+1 if same else 1
        self.previous = points
        return points if self.count >= 3 else None
