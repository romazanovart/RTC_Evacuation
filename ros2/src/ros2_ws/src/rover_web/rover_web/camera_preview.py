"""Small transport preview, independent of the full-resolution QR input."""
import cv2


def encode_preview(frame):
    height, width = frame.shape[:2]
    if width > 640:
        frame = cv2.resize(frame, (640, max(1, round(height * 640 / width))),
                           interpolation=cv2.INTER_AREA)
    ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
    if not ok:
        raise ValueError('Could not encode camera preview')
    return data.tobytes()
