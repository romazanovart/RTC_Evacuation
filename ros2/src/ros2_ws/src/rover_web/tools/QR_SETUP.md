# QR decoder

Install on the robot using its system Python (the ROS node uses /usr/bin/python3):

```bash
python3 -m pip install --only-binary=:all: --no-deps --target "$HOME/RTC_Evacuation/vision-lite/qr-deps" zxing-cpp==3.1.1
```

The node scans the original grayscale image with ZXing-C++, then cycles through overlapping enlarged crops, including contrast-enhanced variants. It only publishes successfully decoded QR text and maps outlines back into the original image coordinates. Extra passes have a soft 240 ms budget checked between decoder calls. Missing ZXing falls back to OpenCV.

Camera defaults in web.launch.py: native 1280x720 MJPEG capture, 5 capture FPS; QR processing requested at 2.5 FPS. Actual throughput depends on scene and hardware. Live native capture resolution was verified on the robot. QR decoding was verified with the supplied near screenshot and reduced crops from it (140–260 pixels). The supplied far screenshot remained unreadable; these tests do not establish a physical maximum reading distance.
