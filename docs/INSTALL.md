# Установка и запуск

## 1. ROS 2 workspace

```bash
cd ros2/src/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Укажите реальные последовательные порты при запуске:

```bash
ros2 launch frob_bringup bringup.launch.py \
  arduino_port:=/dev/ttyACM0 \
  start_lidar:=false
```

Лидар оставлен опциональным и в сценарии движения к QR не используется.

## 2. Камера и веб-интерфейс

```bash
python3 -m pip install --user zxing-cpp==3.1.1 aiohttp PyYAML
source ros2/src/ros2_ws/install/setup.bash
ros2 launch rover_web web.launch.py
```

Проверка:

```bash
curl http://127.0.0.1:8765/api/health
```

## 3. Редактор карты на ноутбуке

Задайте MAC робота в локальном окружении, не в исходном коде:

```bash
export FROB_ROBOT_MAC=00:11:22:33:44:55
python3 tools/field_console.py
```

Редактор доступен на `http://127.0.0.1:1111/`. Если нужен пароль:

```bash
export FROB_FIELD_PASSWORD='change-me'
```

## 4. YOLO worker

```bash
python3 -m pip install --user ultralytics
python3 tools/object_detector.py --model models/rtk-best.pt
```

Для автономного теста без робота:

```bash
python3 tools/object_detector.py \
  --model models/rtk-best.pt \
  --image frame.jpg \
  --output result.jpg
```

## 5. Прошивка

Откройте `arduino/arduino_code/arduino_code.ino` в Arduino IDE, проверьте
пины и параметры кинематики в `robot_params.h`, затем загрузите прошивку.

## 6. Лёгкий автозапуск

Примеры user-сервисов находятся в `deployment/systemd/`. Они запускают веб
раньше остальных узлов и не включают Rosboard, ttyd и модель YOLO на Raspberry
Pi. Перед установкой замените пути или используйте `%h/RTC_Evacuation`.
