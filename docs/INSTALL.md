# Установка и запуск

## 1. ROS 2 workspace

Окружение проверенной сборки: Ubuntu 24.04 и ROS 2 Jazzy. Команды выполняются
из корня репозитория.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/src/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
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
sudo apt install python3-opencv python3-aiohttp python3-yaml
python3 -m pip install --only-binary=:all: --no-deps \
  --target "$HOME/RTC_Evacuation/vision-lite/qr-deps" zxing-cpp==3.1.1
source /opt/ros/jazzy/setup.bash
source ros2/src/ros2_ws/install/setup.bash
ros2 launch rover_web web.launch.py
```

Проверка:

```bash
curl http://127.0.0.1:8765/api/health
```

## 3. Связь с ноутбуком

Задайте MAC робота в локальном окружении, не в исходном коде:

```bash
export FROB_ROBOT_MAC=00:11:22:33:44:55
```

`LANPeer` ищет робота в локальной сети по идентификатору устройства и проверяет
его HTTP health endpoint. Ноутбучный детектор использует эту настройку при
смене DHCP-адреса робота. `tools/field_console.py` содержит вспомогательный
сценарный обработчик текстовых заданий.

## 4. YOLO worker

```bash
python3 -m venv --system-site-packages .venv-objects
.venv-objects/bin/pip install ultralytics
.venv-objects/bin/python tools/object_detector.py --model models/rtk-best.pt
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
