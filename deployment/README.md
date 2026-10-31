# Облегчённый запуск

Сервисы не включают Rosboard, ttyd, лидар и YOLO на Raspberry Pi.

```bash
chmod +x deployment/start_robot.sh
mkdir -p ~/.config/systemd/user
cp deployment/systemd/*.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now rtc-evacuation-web.service
systemctl --user enable --now rtc-evacuation-bringup.service
```

Для запуска после загрузки без активного входа пользователя:

```bash
loginctl enable-linger "$USER"
```

Сначала поднимается доступный оператору веб-интерфейс. Контур моторов, IMU и
одометрии запускается отдельно, а камера обслуживается веб-стеком. При падении
одного сервиса systemd перезапускает только его, не перезагружая Raspberry Pi.
