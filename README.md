# SensorHub

Русская версия: [README_RU.md](README_RU.md)

SensorHub is a compact device that monitors your home environment — temperature, humidity, air quality and more — and displays the readings on a single web page. No additional software or account is required.

## Main features

- Reads the connected sensors and displays the readings in real time.
- Switches lights and outlets; accepts commands from buttons.
- Provides animated effects for LED lighting (fire, rainbow and others).
- Sends the readings to a smart home system (e.g., **Home Assistant**).
- Retains all settings even after a power outage.

## Initial setup — 3 steps

1. Connect the device to a power outlet and join the Wi-Fi network **SensorHub-AP** (password: **sensorhub123**).
2. Open **http://192.168.4.1** in your browser.
3. On the **Connection** page, select your home Wi-Fi network and enter its password. The device will save the settings automatically.

From then on, the device is accessed at a fixed address; the page will display it before the switchover, so please write it down.

## Web interface — 4 tabs

- **Sensors** — live readings; sensors can be added, renamed or removed.
- **Scripts** — simple automation for LED effects and custom readings.
- **Connection** — Wi-Fi and smart home settings.
- **System** — one-click updates, settings backup and device status.

## Notes

- If the Wi-Fi password is entered incorrectly, the device will start its own access point **SensorHub-AP** again, and the setup can be repeated.
- **Backup** (on the Sensors tab) saves all settings into a single file. A copy should be kept before making significant changes.
- Current firmware files are published on the [Releases](releases) page of this project.

## Hardware

The device is built on an ESP32 board. The complete source code is available in this repository.
