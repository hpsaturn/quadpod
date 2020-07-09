# Quadpod ESP32 Bluetooth Alternative

Quadruped robot, spider form factor, improved with a ESP32 and a Bluetooth connection for improve usability and power consumption. Also the original timer library was migrated to a RTOS  task on the dual core of the ESP32 for improve and reducing the electronic, because the original project was using two microcontrollers, now you only need one. 

## Build and Flashing

First of all, install PlatformIO with your favourite IDE (i.e. VSCode). Follow [this](https://platformio.org/platformio-ide) instructions. Also, you may need to install [git](http://git-scm.com/) in your system.

Clone the repo

```sh
git clone https://github.com/hpsaturn/quadpod.git
```

Connect the ESP32 via USB. In Windows 10, drivers are installed automatically. I guess with other OS will be automatically installed too.

Open cloned folder with your PlatformIO IDE and build & upload it. For details please see the [documentation](https://docs.platformio.org/en/latest/integration/ide/vscode.html#quick-start), but the process flow is more easy than Arduino IDE flow, but you can also import it to the Arduino IDE if you want.

![PlatformIO Build tool](https://docs.platformio.org/en/latest/_images/platformio-ide-vscode-toolbar.png)

Or after clone also you can build and upload the current firmware from CLI:

```shell
cd quadpod && pio pio run --target upload
```

# Bluetooth control

<img align="right" width="120" src="images/bt_serial_control.jpg">

For now, you can use this app: [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal) and donwload the file `serial_bluetooth_terminal.cfg.txt` that is under `bt` folder of this project, then send it to your phone and import it with this application for get all commands and buttons for controlling the robot.
