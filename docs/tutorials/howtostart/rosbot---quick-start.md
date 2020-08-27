---
title: ROSbot - quick start
sidebar_label: 2. ROSbot - quick start
id: rosbot---quick-start
---

ROSbot 2.0 and ROSbot 2.0 PRO are autonomous, open source robot platforms running on CORE2-ROS controller. They can be used as learning platforms for Robot Operating System (ROS) as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can get it <a href="https://store.husarion.com/">here</a>.

## Unboxing

What's in the box:

- carrying case
- ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
- Wi-Fi 2.4GHz antenna
- 3x 18650 Li-Ion rechargeable batteries
- universal charger with power adapter
- charging cable
- microSD card with the software for ROSbot
- USB to Ethernet adapter

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_unboxing.jpg"/></center></div>

## Assembly

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna.

To mount batteries:

- turn ROSbot upside down
- unscrew battery cover mounted with two screws
- remove the battery cover
- place batteries accordingly to the symbols, keeping the black strip under the batteries
- place battery cover and mount it with screws

To charge the batteries, follow this <a href="https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf">guide</a>.

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

## Connect ROSbot to your Wi-Fi network

At first ROSbot need to be connected to your Wi-Fi network.

### Option 1: Using display, mouse and keyboard

ROSbot is basically a computer running Ubuntu, so let's open it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Connect to a Wi-Fi network using Ubuntu GUI.
4. Open Linux terminal and type `ifconfig` to find your IP address. Save it for later.

### Option 2: Using Ethernet adapter

In the ROSbot 2.0 set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel.
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter.
4. To connect with ROSbot via ssh, type in your terminal application: `ssh husarion@192.168.0.1` and password `husarion`.
5. Connect to a Wi-Fi network.

- in the terminal type `nmcli c add type wifi save yes autoconnect yes con-name rosbot20wifi ifname wlan0 ssid <WiFi-SSID>` and press Enter
- type `nmcli c modify rosbot20wifi wifi-sec.key-mgmt wpa-psk wifi-sec.psk <WiFi-PASSWORD>` and press Enter to obtain an IP address and connect to the Wi-Fi network

6. type `ifconfig` to find your IP address. Save it for later.

## Access ROSbot terminal using wireless connection

### Connecting over LAN network

The most convenient way to work with ROSbot on daily basis is to do that over Wi-Fi. Connect your laptop to the same Wi-Fi network as ROSbot and type in the terminal:

`ssh husarion@<ROSBOT_IP>` where <ROSBOT_IP> is the IP address obtained in the previous steps.

**FOR WINDOWS USERS:**

If you are Windows user you might like to connect to your ROSbot by using the Remote Desktop Connection:

Press `WinKey` + `r` then type `mstsc`.

You will see a window appear:

![Windows RDP](/docs/assets/img/aws-tutorials/quick-start/win_rdp.png)

Type in your device IP address and click connect.

You will see the ROSbot desktop, from the top menu, choose the `Applications` -> `Terminal`.

### Connecting over the internet (optional)

Not always your laptop and ROSbot can be in the same LAN network. To overcome that obstacle use VPN. [Husarnet](https://husarnet.com/) is a recommended VPN for ROSbots. It's preinstalled on ROSbot so you need to install that on your laptop.

To connect your laptop and ROSbot over VPN:

- **In the Linux terminal on your laptop** (Ubuntu OS is recommended) to install Husarnet type: `curl https://install.husarnet.com/install.sh | sudo bash` to install Husarnet. If the process is done type `sudo systemctl restart husarnet`
- **In the Linux terminal of your laptop and in the Linux terminal of your ROSbot** to configure network type:
  `sudo husarnet websetup` and open the link that will appear in a web browser. The link should look like that: `https://app.husarnet.com/husarnet/fc94xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx` and add your devices to the same network. You will need to do that step both for ROSbot and for your laptop.

At this point your laptop and ROSbot should be in the same VPN network. To access your ROSbot from a level of your laptop over the internet just type in the terminal:

`ssh husarion@<HOSTNAME_OF_YOUR_ROSBOT>`

To get more information about using Husarnet visit this [tutorial](https://docs.husarnet.com/getting-started/)

## Low level firmware installation

In the heart of each ROSbot there is a CORE2 board equipped with STM32F4 family microcontroller. The board is responsible for real time tasks like controlling motors, calculating PID regulator output or talking to distance sensors. High level computation is handled by SBC (single board computer) - ASUS Tinker Board (in ROSbot 2.0) or UP Board (in ROSbot 2.0 PRO).

### Mbed firmware

This firmware version is based on ARM's Mbed OS system. If you're interested in learning more about using Mbed OS check our tutorial [Using CORE2 with Mbed OS](https://husarion.com/tutorials/mbed-tutorials/using-core2-with-mbed-os/). We recommend you also to look at the [ROSbot's Mbed firmware GitHub page](https://github.com/husarion/rosbot-firmware-new).

SSH to ROSbot over LAN network or VPN to get access to it's Linux terminal.

#### stm32loader installation

We will use `stm32loader` tool to upload the firmware to ROSbot. To check if you have this tool already installed on your robot, open the terminal and run:

```bash
sudo stm32loader --help
```

If you get `command not found` you will need to refresh your image following this [manual](https://husarion.com/manuals/rosbot-manual/#system-reinstallation).

You can check if tool works by running following commands:

**ROSbot 2.0:**

```bash
sudo stm32loader -c tinker -f F4
```

**ROSbot 2.0 PRO:**

```bash
sudo stm32loader -c upboard -f F4
```

#### Programming the firmware (using stm32loader)

We prepared for you `.bin` files ready to be uploaded to your ROSbot. They have following settings:

- ws2812b driver is enabled by default (check [ROSbot with WS2812B LEDs signalization](https://husarion.com/tutorials/mbed-tutorials/rosbot-and-ws2812b-led-signalization/))
- rosserial baudrate is set to:

  - `500000` for ROSbot 2.0
  - `460800` for ROSbot 2.0 Pro

The appropriate firmware for your ROSbot should be in `/home/husarion/`, if it's not there, you can download it from links below:

- [`ROSbot 2.0 fw v0.13.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.13.1.bin)
- [`ROSbot 2.0 Pro fw v0.13.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.13.1.bin)

Note that default ROSbot firmware is already flashed on your robot.

In case you need to upload the firmware one more time run:

```bash
./flash_firmware.sh
```

Wait until firmware is uploaded.

#### Required ROS packages - `rosbot_ekf`

In order to use Mbed firmware the `rosbot_ekf` package have to be installed on your ROSbot. The package incorporates a ready to use Extended Kalman Filter that combines both the IMU and encoders measurements to better approximate the ROSbot position and orientation. The package also contains custom messages that are required by the new firmware. The package is already installed on your ROSbot.

## Usage

Programming procedure needs to be done only once, on further uses, you can start from this point:

- Turn on your ROSbot.
- Open a terminal and run ssh using that command: `ssh husarion@x.x.x.x` (instead of "x.x.x.x" type hostname of device if you using Husarnet or your local IP address - your laptop should be in the same Wi-Fi network as you robot in this case)
- In terminal paste following command:

_If you use ROSbot 2.0:_

```bash
roslaunch route_admin_panel demo_rosbot_mbed_fw.launch
```

_If you use ROSbot 2.0 PRO:_

```bash
roslaunch route_admin_panel demo_rosbot_pro_mbed_fw.launch
```

Launch web browser and type IPv6 if you using husarnet:
```
[fc92:a348:c2e8:cbcb:480f:bd93:6a21:f3c7]:8000
```
or the local IP of your ROSbot if both device are in the same network:
```
192.0.2.26:8000
```
*Please note that IP address may vary depending on your network configuration!*

- You should see interface as below, use it to test and control your ROSbot.

Also, you can check how it works in gazebo simulation:
    
```bash
roslaunch route_admin_panel demo_gazebo.launch
```

![panel accessed through husarnet](/docs/assets/img/software/panel_at_husarnet.png)

You can find detailed description of Route Admin Panel in [software section](https://husarion.com/software/route-admin-panel/).

> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](https://husarion.com/manuals/rosbot-manual/#rear-panel-description) if battery level is low). Charging manual is [here](https://husarion.com/manuals/rosbot-manual/#charging-rosbot).
