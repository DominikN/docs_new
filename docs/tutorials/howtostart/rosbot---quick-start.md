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
3. Connect to a Wi-Fi network using Ubuntu GUI
4. Open Linux terminal and type `ifconfig` to find your IP address. Save it for later.

### Option 2: Using Ethernet adapter

In the ROSbot 2.0 set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. To connect with ROSbot via ssh, type in your terminal application: `ssh husarion@192.168.0.1` and password `husarion`
5. Connect to a Wi-Fi network

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

Not always your laptop and ROSbot can be in the same LAN network. To overcome that obstacle use VPN. [Husarnet](https://husarnet.com/) is a recommended VPN for ROSbots. It's preinstalled on ROSbot so you need to install that on you laptop.

To connect your laptop and ROSbot over VPN:

- **In the Linux terminal on your laptop** (Ubuntu OS is recommended) to install Husarnet type: `curl https://install.husarnet.com/install.sh | sudo bash` to install Husarnet. If the process is done type `sudo systemctl restart husarnet`
- **In the Linux terminal of your laptop and in the Linux terminal of your ROSbot** to configure network type:
  `sudo husarnet websetup` and open the link that will appear in a web browser. The link should look like that: `https://app.husarnet.com/husarnet/fc94xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx` and add your devices to the same network. You will need to do that step both for ROSbot and for your laptop.

At this point your laptop and ROSbot should be in the same VPN network. To access your ROSbot from a level of your laptop over the internet just type in the terminal:

`ssh husarion@<HOSTNAME_OF_YOUR_ROSBOT>`

To get more information about using Husarnet visit this [tutorial](https://docs.husarnet.com/getting-started/)

## Low level firmware installation

In the heart of each ROSbot there is a CORE2 board equipped with STM32F4 family microcontroller. The board is responsible for real time tasks like controlling motors, calculating PID regulator output or talking to distance sensors. High level computation is handled by SBC (single board computer) - ASUS Tinker Board (in ROSbot 2.0) or UP Board (in ROSbot 2.0 PRO).

In order to use ROSbot you have to flash ROSbot's CORE2 board with low level firmware. There are two firmware options for you to choose from.

### I. Mbed firmware (recommended)

> **WARNING**: When mbed firmware is uploaded to internal STM32F4 microcontroller, https://cloud.husarion.com is no available for your ROSbot.

This firmware version is based on ARM's Mbed OS system. If you're interested in learning more about using Mbed OS check our tutorial [Using CORE2 with Mbed OS](https://husarion.com/tutorials/mbed-tutorials/using-core2-with-mbed-os/). We recommend you also to look at the [ROSbot's Mbed firmware GitHub page](https://github.com/husarion/rosbot-firmware-new).

SSH to ROSbot over LAN network or VPN to get access to it's Linux terminal.

#### stm32loader installation

We will use `stm32loader` tool to upload the firmware to ROSbot. To check if you have this tool already installed on your robot, open the terminal and run:

```bash
sudo stm32loader --help
```

If you get `command not found` you will need to finish all the steps below. Otherwise, you just need to complete step one.

<strong>1.</strong> Disable `husarnet-configurator` and `husarion-shield services` and reboot your ROSbot. These services are responsible for connection to the Husarion Cloud and they also control GPIO pins that are used for uploading the firmware. We will need direct access to them. Run:

```bash
sudo systemctl disable husarnet-configurator
sudo systemctl stop husarnet-configurator
sudo systemctl disable husarion-shield
sudo reboot
```

<strong>2.</strong> Install necessary support libraries on your robot. In the terminal run:

**ROSbot 2.0:**

```bash
cd ~/ && git clone https://github.com/TinkerBoard/gpio_lib_python.git
cd ~/gpio_lib_python && sudo python setup.py install --record files.txt
```

**ROSbot 2.0 PRO:**

```bash
cd ~/ && git clone https://github.com/vsergeev/python-periphery.git
cd ~/python-periphery && git checkout v1.1.2
sudo python setup.py install --record files.txt
```

Restart the terminal after the installation.

<strong>3.</strong> Install `stm32loader` on your robot:

```bash
cd ~/ && git clone https://github.com/husarion/stm32loader.git
cd ~/stm32loader && sudo python setup.py install --record files.txt
```

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

Download the appropriate firmware to your ROSbot and save it in `/home/husarion/`:

- [`ROSbot 2.0 fw v0.10.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.10.1.bin)
- [`ROSbot 2.0 Pro fw v0.10.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.10.1.bin)

Before uploading the firmware using `stm32loader` make sure you have the `husarnet-configurator` service disabled.

> The following steps remove the software bootloader used by Husarion Cloud. You can flash it again in any moment in case you want to return to hFramework firmware.

To upload the firmware run:

```bash
sudo stm32loader -c tinker -u -W
```

```bash
sudo stm32loader -c tinker -e -w -v rosbot-2.0-***.bin
```

PRO:

```bash
sudo stm32loader -c upboard -u -W
```

```bash
sudo stm32loader -c upboard -e -w -v rosbot-2.0-***.bin
```

Wait until firmware is uploaded.

#### Required ROS packages - `rosbot_ekf`

In order to use mbed firmware the `rosbot_ekf` package have to be installed on your ROSbot. The package incorporates a ready to use Extended Kalman Filter that combines both the imu and encoders measurements to better approximate the ROSbot position and orientation. The package also contains custom messages that are required by the new firmware. To install the package please follow the steps below.

Create new work space and change directory:
```
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
```
Clone rosbot_ekf repository:
```
git clone https://github.com/husarion/rosbot_ekf.git
```
Install dependencies required by rosbot_ekf package:
```
sudo apt-get install ros-kinetic-robot-localization
```
Change directory and build code using catkin_make:
```
cd ~/ros_workspace
catkin_make
```
To set it up permanently, open .bashrc file in text editor:
```
nano ~/.bashrc
```
Go to the end of file and add line:
```
. /home/husarion/ros_workspace/devel/setup.sh
```

To launch rosserial communication and Kalman filter for mbed firmware run:

```
roslaunch rosbot_ekf all.launch
```

For PRO version add parameter:

```
roslaunch rosbot_ekf all.launch rosbot_pro:=true
```

## Setup web user interface for ROSbot

Type the following lines in the terminal to update the package list and upgrade packages:

```
sudo apt update
sudo apt dist-upgrade
```

Install the required packages:

`sudo apt install python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx`

and:

`python -m pip install --user tornado==4.5.3 python-wifi ifparser`

Create a new work space (you can skip next two lines if you have build `rosbot_ekf` package):

`mkdir ~/ros_workspace`

`mkdir ~/ros_workspace/src`

Change directory:

`cd ~/ros_workspace/src`

Clone repository containing rosbot webui to `~/ros_workspace/src/`:

`git clone https://github.com/husarion/rosbot_webui.git`

Clone `husarion_ros` repository:

`git clone https://github.com/husarion/husarion_ros.git`

Change directory and build code using catkin_make:

`cd ~/ros_workspace`

`catkin_make`

Add environmental variables by executing this in Linux command line:

`source devel/setup.sh`

> **Note that you have to do it every time you want to use ROSbot webui**. To set it up permanently, open .bashrc file in text editor:
>
> `nano ~/.bashrc`
>
> Go to the end of file and add line:
>
> `. /home/husarion/ros_workspace/devel/setup.sh`
>
> Staying in terminal issue command:

`sudo nano /etc/nginx/sites-enabled/default`

This will open text editor with configuration file. Find line:

`root /var/www/html;`

and change it to:

`root /home/husarion/ros_workspace/src/rosbot_webui/edit;`

To exit text editor press: "Ctrl + x", "y", "Enter"

Again in terminal issue command:

`sudo systemctl restart nginx`

## Usage

Programming procedure needs to be done only once, on further uses, you can start from this point:

- Turn on your ROSbot.
- Click "edit" next to your device name and select "More" where you will find a local IP address of your device.
- Open a terminal and ssh to that previous remembered IP address using that command: `ssh husarion@x.x.x.x` (instead of "x.x.x.x" type your local IP address - your laptop should be in the same Wi-Fi network as you robot)
- Note the address next to "Local IP" (in "Edit"->"More" section), you will need it in a while.
- In terminal issue following command:

If you use Mbed firmware:

_If you use ROSbot 2.0:_

`roslaunch rosbot_webui demo_rosbot_mbed_fw.launch`

_If you use ROSbot 2.0 PRO:_

`roslaunch rosbot_webui demo_rosbot_pro_mbed_fw.launch`

If you use deprecated hFramework firmware:

_If you use ROSbot 2.0:_

`roslaunch rosbot_webui demo.launch`

_If you use ROSbot 2.0 PRO:_

`roslaunch rosbot_webui demo_rosbot_pro.launch`

- Connect your laptop or mobile device to the same network as ROSbot.
- Launch web browser and type the local IP of your ROSbot (the one you noted before)
- You should see interface as below, use it to test and control your ROSbot.

Also, you can check how it works in gazebo simulation:

`roslaunch rosbot_webui demo_gazebo.launch`

<div><center><img src="https://user-images.githubusercontent.com/29305346/62871739-5425ad00-bd1c-11e9-9e6c-b52262e2282a.png"/><center></div>

What can you see and how to use it:

- In the upper part you can see two red rectangles, the right one is for changing map zoom, the left one is for changing clipping distance - distance from where image will be set to black.
- In the corners of camera reading image you can see distance sensors reading.
- In the left lower part of page you can see navigation panel here you can control your rosbot using virtual joystick.
- In the right lower part of page you can see rosbot status - battery, position, orientation.
- To create exploration task (rosbot will try to map whole area) just press _EXPLORE_ button.
- If you want to use portrait orientation of web page you can click and slide to left/right side, this will change displayed section.

> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](https://husarion.com/manuals/rosbot-manual/#rear-panel-description) if battery level is low). Charging manual is [here](https://husarion.com/manuals/rosbot-manual/#charging-rosbot).
