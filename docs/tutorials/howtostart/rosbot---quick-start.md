---
title: ROSbot - quick start
sidebar_label: 3. ROSbot - quick start
id: rosbot---quick-start
---

ROSbot 2.0 and ROSbot 2.0 PRO are autonomous, open source robot platforms running on CORE2-ROS controller. It can be used as a learning platform for Robot Operating System as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can purchase it <a href="https://store.husarion.com/">here</a>.

## Unboxing ##

What's in the box:

* carrying case
* ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
* Wi-Fi 2.4GHz antenna
* 3x 18650 Li-Ion rechargeable batteries
* universal charger with power adapter
* charging cable
* microSD card with the software for ROSbot
* USB to Ethernet adapter

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_unboxing.jpg"/></center></div>

## Assembly ##

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna. 

To mount batteries:

* turn ROSbot upside down
* unscrew battery cover mounted with two screws
* remove the battery cover
* place batteries accordingly to the symbols, keeping the black strip under the batteries
* place battery cover and mount it with screws

To charge the batteries, follow this <a href="https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf">guide</a>.

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

## Low level firmware installation

In the heart of each ROSbot there is a CORE2 board equipped with STM32F4 family microcontroller. The board is responsible for real time tasks like controlling motors, calculating PID regulator output or talking to distance sensors. The high level computation is handled by the SBC (single board computer). 

In order to start your journey with ROSbot platform you have to flash ROSbot's CORE2 board with low level firmware. We provide two firmware options for you to choose from. 

### I. Mbed firmware (recommended)

This firmware version is based on ARM's Mbed OS system. If you're interested in learning more about using Mbed OS check our tutorial [Using CORE2 with Mbed OS](https://husarion.com/tutorials/mbed-tutorials/using-core2-with-mbed-os/). We recommend you also to look at the [ROSbot's Mbed firmware GitHub page](https://github.com/husarion/rosbot-firmware-new).

Before we start complete following steps:
1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Connect the robot to the Wi-Fi network. Check your robot IP address it will be need in next steps. You can do it by typing `ifconfig` in terminal. 

#### stm32loader installation

We will use `stm32loader` tool to upload the firmware to ROSbot. To check if you have this tool already installed on your robot, open the terminal and run:

```bash
$ sudo stm32loader --help
```

If you get `command not found` you will need to finish all the steps below. Otherwise, you just need to complete step one. 

<strong>1.</strong> Disable `husarnet-configurator` and `husarion-shield services` and reboot your ROSbot. These services are responsible for connection to the Husarion Cloud and they also control GPIO pins that are used for uploading the firmware. We will need direct access to them. Run:

```bash
    $ sudo systemctl disable husarnet-configurator
    $ sudo systemctl stop husarnet-configurator
    $ sudo systemctl disable husarion-shield
    $ sudo reboot
```

<strong>2.</strong> Install necessary support libraries on your robot. In the terminal run:

**ROSbot 2.0:**
```bash
$ cd ~/ && git clone https://github.com/TinkerBoard/gpio_lib_python.git
$ cd ~/gpio_lib_python && sudo python setup.py install --record files.txt
```
**ROSbot 2.0 PRO:**
```bash
$ cd ~/ && git clone https://github.com/vsergeev/python-periphery.git
$ cd ~/python-periphery && sudo python setup.py install --record files.txt
```

Restart the terminal after the installation.

<strong>3.</strong> Install `stm32loader` on your robot:

```bash
$ cd ~/ && git clone https://github.com/byq77/stm32loader.git
$ cd ~/stm32loader && sudo python setup.py install --record files.txt
```

You can check if tool works by running following commands:

**ROSbot 2.0:**
```bash
$ sudo stm32loader -c tinker -f F4
```

**ROSbot 2.0 PRO:**
```bash
$ sudo stm32loader -c upboard -f F4
```

#### Programming the firmware (using stm32loader)

We prepared for you `.bin` files ready to be uploaded to your ROSbot. They have following settings: 
* ws2812b driver is enabled by default (check [ROSbot with WS2812B LEDs signalization](https://husarion.com/tutorials/mbed-tutorials/rosbot-and-ws2812b-led-signalization/))
* rosserial baudrate is set to:
    * `500000` for ROSbot 2.0
    * `230400` for ROSbot 2.0 Pro
  
Download the appropriate firmware to your ROSbot and save it in `/home/husarion/`:
* [`ROSbot 2.0 fw v0.7.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.7.1.bin)
* [`ROSbot 2.0 Pro fw v0.7.1`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.7.1.bin)


Before uploading the firmware using `stm32loader` make sure you have the `husarnet-configurator` service disabled. 

> The following steps remove the software bootloader used by Husarion Cloud. You can flash it again in any moment in case you want to return to hFramework firmware.

To upload the firmware run:

```bash
$ sudo stm32loader -c tinker -u -W
```

```bash
$ sudo stm32loader -c tinker -e -w -v rosbot-2.0-***.bin
```

PRO:

```bash
$ sudo stm32loader -c upboard -u -W
```

```bash
$ sudo stm32loader -c upboard -e -w -v rosbot-2.0-***.bin
```

Wait until firmware is uploaded.

### II. Husarion Cloud + hFramework firmware (deprecated)

First you have to connect ROSbot to Husarion cloud. There are three ways to do that: ethernet connection, mouse + keyboard or mobile app. Choose the most comfortable for you.

#### 1: Using display, mouse and keyboard (works for ROSbot 2.0 and for ROSbot 2.0 PRO) ###
ROSbot is basically a computer running Ubuntu, so let's configure it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Connect to a Wi-Fi network
4. Connect to a Husarion cloud
* open https://cloud.husarion.com in your web browser
* click **Add new** button
* enter device name and click **Next**
* copy a code under a QR code (it looks like: `prod|xxxxxxxxxxxxxxxxxxxxxx`)
* open Linux terminal execude a command (including code from the previous step) 
`sudo husarion-register --code "prod|xxxxxxxxxxxxxxxxxxxxxx"`, and then `sudo systemctl restart husarnet-configurator`
* after a few seconds you should see your device online at https://cloud.husarion.com

#### 2: Using Ethernet adapter (works for ROSbot 2.0 and for ROSbot 2.0 PRO) ###
In the ROSbot 2.0 set there is a USB-Ethernet card. Use it for the first setup.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. Connect with ROSbot via ssh, type in your terminal application: `ssh husarion@192.168.0.1` and password `husarion`
5. Connect to a Wi-Fi network
* in the terminal type `nmcli c add type wifi save yes autoconnect yes con-name rosbot20wifi ifname wlan0 ssid <NetworkSSID>` and press Enter
* type `nmcli c modify rosbot20wifi wifi-sec.key-mgmt wpa-psk wifi-sec.psk <Password>` and press Enter to obtain an IP address and connect to the Wi-Fi network
6. Connect to a Husarion cloud
* open https://cloud.husarion.com in your web browser
* click **Add new** button
* enter device name and click **Next**
* copy a code under a QR code (it looks like: `prod|xxxxxxxxxxxxxxxxxxxxxx`)
* open Linux terminal execude a command (including code from the previous step) 
`sudo husarion-register --code "prod|xxxxxxxxxxxxxxxxxxxxxx"`, and then `sudo systemctl restart husarnet-configurator`
* after a few seconds you should see your device online at https://cloud.husarion.com

#### 3: Using hConfig app (only for ROSbot 2.0) ###
That's a deprecated option, so previously mentioned instructions are preferred.

* Press and hold the hCfg button on ROSbots rear panel.
* Turn on the power switch.
* When blue and yellow LEDs starts blinking, release the hCfg button.
* Connect your mobile device to Husarion Wi-Fi and open hConfig app (<a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">hConfig in App Store</a> or <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">hConfig in Google Play</a>) to connect ROSbot to the Wi-Fi network and your user account at <a href="https://cloud.husarion.com">cloud.husarion.com</a> (<a href="https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud">how to do this</a>).

#### Programming the firmware (using Husarion Cloud)

First you will program the low-level firmware running on STM32F4 microcontroller (part of CORE2-ROS controller):

* Turn on your ROSbot.
* At https://cloud.husarion.com  click "edit" next to your device name and select "IDE".
* Create a new project using CORE2 as your board and **"ROSbot default firmware"** as a template.
* Build and upload program to the device (use button with a cloud in left-up corner).
* Go back to main panel of https://cloud.husarion.com

Next you will proceed to ROS part of software running on a single board computer (ASUS Tinker Board):

## Remote connection 

### On Linux ###

Open terminal and start `ssh` connection, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
ssh husarion@ROSBOT_IP
```

Proceed to **Device setup** section.

### On Windows ###

Press `WinKey` + `r` then type `mstsc`.

You will see a window appear:

![Windows RDP](/docs/assets/img/aws-tutorials/quick-start/win_rdp.png)

Type in your device IP address and click connect.

You will see the ROSbot desktop, from the top menu, choose the `Applications` -> `Terminal`.

## Device setup

Type the following lines in the terminal to update the package list and upgrade packages:

```
sudo apt update
sudo apt dist-upgrade
```

Install the required packages:

`sudo apt install python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx`

and:

`python -m pip install --user tornado==4.5.3 python-wifi ifparser`

Create new work space and change directory:

`mkdir ~/ros_workspace`

`mkdir ~/ros_workspace/src`

`cd ~/ros_workspace/src`

Clone repository containing rosbot webui:

`git clone https://github.com/husarion/rosbot_webui.git`

Clone `husarion_ros` repository:

`git clone https://github.com/husarion/husarion_ros.git`

If you use Mbed version of the ROSbot firmware you also need to clone the `rosbot_ekf` repository:

`git clone https://github.com/byq77/rosbot_ekf.git`

Install dependencies required by `rosbot_ekf` package:

`$ sudo apt-get install ros-kinetic-robot-localization`

Change directory and build code using catkin_make: 

`cd ~/ros_workspace`

`catkin_make`

Add environmental variables by executing this in Linux command line:

`source devel/setup.sh`

>**Note that you have to do it every time you want to use ROSbot webui**. To set it up permanently, open .bashrc file in text editor:
>
>`nano ~/.bashrc`
>
>Go to the end of file and add line:
>
>`. /home/husarion/ros_workspace/devel/setup.sh`
>
Staying in terminal issue command: 

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

* Turn on your ROSbot.
* Click "edit" next to your device name and select "More" where you will find a local IP address of your device.
* Open a terminal and ssh to that previous remembered IP address using that command: `ssh husarion@x.x.x.x` (instead of "x.x.x.x" type your local IP address - your laptop should be in the same Wi-Fi network as you robot)
* Note the address next to "Local IP" (in "Edit"->"More" section), you will need it in a while.
* In terminal issue following command:

If you use Mbed firmware:

*If you use ROSbot 2.0:*

`roslaunch rosbot_webui demo_rosbot_mbed_fw.launch `

*If you use ROSbot 2.0 PRO:*

`roslaunch rosbot_webui demo_rosbot_pro_mbed_fw.launch`

If you use deprecated hFramework firmware:

*If you use ROSbot 2.0:*

`roslaunch rosbot_webui demo.launch`

*If you use ROSbot 2.0 PRO:*

`roslaunch rosbot_webui demo_rosbot_pro.launch`

* Connect your laptop or mobile device to the same network as ROSbot.
* Launch web browser and type the local IP of your ROSbot (the one you noted before)
* You should see interface as below, use it to test and control your ROSbot.

Also, you can check how it works in gazebo simulation:

`roslaunch rosbot_webui demo_gazebo.launch`

<div><center><img src="https://user-images.githubusercontent.com/29305346/62871739-5425ad00-bd1c-11e9-9e6c-b52262e2282a.png"/><center></div>

What can you see and how to use it:

* In the upper part you can see two red rectangles, the right one is for changing map zoom, the left one is for changing clipping distance - distance from where image will be set to black.
* In the corners of camera reading image you can see distance sensors reading.
* In the left lower part of page you can see navigation panel here you can control your rosbot using virtual joystick. 
* In the right lower part of page you can see rosbot status - battery, position, orientation.
* To create exploration task (rosbot will try to map whole area) just press _EXPLORE_ button.
* If you want to use portrait orientation of web page you can click and slide to left/right side, this will change displayed section.
  
> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](https://husarion.com/manuals/rosbot-manual/#rear-panel-description) if battery level is low). Charging manual is [here](https://husarion.com/manuals/rosbot-manual/#charging-rosbot).
