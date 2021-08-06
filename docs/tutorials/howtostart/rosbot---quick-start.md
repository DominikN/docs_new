---
title: ROSbot - quick start
sidebar_label: 2. ROSbot - quick start
id: rosbot---quick-start
---

ROSbot 2.0 and ROSbot 2.0 PRO are autonomous, open source robot platforms running on CORE2-ROS controller. They can be used as learning platforms for Robot Operating System (ROS) as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can get it [here](https://store.husarion.com/).

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

![](/img/howToStart/ROSbot_unboxing.jpg)

## Assembly

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna.

To mount batteries:

- turn ROSbot upside down
- unscrew battery cover mounted with two screws
- remove the battery cover
- place batteries accordingly to the symbols, keeping the black strip under the batteries
- place battery cover and mount it with screws

To charge the batteries, follow this [guide](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf).

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

- In the terminal, type `nmtui` and press Enter. You should see:

![](/img/howToStart/nmtui_1.png)

- Go to `Active a connection` and tap `Enter`

![](/img/howToStart/nmtui_2.png)

- Chose you Wi-Fi network and tap `Enter` one more time. Enter your password, confirm it and tap `Esc` to get back to main menu.


![](/img/howToStart/nmtui_3.png)

- Use `Quit` to close `nmtui`.

6. Type `ifconfig` to find your IP address. Save it for later.

## Remote access in LAN

While ROSbot is connected to Wi-Fi network you can access it by using it's IPv4 address by:

### SSH

It's the simplest way to access ROSbot if you don't need to use graphic tools. You just have to type:

```
ssh husarion@<ROSBOT_IP>
```

### Virtual Desktop (VNC)

#### 1. Enable VNC server on the robot

```bash
ssh husarion@<ROSBOT_IP>
...
vncserver -localhost no -geometry 1280x720
```

#### 2. Install VNC client on your laptop to access the remote desktop of the robot:

```bash
sudo apt-get install tigervnc-viewer 
xtigervncviewer <ROSBOT_IP>
```

### Remote access over Internet (VPN)

Instead of using local IPv4 address you can access the robot by using it's hostname - both in LAN and over the Internet. You just need to setup a VPN connection (Husarnet VPN client is pre-installed)

#### Get the Join Code from your Husarnet network:

You will find your Join Code at **https://app.husarnet.com**  

 **-> Click on the desired network  
 -> `Add element` button  
 -> `Join Code` tab**

![](/img/howToStart/husarnet.png)

- Install Husarnet VPN client on your laptop:

```bash
curl https://install.husarnet.com/install.sh | sudo bash
sudo systemctl restart husarnet
```

- Execute in your laptop's terminal:

```bash
sudo husarnet join <your_join_code_from_step_1> mylaptop
```

- Execute in the robot's terminal:

```bash
sudo husarnet join <your_join_code_from_step_1> myrosbot
```

- That's all - now you can use your device hostname instead of IPv4 addr, eg.:

```bash
ssh husarion@myrosbot
```

or

```bash
xtigervncviewer myrosbot
```

## Low level firmware

In the heart of each ROSbot there is a CORE2 board equipped with STM32F4 family microcontroller. The board is responsible for real time tasks like controlling motors, calculating PID regulator output or talking to distance sensors. High level computation is handled by SBC (single board computer) - ASUS Tinker Board (in ROSbot 2.0) or UP Board (in ROSbot 2.0 PRO).

### Mbed firmware

This firmware version is based on ARM's Mbed OS system. If you're interested in learning more about using Mbed OS check our tutorial [Using CORE2 with Mbed OS](https://husarion.com/tutorials/mbed-tutorials/using-core2-with-mbed-os/). We recommend you also to look at the [ROSbot's Mbed firmware GitHub page](https://github.com/husarion/rosbot-firmware-new).

All additional information about flashing ROSbot firmware and using stm32loader you can find in [ROSbot manual](https://husarion.com/manuals/rosbot/#i-mbed-firmware). 

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

![panel accessed through husarnet](/img/software/panel_at_husarnet.png)

You can find detailed description of Route Admin Panel in [software section](https://husarion.com/software/route-admin-panel/).

> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](https://husarion.com/manuals/rosbot-manual/#rear-panel-description) if battery level is low). Charging manual is [here](https://husarion.com/manuals/rosbot-manual/#charging-rosbot).
