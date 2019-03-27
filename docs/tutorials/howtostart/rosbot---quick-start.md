---
title: ROSbot - quick start
sidebar_label: 6. ROSbot - quick start
id: rosbot---quick-start
---

# ROSbot - quick start #

ROSbot 2.0 and ROSbot 2.0 PRO are autonomous, open source robot platforms running on CORE2-ROS controller. It can be used as a learning platform for Robot Operating System as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can purchase it <a href="https://store.husarion.com/">here</a>.

## Unboxing ##

What's in the box:

* carrying case
* ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
* Wi-Fi 2.4GHz antenna
* 3x 18650 Li-Ion reachargeable batteries
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
* place batery cover and mount it with screws

To charge the batteries, follow this <a href="https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf">guide</a>.

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

## Connecting to the cloud ##

There are three ways to connect ROSbot to Husarion cloud: ethernet connection, mouse + keyboard or mobile app. Choose the most comfortable for you.

### Option 1: using display, mouse and keyboard (works for ROSbot 2.0 and for ROSbot 2.0 PRO) ###
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

### Option 2: using Ethernet adapter (works for ROSbot 2.0 and for ROSbot 2.0 PRO) ###
In the ROSbot 2.0 set there is a USB-Ethernet card. Use it for the first setup.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. Connect with ROSbot via ssh, type in your terminal application: `ssh husarion@192.168.0.1` and passowrd `husarion`
5. Connect to a Wi-Fi network
* in the terminal type `nmcli c add type wifi save yes autoconnect yes con-name rosbot20wifi ifname wlan0 ssid <NetworkSSID>` and press Enter
* type `nmcli c modify rosbot20wifi wifi-sec.key-mgmt wpa-psk wifi-sec.psk <Password>` and press Enter to obtain an IP address and connect to the WiFi network
6. Connect to a Husarion cloud
* open https://cloud.husarion.com in your web browser
* click **Add new** button
* enter device name and click **Next**
* copy a code under a QR code (it looks like: `prod|xxxxxxxxxxxxxxxxxxxxxx`)
* open Linux terminal execude a command (including code from the previous step) 
`sudo husarion-register --code "prod|xxxxxxxxxxxxxxxxxxxxxx"`, and then `sudo systemctl restart husarnet-configurator`
* after a few seconds you should see your device online at https://cloud.husarion.com

### Option 3: using hConfig app (only for ROSbot 2.0) ###
That's a deprecated option, so previously mentioned instructions are prefferred.

* Press and hold the hCfg button on ROSbots rear panel.
* Turn on the power switch.
* When blue and yellow LEDs starts blinking, release the hCfg button.
* Connect your mobile device to Husarion Wi-Fi and open hConfig app (<a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">hConfig in AppStore</a> or <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">hConfig in Google Play</a>) to connect ROSbot to the Wi-Fi network and your user account at <a href="https://cloud.husarion.com">cloud.husarion.com</a> (<a href="https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud">how to do this</a>).


## Programming ##

First you will program the low-level firmware running on STM32F4 microcontroller (part of CORE2-ROS controller):

* Turn on your ROSbot.
* At https://cloud.husarion.com  click "edit" next to your device name and sellect "IDE".
* Create a new project using CORE2 as your board and **"ROSbot default firmware"** as a template.
* Build and upload program to the deivce (use button with a cloud in left-up corner).
* Go back to main panel of https://cloud.husarion.com

Next you will proceed to ROS part of software running on a single board computer (ASUS Tinker Board):

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

`sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx`

Create new work space and change directory:

`mkdir ~/ros_workspace`

`mkdir ~/ros_workspace/src`

`cd ~/ros_workspace/src`

Clone repository containing rosbot webui:

`git clone https://github.com/husarion/rosbot_webui.git`

Clone `husarion_ros` repository:

`git clone https://github.com/lukaszmitka/husarion_ros.git`

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
>Staying in terminal issue command: 
>
>`sudo nano /etc/nginx/sites-enabled/default`
>
>This will open text editor with configuration file. Find line:  
>
>`root /var/www/html;`
>
>and change it to:  
>
>`root /home/husarion/ros_workspace/src/rosbot_webui/edit;`
>To exit text editor press: "Ctrl + x", "y", "Enter"
>
>Again in terminal issue command:  
>
>`sudo systemctl restart nginx`


## Usage

Programming procedure needs to be done only once, on further uses, you can start from this point:

* Turn on your ROSbot.
* Click "edit" next to your device name and sellect "More" where you will find a local IP address of your device.
* Open a terminal and ssh to that IP address using that command: `ssh husarion@x.x.x.x` (instead of "x.x.x.x" type your local IP address - your laptop should be in the same Wi-Fi network as you robot)
* Note the address next to "Local IP" (in "Edit"->"More" section), you will need it in a while.
* In terminal issue following command:

`roslaunch rosbot_webui demo.launch`

* Connect your laptop or mobile device to the same network as ROSbot.
* Launch web browser and type the local IP of your ROSbot (the one you noted before)
* You should see interface as below, use it to test and control your ROSbot.

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_UI.png"
/></center></div>

> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](https://husarion.com/manuals/rosbot-manual/#rear-panel-description) if battery level is low). Charging manual is [here](https://husarion.com/manuals/rosbot-manual/#charging-rosbot).
