---
title: ROSbot 2R - quick start
sidebar_label: 3. ROSbot 2R
id: rosbot2r-quick-start
---

ROSbot 2R is an autonomous, open source robot platforms for research and quick prototyping use cases. It can be used as learning platforms for Robot Operating System (ROS) as well as a base for a variety of robotic applications like inspection robots, custom service robots etc.

If you don't have one, you can get it [here](https://store.husarion.com/).

## Unboxing

What's in the box:

- carrying case
- ROSbot 2R (with optional 3D camera and LiDAR already assembled)
- Wi-Fi 2.4GHz / 5GHz antenna
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

## Accessing ROSbot's Linux terminal

To perform initial network configuration, you need to access ROSbot's Linux terminal first. There are two options

### Option 1: Using display, mouse and keyboard

ROSbot is basically a computer running Ubuntu, so let's open it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Open a `terminal` app

### Option 2: Using an Ethernet adapter

In the ROSbot 2R set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel.
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter.
4. Set a static IP address on your computer for your Ethernet card in a `192.168.77.0/24` subnet, eg:

    - IPv4: `192.168.77.27`
    - mask: `255.255.255.0`

4. To connect with ROSbot via ssh, type in your terminal application: 

    ```bash title="user@mylaptop:~$ ..."
    ssh husarion@192.168.77.2
    ```

    The default password for user `husarion` is also `husarion`.

## Connect ROSbot to your Wi-Fi network

ROSbot 2R is using [netplan](https://netplan.io/) instead of GUI Wi-Fi manager. It allows you to have all physical network interfaces configured from a single file. To connect your ROSbot to a Wi-Fi network edit `/etc/netplan/01-network-manager-all.yaml` file, eg. with `nano`:

```bash title="husarion@rosbot2r:~$ ... "
sudo nano /etc/netplan/01-network-manager-all.yaml
```

And modify lines 31-32 by replacing `PLACE_YOUR_WIFI_SSID_HERE` with your SSID (Wi-Fi network name) and `PLACE_YOUR_WIFI_PASSWORD_HERE` with your Wi-Fi password:

```yaml {31-32} showLineNumbers title="/etc/netplan/01-network-manager-all.yaml"
network:
  version: 2
  renderer: networkd

  ethernets:
    
    # built-in Ethernet port
    eth0:
      dhcp4: no
      dhcp6: no
      addresses:           
       - 192.168.77.2/24

    # USB Ethernet card
    eth1:
      dhcp4: no
      dhcp6: no
      addresses:           
       - 192.168.77.2/24

  wifis:

    # wlan0:  # internal Raspberry Pi Wi-Fi (do not use it when the top cover is used)
    #   optional: true

    wlan1:  # external USB Wi-Fi card (with antenna)
      dhcp4: true
      dhcp6: true
      optional: true
      access-points:
        "PLACE_YOUR_WIFI_SSID_HERE":
          password: "PLACE_YOUR_WIFI_PASSWORD_HERE"

```

**save the file** than, apply the new network setup:

```bash title="husarion@rosbot2r:~$ ... "
sudo netplan apply
```

You can check to which Wi-Fi network you are connected by using this command:

```bash title="husarion@rosbot2r:~$ ... "
iwgetid
```

If your Wi-Fi setup is more complex (eg. if you want to connect to [Eduroam](https://en.wikipedia.org/wiki/Eduroam) based Wi-Fi that is popular in many Universities), visit [netplan configuration examples](https://netplan.io/examples).

4. Open Linux terminal and type `ifconfig` to find your IP address (for `wlan1` network interface). Save it for later.

## Remote access in LAN

While ROSbot is connected to Wi-Fi network you can access it by using it's IPv4 address by:

### SSH

It's the simplest way to access ROSbot if you don't need to use graphic tools. You just have to type:

```
ssh husarion@<ROSBOT_IP>
```
### Remote access over Internet (VPN)

Instead of using local IPv4 address you can access the robot by using it's hostname - both in LAN and over the Internet. You just need to setup a VPN connection (Husarnet VPN client is pre-installed)

#### Get the Join Code from your Husarnet network:

You will find your Join Code at **https://app.husarnet.com**  

 **-> Click on the desired network  
 -> `Add element` button  
 -> `Join Code` tab**

![](/img/howToStart/husarnet.png)

##### Connect your laptop

Install Husarnet VPN client on your laptop:

```bash title="user@mylaptop:~$ ..."
curl https://install.husarnet.com/install.sh | sudo bash
```

```bash title="user@mylaptop:~$ ... "
sudo systemctl restart husarnet
```
```bash title="user@mylaptop:~$ ... "
sudo husarnet join <your_join_code_from_step_1> mylaptop
```

##### Connect your ROSbot 2R

```bash title="husarion@rosbot2r:~$ ... "
sudo systemctl enable husarnet
```

```bash title="husarion@rosbot2r:~$ ... "
sudo systemctl start husarnet
```

```bash title="husarion@rosbot2r:~$ ... "
sudo husarnet join <your_join_code_from_step_1> rosbot2r
```

##### Test the connection

That's all - now you can use your device hostname instead of IPv4 addr, eg.:

```bash title="user@mylaptop:~$ ... "
ssh husarion@rosbot2r
```

## Low level firmware

In the heart of each ROSbot there is a CORE2 board equipped with STM32F4 family microcontroller. The board is responsible for real time tasks like controlling motors, calculating PID regulator output or talking to distance sensors. High level computation is handled by SBC (single board computer) - Raspberry Pi 4

### Mbed firmware

This firmware version is based on ARM's Mbed OS system. If you're interested in learning more about using Mbed OS check our tutorial [Using CORE2 with Mbed OS](/tutorials/mbed/1-enviroment-configuration/). We recommend you also to look at the [ROSbot's Mbed firmware GitHub page](https://github.com/husarion/rosbot-firmware-new).

All additional information about flashing ROSbot firmware and using stm32loader you can find in [ROSbot manual](/manuals/rosbot/#i-mbed-firmware). 

#### Required ROS packages - `rosbot_ekf`

In order to use Mbed firmware the `rosbot_ekf` package have to be installed on your ROSbot. The package incorporates a ready to use Extended Kalman Filter that combines both the IMU and encoders measurements to better approximate the ROSbot position and orientation. The package also contains custom messages that are required by the new firmware. The package is already installed on your ROSbot.


> Note: if you experience any issues, make sure batteries are fully charged ([LED L1 is blinking](/manuals/rosbot/#rear-panel-description) if battery level is low). Charging manual is [here](/manuals/rosbot/#charging-rosbot).
