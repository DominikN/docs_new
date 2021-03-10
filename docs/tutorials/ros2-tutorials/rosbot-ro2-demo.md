---
title: ROS 2 Demo for ROSbot 2.0
sidebar_label: 0. ROSbot Demo
id: rosbot-ros2-demo
---
<!-- 
TODO: add to sidebar.json below "ROS projects section"
      {
        "type": "subcategory",
        "label": "ROS 2 tutorials",
        "ids": [
          "tutorials/ros2-tutorials/rosbot-ros2-demo"
        ]
      }, -->

## Install ROS 2 image on ROSbot

### Get a system image

Working on your laptop, vising https://husarion.com/downloads/, find **Ubuntu 20.04 + ROS2 Foxy + Docker + Husarnet client** and download:
- **Tinker Board** version for ROSbot 2.0 
- **UpBoard** version for ROSbot 2.0 PRO

### Flash the image

To flash the image on ROSbot 2.0 or ROSbot 2.0 PRO, visit [system reinstallation guide](/docs/manuals/rosbot#system-reinstallation).

## First ROSbot Configuration

### Connecting to Wi-Fi

In the ROSbot 2.0 set there is one USB-Ethernet card.

1. Turn on the robot and wait until it boots.
2. Plug in Ethernet adapter (included in set) to USB port in the rear panel
3. Plug in one end of the Ethernet cable into your computer and other one to the adapter
4. To connect with ROSbot via ssh, type in terminal application on the computer: 
    ```bash
    ssh husarion@192.168.0.1
    ```
    a password is also`husarion`

5. To connect to a Wi-Fi network in the terminal type 
    ```bash
    nmcli d wifi connect <WiFi-SSID> password <WiFi-PASSWORD>
    ``` 
6. type `ifconfig` to find your IP address. Save it for later.

### Enabling the Remote Desktop

In the ROSbot's terminal type:

```bash
TODO: Adam opisz tutaj wszystko co trzeba zrobić aby skonfigurować tigerVNC + noVNC, aby remote desktop działał z poziomu przeglądarki internetowej i po reboocie włączał się automatycznie
```

## Launching a demo project

In your web browser type the following URL: `http:/<YOUR_ROSBOT_IP_ADDR>/vnc` to get access to it's remote desktop...

TODO: Adam, opisz tutaj jak odpalić launcha, który ma już wszystko skonfigurowane. Launch ma odpalić RViz'a, gdzie autonomicznie mogę sobie pojeździć robotem i podejrzeć odczyty ze wszystkich sensorów

## Using a demo

TODO: Adam, zrób proszę taki mega proste wytłumaczenie jak korzystać z RViz'a



