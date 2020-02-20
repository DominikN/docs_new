---
title: ROSbot manual
id: rosbot-manual
---

## Overview ##

ROSbot is an autonomous robot platform based on Husarion CORE2-ROS robot controller available in two version: 2.0 and 2.0 PRO. 

<div align="center">
<iframe width="784" height="441" src="https://www.youtube.com/embed/QHJFNMX4Us8" frameborder="0" gesture="media" allowfullscreen></iframe>
</div>

ROSbot is an affordable robot platform for rapid development of autonomous robots. It can be a base for custom service robots, inspection robots and robots working in swarms. Both version integrates:

- 4-wheels mobile platform containing DC motors with encoders and an aluminum frame
- Orbbec Astra RGBD camera
- MPU 9250 inertial sensor (accelerometer + gyro)
- rear panel providing interfaces for additional modules

In ROSbot 2.0:
- CORE2-ROS controller with <a href="https://www.asus.com/pl/Single-Board-Computer/Tinker-Board/">Asus Tinker Board</a> with Rockchip RK3288 up to 1,8GHz, 2GB DDR3 RAM and 16 GB MicroSD.
- RPLIDAR A2 laser scanner

<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/colour_front.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/ROSbot_manual/colour_front_thumb.jpg" alt="CORE2mini board" class="hover-shadow"/>
    </a>
    <span>Front</span>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/colour_back.jpg" data-fancybox="gallery" data-caption="Back">
    <img src="/docs/assets/img/ROSbot_manual/colour_back_thumb.jpg" alt="CORE2mini pinout" class="hover-shadow"/>
    </a>
    <span>Back</span>
</div> 
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/colour_perspective.jpg" data-fancybox="gallery" data-caption="Perspective">
    <img src="/docs/assets/img/ROSbot_manual/colour_perspective_thumb.jpg" alt="CORE2mini pinout" class="hover-shadow"/>
    </a>
    <span>Perspective</span>
</div> 
</div>

In ROSbot 2.0 PRO:
- CORE2-ROS controller with <a href="https://up-board.org/up/specifications/">UpBoard UP</a> with Intel® ATOM™ x5-Z8350 Processors 64 bits up to 1.92GHz, 4GB DDR3L RAM and 32GB eMMC.
- RPLIDAR A3 laser scanner

<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/pro_colour_front.jpg" data-fancybox="gallery" data-caption="Front">
    <img src="/docs/assets/img/ROSbot_manual/pro_colour_front_thumb.jpg" alt="CORE2mini board" class="hover-shadow"/>
    </a>
    <span>Front</span>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/pro_colour_back.jpg" data-fancybox="gallery" data-caption="Back">
    <img src="/docs/assets/img/ROSbot_manual/pro_colour_back_thumb.jpg" alt="CORE2mini pinout" class="hover-shadow"/>
    </a>
    <span>Back</span>
</div> 
<div class="img-container w3">
    <a href="/docs/assets/img/ROSbot_manual/pro_colour_perspective.jpg" data-fancybox="gallery" data-caption="Perspective">
    <img src="/docs/assets/img/ROSbot_manual/pro_colour_perspective_thumb.jpg" alt="CORE2mini pinout" class="hover-shadow"/>
    </a>
    <span>Perspective</span>
</div> 
</div>

If you do not own ROSbot yet, you can purchase it <a href="https://store.husarion.com/">here</a>.

You can also test the performance of ROSbot using our simulation model in Gazebo environment. It is available here, at our <a href="https://github.com/husarion/rosbot_description">GitHub page</a>.

![ROSbot gazebo](/docs/assets/img/ROSbot_manual/rosbot_gazebo.png "ROSbot gazebo")

You can find free <b>ROS tutorials</b> dedicated for ROSbot under this <a href="https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/">link</a>. They will guide you through different aspects of programming autonomous vehicles in ROS

## Hardware guide ##

## Specification ##

<table class="text_table">
    <tr>
       <th>Attribute</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Dimensions with camera and LiDAR</td>
        <td>200 x 235 x 220mm / 7.87 x 9.25 x 8.66in [L x W x H]</td>
    </tr>
    <tr>
        <td>Dimensions without LiDAR</td>
        <td>200 x 235 x 146mm / 7.87 x 9.25 x 5.74in [L x W x H]</td>
    </tr>
    <tr>
        <td>Dimensions without camera and LiDAR</td>
        <td>200 x 235 x 106mm / 7.87 x 9.25 x 4.17in [L x W x H]</td>
    </tr>
    <tr>
        <td>Weight</td>
        <td>2,84kg / 100oz (with camera and LiDAR), 2,45kg / 86oz (without camera and LiDAR)</td>
    </tr>
    <tr>
        <td>Wheel diameter / Clearance / Wheelbase</td>
        <td>85mm / 22mm / 105mm</td>
    </tr>
    <tr>
        <td>Chassis material</td>
        <td>Powder-coated aluminum plate, 1.5mm thick</td>
    </tr>
    <tr>
        <td>Maximum translational velocity</td>
        <td>1.25 m/s</td>
    </tr>
    <tr>
        <td>Maximum rotational velocity</td>
        <td>420 deg/s (7.33 rad/s)</td>
    </tr>
    <tr>
        <td>Maximum load capacity</td>
        <td>10kg / 352oz</td>
    </tr>
    <tr>
        <td>Battery life</td>
        <td>1.5h - 5h</td>
    </tr>
</table>

## Components ##

![Side scheme](/docs/assets/img/ROSbot_manual/scheme_side.png "Side scheme")

![Back](/docs/assets/img/ROSbot_manual/colour_back.jpg "Scheme back")

### Components description ###

<table class="text_table">
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
    <tr>
        <td>Infrared distance sensor</td>
        <td>4</td>
        <td>VL53L0X Time-of-Flight distance sensor with up to 200 cm range, <a href="https://www.pololu.com/file/0J1187/VL53L0X.pdf"> more details</a>.</td>
    </tr>
    <tr>
        <td>DC motor</td>
        <td>4</td>
        <td>Xinhe Motor XH-25D
		Motor used: RF-370, 6VDC nominal, 5000rpm
		No load speed at the output shaft: 165 rpm
		Stall torque: 2.9 kg*cm
		Stall current: 2.2A
		Gear ratio: ~34 (exact ratio is 30613/900)
		Encoder: magnetic, 48ppr, 12 poles</td>
    </tr>
    <tr>
        <td>IMU sensor</td>
        <td>1</td>
        <td>Powerful 9-Axis Accel/Gyro/Magnetometer sensor with MPU-9250, <a href="https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf"> more details</a>.</td>
    </tr>
    <tr>
        <td>RGBD camera</td>
        <td>1</td>
        <td>Orbbec Astra with RGB image size 640x480 and depth image size 640x480.</td> 
    </tr>
    <tr>
        <td>Batteries</td>
        <td>3</td>
        <td>Li-Ion 18650 protected, rechargeable batteries, 3500mAh capacity, 3.7V nominal voltage <br>
		Note: Device may be shipped interchangeably with similar batteries.</td>
    </tr>
    <tr>
        <td>Antenna</td>
        <td>1</td>
        <td>Connected directly to the ASUS Tinker Board Wi-Fi module. Uses an RP-SMA(m) <-> I-PEX MHF4 cable to connect the antenna with SBC.</td>
    </tr>	
</table>



**In ROSbot 2.0:**



<table class="text_table">
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2-ROS</td>
        <td>1</td>
        <td>Advanced version of CORE2 with an ASUS Tinker board computer. CORE2 real-time controller is based on STM32F407 microcontroller. The SBC runs on Ubuntu-based OS, customized to use ROS.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A2, 360 degree and up to 8m range, <a href="https://www.slamtec.com/en/Lidar/A2"> more details</a>.</td>
    </tr>
</table>



**In ROSbot 2.0 PRO:**



<table class="text_table">
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>CORE2-ROS</td>
        <td>1</td>
        <td>Advanced version of CORE2 with an Up Board computer. CORE2 real-time controller is based on STM32F407 microcontroller. The SBC runs on Ubuntu-based OS, customized to use ROS.</td>
    </tr>
    <tr>
        <td>LIDAR</td>
        <td>1</td>
        <td>RpLidar A3, 360 degree and up to 25m range, <a href="https://www.slamtec.com/en/Lidar/A3"> more details</a>.</td>
    </tr>
</table>

## Rear panel description ##

![Rear panel description](/docs/assets/img/ROSbot_manual/ROSbot2_rear_panel_v1.1.png "Rear panel description")

<table class="text_table">
    <tr>
       <th>Component</th>
       <th>Quantity</th>
       <th>Description</th>
    </tr>
    <tr>
        <td>Antenna connector</td>
        <td>1</td>
        <td>Wi-Fi antenna RP-SMA socket. Required for Wi-Fi connectivity.</td>
    </tr>
    <tr>
        <td>USB</td>
        <td>2</td>
        <td>USB 2.0 host ports from SBC.</td>
    </tr>
    <tr>
        <td>HDMI</td>
        <td>1</td>
        <td>HDMI output from SBC.</td>
    </tr>
    <tr>
        <td>Power switch</td>
        <td>1</td>
        <td>Turns ROSbot completely ON or OFF.</td>
    </tr>
    <tr>
        <td>LEDs</td>
        <td>6</td>
        <td>LR1(yellow), LR2(blue), L1(red), L2(green), L3(green), PWR(red), more details <a href="/manuals/core2#leds-and-buttons">here</a>.</td>
    </tr>
    <tr>
        <td>reset button</td>
        <td>1</td>
        <td>Button used for reset CORE2.</td>
    </tr>
    <tr>
        <td>hCfg button</td>
        <td>1</td>
        <td>Button used for connecting ROSbot to [Husarion Cloud](https://cloud.husarion.com/).</td>
    </tr>
    <tr>
        <td>hBtn</td>
        <td>2</td>
        <td>hBtn1, hBtn2 - programmable buttons.</td>
    </tr>	
    <tr>
        <td>Outputs for servo</td>
        <td>6</td>
        <td>Servo output with PWM, more details <a href="core2#hservo">here</a>.</td>
    </tr>
    <tr>
        <td>USB serial</td>
        <td>1</td>
        <td>USB serial port used for debugging the firmware on CORE2-ROS controller.</td>
    </tr>
    <tr>
        <td>Charging connector</td>
        <td>1</td>
        <td>6-pin connector for charging internal Li-Ion batteries.</td>
    </tr>
    <tr>
        <td>DC power input</td>
        <td>1</td>
        <td>DC for working with external 12V power supply. Use the power supply included with charger or any 12V, min. 5A power supply with 5.5/2.5mm plug (center-positive).</td>
    </tr>
    <tr>
        <td>Time-of-Flight distance sensor</td>
        <td>2</td>
        <td>VL53L0X Time-of-Flight distance sensor with up to 200 cm range, more details <a href="https://www.pololu.com/file/0J1187/VL53L0X.pdf">here</a>.</td>.
    <tr>
        <td>hExt</td>
        <td>1</td>
        <td>12xGPIO, 7x ADC, SPI, I2C, UART, more details <a href="core2#hext">here</a>.     </td>
    </tr>
    <tr>
        <td>hSens</td>
        <td>1</td>
        <td>4 xGPIO, ADC, UART, more details <a href="core2#hsensor">here</a>.</td>
    </tr>
</table>

## Power supply ##

ROSbot is powered from an internal, rechargeable Li-Ion battery pack that contains 3 Li-Ion cells, connected in series. This type of connection is called “3S”. The schematic below explains how the cells are wired together and with the charging connector (on ROSbot side).

<img width=50% src="/docs/assets/img/ROSbot_manual/batt_connection.png" alt="Battery connections" />

The BAT+ and BAT- are the power connections and the “bal Bxx” wires are used to monitor the voltage on each cell. It is strongly recommended to keep equal voltages on each cell during the charging process. The charger included with ROSbot can charge batteries in the described way and, thanks to that, the long life of the battery set is possible.

The nominal voltage of each cell is 3.7V but the useful range is 3.2V to 4.2V.

**Important - discharge indicator**
If only the right firmware is preloaded to the internal controller (CORE2), the LED1 is programmed to indicate the power status:
- the LED1 is on when the robot is turned on
- the LED1 is blinking when battery is low – please charge immediately!

Please make sure that the user firmware always contains the function that monitors the supply voltage level. Deep discharging of batteries may decrease their lifecycle. Discharging to the voltage lower than 3.0V/cell can also trigger the over discharge protection. If the voltage is too low, turn ROSbot off and charge batteries as soon as possible.

## Charging ROSbot ##

<a href="/docs/assets/img/ROSbot_manual/charger+cables+PSU.jpg" data-fancybox data-caption="Charging kit">
<img src="/docs/assets/img/ROSbot_manual/charger+cables+PSU.jpg" alt="Charging kit" width="50%" />
</a>
The ROSbot kit contains the Redox Beta charger. It is an universal charger, suitable for charging NiCd, NiMH, Li-Po, Li-Fe, Li-Ion and Pb (AGM, VRLA) batteries. ROSbot shall be charged using an included charger and cable.

Charger kit includes:
- Redox Beta charger
- AC/DC power adapter 100...240V to 12V 5A with 5.5/2.5mm plug on the 12V side
- a cable to connect charger with ROSbot charging port

**Quick charging guide:**
1. Connect the power adapter to the charger and the output cable between charger and ROSbot (2 connectors on charger side, 1 black connector to ROSbot charging port).
2. Use red and blue buttons to select “LiPo BATT” mode and press [Start].
3. Use arrows to select “LiPo CHARGE” mode.
4. Press [Start] - the current value should start blinking. Use arrows to set the current to 1.5A. 
5. Press [Start] again - the voltage value should start blinking. Select “11.1V(3S)” using arrows. The picture below shows the desired result.
6. Press and hold [Start] for 2 seconds. The charger should now ask for confirmation. Press [Start] again. The charging process should begin now.
7. When the charging will be finished (after about 3 hours), the charger will generate a loud “beep” sound and will finish charging at the same time.

![Charge config](/docs/assets/img/ROSbot_manual/charge-config.png "Charge config")

If you need more information about charging, please read the [Charging manual for ROSbot](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf) in PDF format.

**Notes** 
- You can change charging current to maximum 3A. Please note that a regular charging with the maximum current can shorten the battery life.
- If you are going to use ROSbot stationary for a long time, you can use ROSbot with charger connected (and charging) all the time. It will increase the batteries lifetime. Align the charging current to keep the voltage at about 11.1V-12V.
- In case you need to replace batteries, use only 18650 Li-Ion batteries, with the capacity in a range of 1800...3500mAh and with a protection circuit! Using unprotected batteries may result in serious injuries or fire.
- Unplug charging connectors carefully. You shall not unplug the charger connectors holding the wires. The balancer connection on ROSbot side has a latching tab (see photo below) that must be pressed before unplugging. On the charger side there is no latching tab but you should also unplug this connector holding the white plug.

![Latched connector](/docs/assets/img/ROSbot_manual/charger-connector.jpg "Latched connector")

## Software ##

Software for ROSbot can be divided into 2 parts:
 * A [low-level firmware](https://github.com/husarion/rosbot-firmware-new) that works on the real-time controller (CORE2). It can be developed using [Visual Studio Code IDE](/tutorials/mbed-tutorials/using-core2-with-mbed-os/).
 * OS based on Ubuntu 16.04, which runs on the SBC (ASUS Tinker Board or UpBoard) and contains all components needed to start working with ROS immediately. The microSD card with OS is included with each ROSbot. The OS has been modified to make the file system insensitive to sudden power cuts.


### ROS API ###

Below are topics and services available in ROSbot:

| Topic | Message type | Direction | Node |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Description&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|
| --- | --- | --- | --- | --- |
| `/mpu9250` | `rosbot_ekf/Imu` | publisher | `/serial_node` | Raw IMU data in custom message type |
| `/range/fl`| `sensor_msgs/Range` | publisher | `/serial_node` | Front left range sensor raw data |
| `/range/fr`|`sensor_msgs/Range` | publisher | `/serial_node` | Front right range sensor raw data |
| `/range/rl`| `sensor_msgs/Range` | publisher | `/serial_node` | Rear left range sensor raw data |
| `/range/rr`| `sensor_msgs/Range` | publisher | `/serial_node` | Rear right range sensor raw data |
| `/joint_states`| `sensor_msgs/JointState` | publisher | `/serial_node` | Wheels rotation angle |
| `/battery` | `sensor_msgs/BatteryState` | publisher | `/serial_node` | Battery voltage |
| `/buttons` | `std_msgs/UInt8` | publisher | `/serial_node` | User buttons state, details in [User buttons](#user-buttons) section |
| `/pose` | `geometry_msgs/PoseStamped` | publisher | `/serial_node` | Position based on encoders |
| `/odom/wheel` | `nav_msgs/Odometry` | publisher | `/msgs_conversion` | Odometry based on wheel encoders |
| `/velocity` | `geometry_msgs/Twist` | publisher | `/serial_node` | Odometry based on encoders |
| `/imu` | `sensor_msgs/Imu` | publisher | `/msgs_conversion` | IMU data wrapped in standard ROS message type |
| `/odom` | `nav_msgs/Odometry` | publisher | `/rosbot_ekf` | Odometry based on sensor fusion |
| `/tf` | `tf2_msgs/TFMessage` | publisher | `/rosbot_ekf` | ROSbot position based on sensor fusion |
| `/set_pose` | `geometry_msgs/` `PoseWithCovarianceStamped` | subscriber | `/rosbot_ekf` | Allow to set custom state of EKF |
| `/cmd_vel` | `geometry_msgs/Twist` | subscriber | `/serial_node` | Velocity commands |
| `/config` | `rosbot_ekf/Configuration` | service server | `/serial_node` | Allow to control behaviour of CORE2 board, detaild in [CORE2 config](#core2-config) section |

#### External documentation ####

 - Orbbec Astra camera API is documented in [driver repository](https://github.com/orbbec/ros_astra_camera)

 - Slamtec RpLidar scanner API is documented in [driver repository](https://github.com/Slamtec/rplidar_ros)


#### User buttons ####

User button message is published only once when button is pushed. In case when both buttons are presse at the same time, two messages will be pubilshed.
Possible values are:
 - `1` - button 1 pressed
 - `2` - button 2 pressed

#### CORE2 config ####

Config message definition `rosbot_ekf/Configuration`:
```
string command
string data
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 COMMAND_NOT_FOUND=2
string data
uint8 result
```
Available commands:

- SLED - Set LED state, data structure is `LED_NUMBER LED_STATE`, where:
    
    `LED_NUMBER` is number of LED, could be `1`, `2` or `3`
    
    `LED_STATE` is desired LED state, could be `0` to set LED off and `1` to set LED on

- EIMU - Enable/disable IMU, possible values:
  
    `'1'` - enable
  
    `'0'` - disable

- RIMU - Reset IMU (for Kalman related odometry)

    To reset IMU MPU9250 call with empty `data` field.

- EJSM - Enable/disable joint state message publication, possible values

    `'1'` - enable

    `'0'` - disable

- RODOM - Reset odometry

    To reset odometry call with empty `data` field.

- CALI - Odometry valibration (update coefficients), data structure is: `X Y`, where

    `X` - diameter_modificator value

    `Y` - tyre_deflation value

- EMOT - Enable/disable motors, possible values:

    `'0'` - disconnect motors

    `'1'` - connect motors

- SANI - Set WS2812B LEDs animation
    This functionality is not default for ROSbots. It requires WS2812B LED stripe connected to servo 1 output on ROSbot back panel and rebuilding firmware with custom configuration.
    To enable the WS2812B interface open the mbed_app.json file and change the line:
    ```
    "enable-ws2812b-signalization": 0
    ```
    to
    ```
    "enable-ws2812b-signalization": 1
    ```

    Possible values:
        
    `O` - OFF
    
    `S <hex color code>` - SOLID COLOR
    
    `F <hex color code>` - FADE IN FADE OUT ANIMATION
    
    `B <hex color code>` - BLINK FRONT/REAR ANIMATION
    
    `R` - RAINBOW ANIMATION



 ### System reinstallation ###
 
 In some cases you will need to restore ROSbot system to its default settings:
 - in case of accidential damage of the system,
 - to update the OS (it can be udpated remotely, but flashing the microSD card can be easier sometimes),
 - to clear all user changes and restore factory settings.

This process will differ depending on ROSbot version that you have.

#### ROSbot 2.0 ####

1. Extract SD card from ROSbot, by pushing card carefully until it is released back by card holder, thel pull it out. You can find SD card slot on ROSbot right side.
 ![SD card side view](/docs/assets/img/ROSbot_manual/sd_card_side_view.png) 
2. Download image for Raspberry Pi/Tinkerboard from [here](https://husarion.com/downloads) (there is a single image for both platforms).
3. Extract downloaded image (For this process we recommend using [7zip](https://www.7-zip.org/))
4. Flash the extracted image onto SD card (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
 - If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 
 - Download [Etcher](https://www.balena.io/etcher/) and install it.
 - Connect an SD card reader with the SD card inside.
 - Open Etcher and select from your hard drive .img file that you extracted.
 - Select the SD card you wish to write your image to.
 - Review your selections and click 'Flash!' to begin writing data to the SD card.
5. Insert SD card back to ROSbot
6. Proceed to [Connect ROSbot to your Wi-Fi network](#connect-rosbot-to-your-wi-fi-network) section.

#### ROSbot 2.0 PRO ####

Before you begin, you will need:
- USB drive (at least 8GB)
- Mouse, keyboard and USB hub
- Display with HDMI cable

1. Download Ubuntu 16.04 installation image from official [Ubuntu Releases](http://releases.ubuntu.com/16.04/ubuntu-16.04.6-desktop-amd64.iso).
2. Flash Ubuntu on USB drive (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
 - Download [Etcher](https://www.balena.io/etcher/) and install it.
 - Plug in USB drive into your computer.
 - Open Etcher and select from your hard drive .iso file that you downloaded.
 - Select the USB drive you wish to write your image to.
 - Review your selections and click 'Flash!' to begin writing data to the USB drive.
3. Plug keyboard and mouse through USB hub to one of the USB-A ports on ROSbot rear panel.
4. Connect monitor to HDMI port on ROSbot rear panel.
5. Plug in USB drive into second USB-A port on ROSbot rear panel.
6. Press "Esc" during booting.
7. You will see blue window with "Enter Password", press "Enter".
8. Click "Right arrow" to enter Boot card and change Boot Option Priorities for your USB drive.
9. Save & Exit.
10. After Restart chose option Install Ubuntu (remember to choose option with erasing new Ubuntu and remove all part of the old one).
11. After installation go to [files.husarion.com](https://files.husarion.com) and download `upboard.sh` file.
12. Change file permissions: `sudo chmod 777 upboard.sh`
13. Run file: `sudo ./upboard.sh` (remember to chose option "NO" when you will see a dialog window with question about abandon kernel removal).
14. After finish, reboot device.
15. Proceed to [Connect ROSbot to your Wi-Fi network](#connect-rosbot-to-your-wi-fi-network) section.

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

- [`ROSbot 2.0 fw v0.10.0`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.10.0.bin)
- [`ROSbot 2.0 Pro fw v0.10.0`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.10.0.bin)

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

In order to use mbed firmware the `rosbot_ekf` package have to be installed in your ROSbot. The package incorporates a ready to use Extended Kalman Filter that combines both the imu and encoders measurements to better approximate the ROSbot position and orientation. The package also contains custom messages that are required by the new firmware. To install the package please follow the steps below.

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

### II. Husarion Cloud + hFramework firmware (deprecated)

[hFramework](https://github.com/husarion/hFramework) is a framework used to create the previous version of ROSbot low-level firmware. It is not recommended to be used. Instead of hFramework based firmware please consider using mbed based ROSbot firmware mentioned. However that instruction is preserved for backward compatibility.

At first you have to connect ROSbot to Husarion cloud. There are two ways to do that: over the terminal and using hConfig app.

**a. Using Linux terminal**

- open https://cloud.husarion.com in your web browser
- click **Add new** button
- enter device name and click **Next**
- copy a code under a QR code (it looks like: `prod|xxxxxxxxxxxxxxxxxxxxxx`)
- open Linux terminal execude a command (including code from the previous step)
  `sudo husarion-register --code "prod|xxxxxxxxxxxxxxxxxxxxxx"`, and then `sudo systemctl restart husarnet-configurator`
- after a few seconds you should see your device online at https://cloud.husarion.com

**b. Using hConfig app (only for ROSbot 2.0)**
That's a deprecated option, so previously mentioned instructions are preferred.

- Press and hold the hCfg button on ROSbots rear panel.
- Turn on the power switch.
- When blue and yellow LEDs starts blinking, release the hCfg button.
- Connect your mobile device to Husarion Wi-Fi and open hConfig app (<a href="https://itunes.apple.com/us/app/hconfig/id1283536270?mt=8">hConfig in App Store</a> or <a href="https://play.google.com/store/apps/details?id=com.husarion.configtool2">hConfig in Google Play</a>) to connect ROSbot to the Wi-Fi network and your user account at <a href="https://cloud.husarion.com">cloud.husarion.com</a> (<a href="https://husarion.com/core2/tutorials/howtostart/run-your-first-program/#run-your-first-program-connecting-to-the-cloud">how to do this</a>).

If your ROSbot is connected to Husarion cloud do the following steps:

- Turn on your ROSbot.
- At https://cloud.husarion.com click "edit" next to your device name and select "IDE".
- Create a new project using CORE2 as your board and **"ROSbot default firmware"** as a template.
- Build and upload program to the device (use button with a cloud in left-up corner).
- Go back to main panel of https://cloud.husarion.com


## ROS tutorials ##

ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It's very powerful and 
functional tool dedicated to design robots. We created the set of [ROS tutorials dedicated for this platform](/tutorials/ros-tutorials/1-ros-introduction/ "ROS tutorials dedicated for this platform") to make it easier to familiarize yourself with these frameworks. 

## Configuring ROSbot to work with 5GHz WiFi. ##

By default ROSbot supports WiFi in 2.4GHz band, this is sufficent for most cases.
If you encounter problems with data transfers e.g. due to processing large amounts of data or noise from other networks you can try to use connection in 5GHz band.

### Device installation

To do this, you will need a USB 5GHz WiFi card (any device based on [RTL8811AU or RTL8812AU](https://wikidevi.com/w/index.php?title=Special%3AAsk&q=%5B%5BChip1+model%3A%3ARTL8811AU%5D%5D+OR+%5B%5BChip1+model%3A%3ARTL8812AU%5D%5D&po=%3FInterface%0D%0A%3FForm+factor%3DFF%0D%0A%3FInterface+connector+type%3DUSB+conn.%0D%0A%3FFCC+ID%0D%0A%3FManuf%0D%0A%3FManuf+product+model%3DManuf.+mdl%0D%0A%3FVendor+ID%0D%0A%3FDevice+ID%0D%0A%3FChip1+model%0D%0A%3FSupported+802dot11+protocols%3DPHY+modes%0D%0A%3FMIMO+config%0D%0A%3FOUI%0D%0A%3FEstimated+year+of+release%3DEst.+year%0D%0A&eq=yes&p%5Bformat%5D=broadtable&sort_num=&order_num=ASC&p%5Blimit%5D=500&p%5Boffset%5D=&p%5Blink%5D=all&p%5Bsort%5D=&p%5Bheaders%5D=show&p%5Bmainlabel%5D=&p%5Bintro%5D=&p%5Boutro%5D=&p%5Bsearchlabel%5D=%E2%80%A6+further+results&p%5Bdefault%5D=&p%5Bclass%5D=sortable+wikitable+smwtable&p%5Bsep%5D=&eq=yes) should be fine,we have tested it with **TP-Link Archer T4U** and **D-Link DWA-172**).

**ROSbot 2.0** is supporting the adapters by default, if you have recent image version you can skip update and kernel instal, otherwise do:
```  
sudo apt-get update
sudo apt-get install tinkerboard-kernel
modprobe rtl8812au
```

For **ROSbot 2.0 PRO** some modifications are required:
```bash
sudo apt install linux-headers-generic-hwe-16.04-upboard
git clone https://github.com/abperiasamy/rtl8812AU_8821AU_linux.git
cd rtl8812AU_8821AU_linux/
make
sudo make install
sudo modprobe -a rtl8812au
sudo cp -R . /usr/src/rtl8812AU_8821AU_linux-1.0
sudo dkms add -m rtl8812AU_8821AU_linux -v 1.0
sudo dkms build -m rtl8812AU_8821AU_linux -v 1.0
sudo dkms install -m rtl8812AU_8821AU_linux -v 1.0
```

### WiFi setup

For **Remote Desktop** or keyboard and display, you can use desktop GUI tools to setup connection.

For **SSH** method type `ifconfig` to list network interfaces, you should see now new interface named `wlan1`.

Now, list all available newtworks:

```
sudo iwlist wlan1 scanning | grep ESSID
```

Connect to selected WiFi with:

```bash
nmcli c add type wifi save yes autoconnect yes con-name <NetworkSSID> ifname wlan1 ssid <NetworkSSID>
nmcli c modify <NetworkSSID> wifi-sec.key-mgmt wpa-psk wifi-sec.psk <Password>
```

Remember to replace `<NetworkSSID>` and `<Password>` with name and passowrd of chosen network.

ROSbot will try to connect to this network each time it boots.

## Docs and links ##
All helpful documents and links in one place:

* [ROSbot Safety Instructions](https://files.husarion.com/docs2/ROSbot_safety_instructions_1.0.pdf "ROSbot Safety Instructions") - important!
* [Charging manual for ROSbot](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf)
* [ROSbot drawing](https://files.husarion.com/docs2/ROSbot_2.0-Drawing.pdf)
* [ROS tutorials for ROSbot](/tutorials/ros-tutorials/1-ros-introduction)
* [ROSbot on ROS webpage](https://robots.ros.org/rosbot-2.0/)
* [ROSbot on ROS Wiki](http://wiki.ros.org/Robots/ROSbot-2.0)
* [URDF model of ROSbot - for Gazebo integrated with ROS](https://github.com/husarion/rosbot_description)
* [ROSbot project on hackaday.io](https://hackaday.io/project/21885-rosbot-autonomous-robot-platform "ROSbot project on hackaday.io")
* [ROSbot project on instructables.com](http://www.instructables.com/id/ROSbot-Autonomous-Robot-With-LiDAR/ "ROSbot project on instructables.com")
