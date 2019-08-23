---
id: rosbot-and-ws2812b-led-signalization
sidebar_label: 2. ROSbot with WS2812B LEDs signalization 
title: ROSbot with WS2812B LEDs signalization
---

> This tutorial requires a new, Mbed OS version of the ROSbot firmware!

## Introduction

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img1.jpg" data-fancybox="images" data-caption="ROSbot status illumination">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img1-small.jpg" alt="ROSbot status illumination" class="hover-shadow"/>
</a>

The ROSbot by default has three LEDs that can be used by user's applications to indicate status and battery condition. However, their location at the rear panel and the fact that they have only single color can limit possible use cases. In more demanding applications where there are many distinct robot's states we need more clear, visible and robust solution.  

<p align="center">
<img alt="WS2812B chip" src="/docs/assets/img/mbed-tutorials/ws2812b-chip.jpg" title="WS2812B chip" />
</p>

In this tutorial we will present you with elegant and easy approach to this problem. We will use a popular and cheap ws2812b RGB LED strip. Each LED pixel is individually addressed and it is capable of displaying 256 levels of brightness for each color giving overall 16 millions of colors depth.
The ws2812b IC utilizes the NZR protocol with 800Kbit/s speed which allows high refresh rates. Using these LEDs we can achieve a clear ROSbot's visual state indication.

## Prerequisites

### Hardware

Additionally to ROSbot, basic soldering skills and soldering equipment you will need following items and tools:

* **WS2812b LED strip 1m 30leds/pixels/m** - we will use only 16 pixels, but you can use more. However, these leds are quite power hungry and they can significantly reduce the ROSbot's operation time if used in large quantities (they consume 52.5 mA max per pixel).

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img2.jpg" data-fancybox="images" data-caption="WS2812b">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img2-small.jpg" alt="WS2812b" class="hover-shadow"/>
</a>

* **scissors**
* **electric wire stripper**
* **double sided tape**
* **single-core cable** - three colors
* **heat shrink sleeves** to protect connections between cuted led strips
* **2.54mm (0.1") Pitch Female Connector 3 Position** - to swap with the default connector that comes with the strip

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img3.jpg" data-fancybox="images" data-caption="female connector 3 position">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img3-small.jpg" alt="female connector 3 position" class="hover-shadow"/>
</a>

* **crimping pliers or universal pliers** - to attach crimp pins 

<!-- zbiorcze zdjęcie wszystkich elementów -->

### Software

This tutorial uses the new firmware for ROSbot, based on Mbed OS system. To learn more about the new firmware please get familiar with the firmware's [GitHub page](https://github.com/husarion/rosbot-firmware-new).

We recommend you to check our [Using CORE2 with Mbed OS tutorial](/docs/tutorials/mbed-tutorials/using-core2-with-mbed-os). It will introduce you to the Mbed OS environment and tools.

#### Building from source

To prepare the environment, build and upload the firmware please follow the instructions from the [README](https://github.com/husarion/rosbot-firmware-new/blob/master/README.md) file.

Before building the code, find `mbed_app.json` file in your project  and enable the **ws2812b driver** by changing the line:

```json
"enable-ws2812b-signalization": 0
```

to:

```json
"enable-ws2812b-signalization": 1 
```

If you intend to use more then 16 led pixels (which is the value hardcoded in the firmware) you can easily change that by adding a new line:

```json
"ws2812b-dma-driver.num_leds": x
```

where `x` is the number of leds you want to use.

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img4.png" data-fancybox="images" data-caption="mbed_app.json">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img4.png" alt="mbed_app.json" width="600" class="hover-shadow" />
</a>

Build firmware using `BUILD (RELEASE)` task. The `firmware.bin` file should appear in the directory `BUILD/RELEASE`. To upload the firmware using `stm32loader` please follow the guide from [previous tutorial](/docs/tutorials/mbed-tutorials/using-core2-with-mbed-os#stm32loader-usage).

#### Ready to use firmware packages

If you don't want to go through the procedures above, we prepared for you `.bin` files ready to be uploaded to your ROSbot. They have following settings: 
* ws2812b driver is enabled by default
* number of led pixels is set to 16
* rosserial baudrate is set to:
    * `500000` for ROSbot 2.0
    * `230400` for ROSbot 2.0 Pro

Use `stm32loader` to upload the firmware. Please follow the guide from [previous tutorial](docs/tutorials/mbed-tutorials/using-core2-with-mbed-os#stm32loader-usage).

Downloads:
* [`ROSbot 2.0 fw v0.7.0`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-fw-v0.7.0.bin)
* [`ROSbot 2.0 Pro fw v0.7.0`](https://files.husarion.com/rosbot-firmware/rosbot-2.0-pro-fw-v0.7.0.bin)

<!-- TODO: prepare the firmware files on ownCloud -->

## LED strip installation

<strong>1. Install the new connector.</strong>

First thing you have to do is to install pin crimps for the new connector. I used a crimp pliers, but you can also attach them using a standard pliers. It might be a little bit more difficult though.

<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img5.jpg" data-fancybox="images" data-caption="old pin crimps">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img5-small.png" alt="old pin crimps" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img6.jpg" data-fancybox="images" data-caption="new pin crimps">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img6-small.png" alt="new pin crimps" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img7.jpg" data-fancybox="images" data-caption="new connector">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img7-small.jpg" alt="new connector" class="hover-shadow"/>
    </a>
</div>
</div>

<div class="clearfix">
<div class="img-container w2">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img8.jpg" data-fancybox="images" data-caption="color code">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img8-small.jpg" alt="color code" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w2">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img9.jpg" data-fancybox="images" data-caption="new connector in ROSbot">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img9-small.jpg" alt="new connector in ROSbot" class="hover-shadow"/>
    </a>
</div>
</div>

It is a good practice to color code your connectors, in this case to mark the connector polarity. I used a label printer to label an "up" of the connector, but a pice of color tape should be enough. 

<strong>2. Prepare led strips.</strong>

I recommend to place your led installation in "chamfers" under front and rear panel of ROSbot's body. This way illumination will be clearly visible but it won't be irritating for eyes. In each "chamfer" we can fit 2 strips, 4 pixels each. You should prepare 4 pieces:

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img10.jpg" data-fancybox="images" data-caption="led strips">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img10-small.jpg" alt="led strips" class="hover-shadow" />
</a>

<strong>3. Install double sided tape.</strong>

It is not necessary since the strips usually come with its own adhesive.

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img11.jpg" data-fancybox="images" data-caption="double sided tape installation">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img11-small.jpg" alt="double sided tape installation" class="hover-shadow" />
</a>

<strong>4. Solder the strips together.</strong>

Here is the connection diagram:

<a href="/docs/assets/img/mbed-tutorials/ws2812b-connection-diagram.png" data-fancybox="images" data-caption="double sided tape installation">
    <img src="/docs/assets/img/mbed-tutorials/ws2812b-connection-diagram.png" alt="double sided tape installation" class="hover-shadow" width="640"/>
</a>

Measure the distance between each strip to prepare enough cable and install the heat shrinking sleeves before soldering. Remember to connect the DOUT port of one strip with the DIN port of an another strip, otherwise they won't work. After you have the whole installation prepared you can attach it to the ROSbot using provided adhesive or the double sided tape.

<div class="clearfix">
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img12.jpg" data-fancybox="images" data-caption="old pin crimps">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img12-small.jpg" alt="old pin crimps" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img14.jpg" data-fancybox="images" data-caption="new pin crimps">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img14-small.jpg" alt="new pin crimps" class="hover-shadow"/>
    </a>
</div>
<div class="img-container w3">
    <a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img13.jpg" data-fancybox="images" data-caption="new connector">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img13-small.jpg" alt="new connector" class="hover-shadow"/>
    </a>
</div>
</div>

## Using the new functionality

### Required ROS packages

Before we start make sure you have the `rosbot` package installed on your device. The package contains the EKF and custom messages used by the new firmware. It is required for the new firmware to work correctly. The package also contains example nodes used further in this tutorial.

The package is located [HERE](https://github.com/byq77/rosbot). Clone the package to your ROSbot's `ros_ws/src` directory.

Following dependencies are also required. On your device please run:

```bash
$ sudo apt-get install ros-kinetic-robot-localization
```

Now you can compile the `rosbot` package. In your `ros_ws` directory run `catkin_make`.

### Using `rosbot` package

To start the rosserial communication and EKF run:
```bash
$ roslaunch rosbot all.launch
```

For PRO version add parameter:

```bash
$ roslaunch rosbot all.launch rosbot_pro:=true
```

You can also include this launch in your custom launch files using:

```xml
<include file="$(find rosbot)/launch/all.launch"/>
```

For PRO version it will look like that:

```xml
<include file="$(find rosbot)/launch/all.launch">
    <arg name="rosbot_pro" value="true"/>
</include>
```
### WS2812B animations interface in ROS

The new firmware provides a service server `/config` with the custom message type `rosbot/Configuration`. 

```bash
$ rossrv show rosbot/Configuration 
string command
string data
---
uint8 SUCCESS=0
uint8 FAILURE=1
uint8 COMMAND_NOT_FOUND=2
string data
uint8 result
```

The WS2812B animations are accessible via `SANI` command. For most commands there are two arguments. First is the animation letter and second is the color of animation. Here is an example for setting a red fading animation:  

```bash
$ rosservice call /config "command: `SANI`
>data: 'F #aa0000'"
```

Available commands:
* `O` - OFF
* `S <hex color code>` - SOLID COLOR 
* `F <hex color code>` - FADE IN FADE OUT ANIMATION
* `B <hex color code>` - BLINK FRONT/REAR ANIMATION
* `R` - RAINBOW ANIMATION

### `cmd_vel` example

<table class="text_table">
<tbody>
    <tr>
	    <th>Behavior</th>
        <th>Description</th>
        <th>GoalStatus ID</th>
        <th>Command</th>
    </tr>
</tbody>
</table>

### `move_base` example

<table class="text_table">
<tbody>
    <tr>
	    <th>Behavior</th>
        <th>Description</th>
        <th>GoalStatus ID</th>
        <th>Command</th>
    </tr>
    <tr>
        <td align="center">No animation</td>
        <td>WS2812B interface is off</td>
        <td></td>
        <td><code>SANI -1</code></td>
    </tr>
    <tr>
        <td align="center">No animation</td>
        <td>WS2812B interface is in standby</td>
        <td align="center"><code>PENDING</code></td>
        <td><code>SANI 0</code></td>
    </tr>
    <tr>
        <td><img src="/docs/assets/img/mbed-tutorials/anim_blue.gif" width="200" /></td>
        <td>fading in fading out animation (blue)</td>
        <td align="center"><code>ACTIVE</code></td>
        <td><code>SANI 1</code></td>
    </tr>
    <tr>
        <td><img src="/docs/assets/img/mbed-tutorials/anim_yellow.gif" width="200" /></td>
        <td>fading in fading out animation (yellow)</td>
        <td align="center"><code>PREEMPTED</code></td>
        <td><code>SANI 2</code></td>
    </tr>
    <tr>
        <td><img src="/docs/assets/img/mbed-tutorials/anim_green.gif" width="200" /></td>
        <td>fading in fading out animation (green)</td>
        <td align="center"><code>SUCCEEDED</code></td>
        <td><code>SANI 3</code></td>
    </tr>
    <tr>
        <td><img src="/docs/assets/img/mbed-tutorials/anim_red.gif" width="200" /></td>
        <td>front and rear blinks alternately (red)</td>
        <td align="center"><code>ABORTED</code></td>
        <td><code>SANI 4</code></td>
    </tr>
</tbody>
</table>


<a data-fancybox href="#feature-sample">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-still-frame.png" width="480px" class="hover-shadow"/>
</a>

### Creating your own animations

<video width="640" height="320" controls id="feature-sample" style="display:none;">
    <source src="/docs/assets/video/mbed-tutorials/feature_sample.webm" type="video/webm">
    Your browser doesn't support HTML5 video tag.
</video>

## Summary
