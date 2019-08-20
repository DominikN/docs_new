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

* **WS2812b LED strip 1m 30leds/pixels/m**

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img2.jpg" data-fancybox="images" data-caption="WS2812b">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img2-small.jpg" alt="WS2812b" class="hover-shadow"/>
</a>

* **scissors**
* **electric wire stripper**
* **double side tape**
* **heat shrink sleeves** to protect connections between led strips
* **2.54mm (0.1") Pitch Female Connector 3 Position** - to swap with the default connector that comes with the strip

<a href="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img3.jpg" data-fancybox="images" data-caption="female connector 3 position">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-img3-small.jpg" alt="female connector 3 position" class="hover-shadow"/>
</a>

<!-- zbiorcze zdjęcie wszystkich elementów -->

### Software

This tutorial uses the new firmware for ROSbot, based on Mbed OS system. To learn more about the new firmware please get familiar with the firmware's [GitHub page](https://github.com/husarion/rosbot-firmware-new).

We recommend you to check our [Using CORE2 with Mbed OS tutorial](/docs/tutorials/mbed-tutorials/using-core2-with-mbed-os). It will introduce you to the Mbed OS environment and tools.

#### Building from source

To prepare the environment, build and upload the firmware please follow the instructions from the [README](https://github.com/husarion/rosbot-firmware-new/blob/master/README.md) file.

Before building the code, in your project find `mbed_app.json` file and enable the **ws2812b driver** by changing the line:

```json
"enable-ws2812b-signalization": 0
```

to:

```json
"enable-ws2812b-signalization": 1 
```

#### Ready to use firmware package

## Hardware installation

## WS2812B library for CORE2

## Enabling signalization support in ROSbot firmware

<a data-fancybox href="#feature-sample">
    <img src="/docs/assets/img/mbed-tutorials/mbed-tutorial2-still-frame.png" width="360px" class="hover-shadow"/>
</a>

<video width="640" height="320" controls id="feature-sample" style="display:none;">
    <source src="/docs/assets/video/mbed-tutorials/feature_sample.webm" type="video/webm">
    Your browser doesn't support HTML5 video tag.
</video>
