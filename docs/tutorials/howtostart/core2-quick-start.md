---
title: CORE2 - quick start
sidebar_label: 1. CORE2 - quick start
id: core2-quick-start
---

## Preparing hardware ##

Connect your CORE2 to a DC power supply. The power connector is a standard DC 5.5/2.1 (center-positive), and provides 6 to 16V output. You can use:

* DC adapter
* Li-poly/Li-ion packages - 2S or 3S (e.g. 18650 batteries)
* AA alkaline batteries (4-10 pieces)

<div><center><img src="/docs/assets/img/howToStart/core2_power_supply.png"
/></center></div>

Set the power switch to "ON" position and now your device is ready to use!

## First Steps

In this tutorial we will show you how to flash mbed applications on **CORE2** using Husarion tools. Let's hack!

#### Hardware

* CORE2 or CORE2-ROS (with SBC)
* USB A <-> USB micro cable

#### Software prerequisites:

Before we start make sure you already downloaded:

* [Husarion tools](https://files.husarion.com/husarion-tools/husarion-tools.zip)
* [Hex file with example compiled code](https://files.husarion.com/husarion-tools/example.hex)

We will explain how to use it in next steps of this tutorial 
## First flashing

#### The code

Take a minute to analyze the program below. We provided you with comments to make it easier. 

```cpp
#include <mbed.h>

static const uint8_t led_mask[] = {
0,0b00000001,0b00000010,0b00000100,0b00000010,
0b00000001,0,0b00000111,0b00000101,0b00000010};
BusOut leds(LED1,LED2,LED3);

int main()
{
    int k = sizeof(led_mask), n = 0;
    while(1)
    {
        leds = led_mask[n%k];
        n++;
        ThisThread::sleep_for(1000);
    }
}
```

The sample code is very simple. It instantiates a `BusOut` object that controls multiple GPIOs regardless of ports they belong to. On-board LEDs blink in order described by `leds_mask` array at the interval introduced by function `ThisThread::sleep_for(1000)`.    

#### The flashing

Before we start extract file husarion-tools.zip which contain few versions of core2-flasher script for different operating system and processor architecture. For example when you use Ubuntu x64 you have use amd64-linux version.

Now let's plug in micro USB cable to USB hSerial port and to your computer. Open console and run core2-flasher with our example.hex:

```
.<directory>/core2-flasher <directory>/example.hex
```
for example:
```
./Desktop/husarion-tools/amd64-linux/core2-flasher ~/Desktop/example.hex
```
Everything went OK, if you will see output similar to this:

<div>
<center><img src="/docs/assets/img/howToStart/output.png" alt="output"/></center>
</div> 

#### Checking the results

If LEDs start blinking like on the animation below then congratulations! You've just successfully flashed your first Mbed application for CORE2!

<div>
<center><img src="/docs/assets/img/mbed-tutorials/mbed-tutorial-animation.gif" alt="result"/></center>
</div> 

## Summary

After completing this tutorial you should know how to connect CORE2 to your computer and flash code. 

You can go now to next tutorial named [Configuration of environment for Mbed](https://husarion.com/core2/tutorials/mbed/1-enviroment-configuration) to find information how to write and compile your own code based on Mbed.

---------

*by Hubert Zwiercan and Szymon Szantula, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*

