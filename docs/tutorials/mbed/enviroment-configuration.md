---
title: Enviroment configuration
sidebar_label: 1. Enviroment configuration
id: 1-enviroment-configuration
---

## Using CORE2 with Mbed OS

**Mbed OS** is free, open-source platform and embedded operating system designed for IoT devices based on Arm Cortex-M family of microcontrollers. It is developed as collaborative project by *Arm*, its partners and growing community of individual devs from across the world. Mbed OS is distributed under the [Apache-2.0 License](https://en.wikipedia.org/wiki/Apache_License) and it's available on project's [GitHub page](https://github.com/ARMmbed/mbed-os).


Besides support for variety of boards from different manufacturers the framework has features like:
* built-in support for connectivity options like *Bluetooth LE*, *Wi-Fi*, *Ethernet*, *Cellular*, *LoRa LPWAN*, *NFC* and others,
* RTOS core based on open-source *CMSIS-RTOS RTX*,
* *Hardware Enforced Security* and *Communications Security*,
* easy and portable API.

<div>
<center>
<img src="/img/mbed-tutorials/mbed_logo.png" width="400px" alt="Mbed OS logo"/>
</center></div>

* [Official Webpage](https://www.mbed.com/en/platform/mbed-os/)
* [Mbed OS documentation](https://os.mbed.com/docs/v5.10/)
* [Mbed OS Doxygen API](https://os.mbed.com/docs/v5.10/mbed-os-api-doxy/modules.html)

## First Steps

In this tutorial we will show you how to build, compile and run mbed applications on **CORE2** using mbed offline tools. You will be introduced to basics of mbed API, learn how to use rosserial library to connect your mbed application with SBC and more. Let's hack!

![](https://cdn.shopify.com/s/files/1/2545/8446/products/CORE2-ROS_1024x1024@2x.png?v=1520001976)

### Prerequisites

#### Hardware

* CORE2-ROS (with SBC) or CORE2 and computer running Linux with ROS Kinetic or Melodic. Note that ROSbot 2.0 contain CORE2-ROS inside.
* USB cable if you want to flash code straight from your PC or laptop.     
#### Software prerequisites:

Before we start make sure you have following tools installed on your system:

* [Visual Studio Code IDE](https://code.visualstudio.com/)
* [GNU Arm Embedded version 6 toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* [Mbed CLI](https://os.mbed.com/docs/mbed-os/v5.12/tools/installation-and-setup.html)

#### Required Visual Studio Code extensions:
* [Microsoft C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (`ms-vscode.cpptools`)
* [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) (`marus25.cortex-debug`)

Everything up and ready? Proceed to the next steps then.

### Mbed CLI installation

`mbed-cli` is a package name of **Arm Mbed CLI**, a command-line tool that enables use of Mbed build system, GIT/Mercurial-based version control, dependencies management and more. Check [Mbed CLI GitHub page](https://github.com/ARMmbed/mbed-cli) or [Mbed documentation](https://os.mbed.com/docs/mbed-os/v5.15/tools/developing-mbed-cli.html) for details about the tool.  

To install `mbed-cli` follow [this](https://os.mbed.com/docs/mbed-os/v5.14/tools/installation-and-setup.html) tutorial from the Mbed documentation. 

Installers for both Windows and macOS are provided. Linux users have to install tool manually. In case you are user of the latter system check if you have both Git and Mercurial installed before you start. See [Instructions for Linux](https://os.mbed.com/docs/mbed-os/v5.14/tools/manual-installation.html) page for more details.

Check if the installation was successful by running following command in the terminal:

```bash
mbed --version
1.10.1
```

After installation you have to inform Mbed CLI about location of compiler (in our case GCC Arm Embedded Compiler) binaries. We will use global setting. Run:

```bash
mbed config -G GCC_ARM_PATH <path to the compiler>
```

Linux example:

```bash
mbed config -G GCC_ARM_PATH /home/szysza/opt/gcc-arm-none-eabi-6-2017-q2-update/bin
```

Windows example:

```bash
where arm-none-eabi-gcc # prints path to arm-none-eabi-gcc.exe if in PATH
mbed config -G GCC_ARM_PATH "C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update\bin"
```

You can check current configuration by running:

```bash
mbed config --list
```

### Preparing a workspace

Create a new folder `core2-mbed-workspace`. It will serve as a workspace for your mbed projects. Run:

```bash
mkdir core2-mbed-workspace && cd core2-mbed-workspace
```

Next step is to import `mbed-os` library. It will be used by all your projects. In your workspace's folder run:

```bash
mbed import mbed-os
[mbed] Working path "E:\mbed_projects\core2-mbed-workspace" (directory)
[mbed] Program path "E:\mbed_projects\core2-mbed-workspace"
[mbed] Importing program "mbed-os" from "https://github.com/ARMmbed/mbed-os" at latest revision in the current branch
```

Set Mbed OS version to supported by this template:

```bash
cd mbed-os
mbed update mbed-os-5.14.1
```

During Mbed OS installation you can be asked to install additional python libraries. Switch to mbed-os dir and run:
```bash
pip install -r requirements.txt --user
```

Mbed CLI needs to know the path to `mbed-os` directory. This way all your projects can use one instance of library (default configuration is to have separate instance of library for each project). Run:

```bash
mbed config -G MBED_OS_DIR <path to mbed-os>
```

Example:

```bash
mbed config -G MBED_OS_DIR "E:\mbed_projects\core2-mbed-workspace\mbed-os"
[mbed] Working path "E:\mbed_projects\core2-mbed-workspace" (directory)
[mbed] Program path "E:\mbed_projects\core2-mbed-workspace"
[mbed] E:\mbed_projects\core2-mbed-workspace\mbed-os now set as global MBED_OS_DIR
```

#### Adding a `.mbedignore` file

In order to add support for CORE2 target and speed-up building of your projects we will exclude certain folders of `mbed-os` library from compilation. For this purpose Mbed build system provides `.mbedignore` files. They have similar structure to `.gitignore` files used by GIT.

In your local `mbed-os` library directory create a new file and name it `.mbedignore`. Open it and add following lines:

```
features/cellular/*
features/cryptocell/*
features/deprecated_warnings/*
features/lorawan/*
features/lwipstack/*
features/nanostack/*
features/netsocket/*
features/nfc/*
features/unsupported/*
components/wifi/*
components/cellular/*
components/802.15.4_RF/*
components/TARGET_PSA/*
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/STM32F407XG.ld
targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F407xG/device/TOOLCHAIN_GCC_ARM/startup_stm32f407xx.S
```

## Template Project

We will start by setting up a template project. You can use it as starting point for all of your mbed applications. 

Just download the zip : https://github.com/husarion/core2-mbed-template/archive/master.zip and extract it in your workspace.

On Linux:

```bash
wget https://github.com/husarion/core2-mbed-template/archive/master.zip && unzip master.zip
```

You can also clone the repository using GIT:

```bash
git clone https://github.com/husarion/core2-mbed-template.git
```

Open the template project in Visual Studio Code. In file `setting.json` from the directory `.vscode` in your template, change the value of `C_cpp.default.compilerPath` to match location of `arm-none-eabi-gcc` on your system:

Windows:
```json
{
"C_Cpp.default.compilerPath": "C:/Program Files (x86)/GNU Tools ARM Embedded/6 2017-q2-update/bin/arm-none-eabi-g++"
}
```

Linux:
```json
{
"C_Cpp.default.compilerPath": "/usr/bin/arm-none-eabi-g++"
}
```

> The paths may differ on your system.

This will enable more accurate IntelliSense and remove some error notifications.

### Template description

Open the template project's directory and select `src/main.cpp`. You should see:

![](/img/mbed-tutorials/mbed-tutorial1-img1.png)

The sample code is very simple. It instantiates a `BusOut` object that controls multiple GPIOs regardless of ports they belong to. On-board LEDs blink in order described by `leds_mask` array at the interval introduced by function `ThisThread::sleep_for(1000)`.    

In directory's root folder find `custom_targets.json` file:

![](/img/mbed-tutorials/mbed-tutorial1-img2.png)

Mbed OS Configuration system uses this file to describe user's custom boards. It allows using Mbed OS with boards that aren't officially supported. We use `custom_targets.json` and files from `TARGET_CORE2` to define `CORE2` target. You can learn more about configuration system [here](https://os.mbed.com/docs/v5.13/reference/configuration.html).

In the directory `TARGET_CORE2` you can find files `PinNames.h` and `PeripheralPins.c`. First one defines pin names of mcu and the latter defines peripherals that can be used on each pin.

Another file that is used by Mbed OS configuration system is `mbed_app.json`. Open it.

![](/img/mbed-tutorials/mbed-tutorial1-img3.png)

This file is used to configure your application. It allows to override the default configuration of mbed libraries (or your own) for specific targets (tag `"target_overrides"`). You can also define your own macros that will have global visibility (tag `"macros"`) and create configuration entries (tag `"config"`).

You can also chosee here format of output file. If you want to use stm32loader and program your CORE2 straight from SBC you should leave:

```
"target.OUTPUT_EXT": "bin"
```
But if you want to use core2_flasher and USB cable you should change it to:

```
"target.OUTPUT_EXT": "hex"
```

You can learn details of your configuration by running following command in the root directory of the template project:

```bash
mbed compile --config --source . --source ../mbed-os/ -v
```

The last file we will check is `task.json` from `.vscode` directory. It defines tasks that are recognized by Visual Studio Code IDE. The tasks can be accessed by pressing `CTRL + SHIFT + P` and typing `Task: Run Task` in Command Pallete.

![](/img/mbed-tutorials/mbed-tutorial1-img4.png)

### Building and flashing firmware

> **Important!**
If it's your first time with Mbed OS on CORE2 and you have been using Husarion Cloud until now you will need to do one more thing before proceeding. The Husarion Cloud used small program that resided in flash memory before main application called bootloader. This memory area is write protected so you have to use `core2-flasher` tool to unprotect it:
`./core2-flasher --unprotect` 

Press `CTRL + SHIFT + B`. It will run `BUILD (RELEASE)` task. Wait until compilation finishes.

![](/img/mbed-tutorials/mbed-tutorial1-img5.png)

Here we have two option of flashing firmawre to CORE2.

#### Flashing using .hex file and core2_flasher 

This method was explained in tutorial [CORE2 - quick start](/tutorials/howtostart/core2-quick-start/#the-flashing) so if you don't know it yet please check this tutorial.

![](/img/howToStart/core2_hSerial.png)

> **Note!**
The hSerial port on rear panel of ROSbot is the same port as hSerial on CORE2 visible in the illustration. 

#### Flashing using .bin file and STM32loader

This method is only for CORE2 with SBC connected to hRPi connector (CORE2-ROS or ROSbot's). You don't have to use any additional cables because it's using UART from hRPi connector. 

Open console of your SBC. Go to the directory including your .bin file. Next type:

```
sudo stm32loader -c <your_sbc> -e -w -v firmware.bin
```
You have to replace `<your_sbc>` with `rpi`, `tinker` or `upboard`.

Full documentation of STM32loader you can find in our [Software](/software/stm32loader/) section.

#### Results

If LEDs start blinking like on the animation below then congratulations! You've just successfully built and flashed your first Mbed application for CORE2!

![](/img/mbed-tutorials/mbed-tutorial-animation.gif)

### Tasks
- Modify existing application so as the on-board leds blink in Gray code. 
- Add [Serial](https://os.mbed.com/docs/v5.10/apis/serial.html) to print current sequence to the stdout (micro-usb port on CORE2) at the same time. 
- In configuration files change baudrate to 9600. 
- Check target's files and learn which UART instance is connected to pins `USBTX` and `USBRX`.    

## Rosserial library

If you made it this far you must be really into this stuff! Let's do something more interesting and learn how to communicate with devices running ROS using mbed. For this purpose we will use [rosserial for mbed platforms](http://wiki.ros.org/rosserial_mbed). 

<div>
<center><img src="/img/mbed-tutorials/ros_logo.png" width="400px" alt="result"/></center>
</div> 

### Example publisher

First we will create a new project directory. Just simply duplicate template project and name it `example-publisher`:

```bash
cp -rf core2-mbed-template example-publisher
```

Another way is to create empty mbed project using Mbed CLI. Just type:
```bash
mbed new <project-name> --program
```
After that copy content of template project except `.git` directory to your newly created program directory.

> **Tip**
>
> If you cloned your template project from online repository and you don't want to have version control in it just delete `.git` directory:
> ```bash
> rm -rf ./core2-mbed-template/.git/
> ```
> You can add it latter by running `git init .` in root dir of your project.   

Open `example-publisher` directory in Visual Studio Code. In the program press `CTRL + ~` to open built-in terminal. In the terminal type:

```bash
cd lib 
mbed add https://github.com/husarion/rosserial-mbed
```

This will add `rosserial-mbed` library to your project and download all library's dependencies. Managing libraries this way is simple:
* `mbed add <library-url>` - adds library to project,
* `mbed remove <library-name>` - removes library from project.

![](/img/mbed-tutorials/mbed-tutorial1-img7.png)

#### The code

Take a minute to analyze the program below. We provided you with comments to make it easier. 

```cpp
/*
 * main.cpp
 */
#include <mbed.h>
#include <Thread.h>
#define ARRAY_SIZE 4

// This header file must be included before any other 
// ros header files.
#include <ros.h> 
#include <std_msgs/String.h>

// DigitalOut objects for controlling on-board leds.
DigitalOut led1(LED1,0), led2(LED2,1), ros_led(LED3,0);

// Thread object for controlling ros task. Feature of RTOS.
Thread ros_thread;

// Ticker object for controlling reoccurring interrupt 
// that calls attached callback at given rate.  
Ticker blinker;

// Node handle instantiation - required for registering publishers, subscribers
// and handling serial communication.
ros::NodeHandle  nh;

static const char * messages_array[ARRAY_SIZE] = {
    "mbed V5.10", "CORE2", "MCU", "STM32F407ZG", 
};

// ros task
void rosThreadCallback(void)
{
    std_msgs::String str_msg;
        
    // Instantiate Publisher object with topic name "mbed device".
    // Second parameter is reference to instance of object that
    // will be used in communication.
    ros::Publisher mbed_device("mbed_device", &str_msg);

    // Initialize node and advertise our custom topic. 
    nh.initNode();
    nh.advertise(mbed_device);

    int i=0;

    while(1)
    {
        ros_led = !ros_led;
        str_msg.data = messages_array[i%ARRAY_SIZE];

        // publish message to topic
        mbed_device.publish(&str_msg);
        i++;

        // process all messages
        nh.spinOnce();

        ThisThread::sleep_for(1000);
    }
}

void blinkerCallback(void)
{
    led1 = !led1;
    led2 = !led2;
}

int main()
{
    // registering a ros task
    ros_thread.start(callback(rosThreadCallback));

    // We attach callback to blink leds every 2s
    blinker.attach(callback(blinkerCallback),2.0);
}
```

As you can see the program is pretty straightforward. It creates topic "device_mbed" and publishes messages to it blinking board's leds at the same time. 

If you would like to know more about mbed specific code check [Mbed API](https://os.mbed.com/docs/v5.10/apis/index.html) :
* [Thread](https://os.mbed.com/docs/v5.10/apis/thread.html),
* [Ticker](https://os.mbed.com/docs/v5.10/apis/ticker.html),
* [DigitalOut](https://os.mbed.com/docs/v5.10/apis/digitalout.html).

#### Running the code

To get the application working we need to configure library's serial pins and baudrate. You can check default values in `rosserial-lib/mbed_lib.json`. If you use CORE2 with SBC connected to RPI connector, add this lines to your `mbed_app.json` file under `target_overrides.CORE2` object:

```json
"rosserial-mbed.tx_pin": "RPI_SERIAL_TX",
"rosserial-mbed.rx_pin": "RPI_SERIAL_RX",
"rosserial-mbed.baudrate": "115200"
```

Your `mbed_app.json` file should look like this:
```json
{
    "config": {},
    "macros": [
        "ENCODER_1=TIM2",
        "ENCODER_2=TIM8",
        "ENCODER_3=TIM3",
        "ENCODER_4=TIM4",
        "UPPER_RESISTOR=5.6e4",
        "LOWER_RESISTOR=1.0e4",
        "VIN_MEAS_CORRECTION=0.986"
    ],
    "target_overrides": {
        "CORE2": {
            "events.shared-dispatch-from-application": 0,
            "events.shared-eventsize": 512,
            "events.shared-stacksize": 2048,
            "platform.default-serial-baud-rate": 230400,
            "platform.stdio-baud-rate": 230400,
            "platform.all-stats-enabled": false,
            "rosserial-mbed.tx_pin": "RPI_SERIAL_TX",
            "rosserial-mbed.rx_pin": "RPI_SERIAL_RX",
            "rosserial-mbed.baudrate": "115200"
        }
    }
}
```
Now you can compile the project and flash it to your board. To view communication on your SBC you must disable Husarion Cloud.

If you haven't already disabled `husarnet-configurator` service please run:
```bash
systemctl disable husarnet-configurator
sudo shutdown -r now # it will reboot your SBC
```

After reboot open terminal and in first tab run `roscore`. Press `CTRL + SHIFT + T` and in second tab run:

* Raspberry PI
```bash
rosrun rosserial_python serial_node.py _port:=/dev/serial0 _baud:=115200
```

* Asus Tinker Board
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyS1 _baud:=115200
```

* Upboard
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyS4 _baud:=115200
```

This will forward your MBED messages to rest of ROS. 

To view communication on "mbed_device" topic open new termina and run:
```bash
rostopic echo mbed_device
```

![](/img/mbed-tutorials/mbed-tutorial1-img8.png)

### Example subscriber

In this project we'll use both publisher and subscriber as well as some cryptographic functionality of [mbed TLS library](https://tls.mbed.org/) . Mbed TLS library include crypto and SSL/TLS capabilities with minimal footprint and easy to use API. It is available as part of Mbed OS.

Like in previous example create a new project and name it `subscriber-example`.

#### The code

The example creates two topics - "raw_input" for user to send short String messages to be encrypted by [AES-ECB](https://en.wikipedia.org/wiki/Advanced_Encryption_Standard) block cipher and "encrypted_output" for user to collect encrypted messages.

```cpp
/*
 * main.cpp
 */
#include <mbed.h>
#include <Thread.h>
#include <ros.h> 
#include <std_msgs/String.h>

// header file for AES
#include <mbedtls/aes.h>
#define BLOCK_SIZE 16

// LED1 = PE_2, 
// LED2 = PE_3, 
// LED3 = PE_4,

// Port masks
enum {
    NONE = 0,
    L1 = 0b00000100,
    L2 = 0b00001000,
    L3 = 0b00010000,
    L1L2 = 0b00001100,
    ALL = 0b00011100
};

uint8_t masks[] = {NONE,L1,L1L2,ALL};

// Controls port E
PortOut leds(PortE,ALL);

Thread ros_thread;
volatile bool message_ready = false;

// variables required for AES encryption ECB mode
mbedtls_aes_context aes;
const uint8_t secret_key[BLOCK_SIZE+1] = "YMZE4oIxB9M14bkF"; // 128-bit key
uint8_t input[BLOCK_SIZE];
uint8_t output[BLOCK_SIZE];
char formatted_output[2*BLOCK_SIZE+1];

void subscriberCallback(const std_msgs::String &raw)
{
    // check if length <= 16
    int n = strlen(raw.data);
    if (n == 0 || n > 16)
        return;
    memcpy(input, raw.data, n);
    // zero padding
    for (int i = n; i < BLOCK_SIZE; ++i)
        input[i] = 0;

    // encrypt message
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input, output);
    message_ready = true;
}

void rosThreadCallback(void)
{
    ros::NodeHandle nh;

    // the output will be formated string of characters
    std_msgs::String str_output;
    ros::Publisher pub("output_encrypted", &str_output);

    // We instantiate publisher object with "input_raw" topic and attach
    // callback for subscriber event (when user sends something).
    ros::Subscriber<std_msgs::String> sub("input_raw", subscriberCallback);

    nh.initNode();
    nh.advertise(pub);

    // subscribe to topic
    nh.subscribe(sub);

    // set aes key
    mbedtls_aes_setkey_enc(&aes, secret_key, 128);
    while (1)
    {
        // if message was encrypted send result to topic
        if (message_ready)
        {
            int j = 0;
            // format data
            for (int i = 0; i < BLOCK_SIZE; ++i)
            {
                sprintf(formatted_output + j, "%02X", *(output + i));
                j += 2;
            }
            str_output.data = formatted_output;
            pub.publish(&str_output);
            message_ready = false;
        }
        nh.spinOnce();
        ThisThread::sleep_for(50);
    }
}

int main()
{
    // registering a ros task
    ros_thread.start(callback(rosThreadCallback));
    int i = 0, n = sizeof(masks);
    while(1)
    {
        leds = masks[i%n];
        i++;
        ThisThread::sleep_for(500);
    }
}
```

More about API used:
* [PortOut](https://os.mbed.com/docs/v5.10/apis/portout.html),
* [mbedtls_aes_crypt_ecb()](https://tls.mbed.org/api/aes_8h.html#a0e59fdda18a145e702984268b9ab291a).

#### Running the code

On your SBC open a terminal and in separate tabs start `roscore` and `rosrun rosserial_python serial_node.py` bridge with the same parameters like in the previous example. 

To receive encrypted messages, in a new tab run:
```bash
rostopic echo output_encrypted
```

To publish new message to "input_raw" topic open a new tab and run:

```bash
rostopic pub input_raw std_msgs/String "Hello World!" --once
```

![](/img/mbed-tutorials/mbed-tutorial1-img9.png)

If you want to learn more - check official [rosserial mbed tutorials](http://wiki.ros.org/rosserial_mbed/Tutorials) from **ros.org**. 

### Tasks
* Create an application that monitors the on-board button and publish the number of pushes to topic "button" every time the button's state changes. Use [InterruptIn](https://os.mbed.com/docs/v5.10/apis/interruptin.html) object. 
* Create an application that lights up on-board leds accordingly to the mask value received on "led_mask" topic. Use `std_msgs::Uint8` type for ROS communication.

## Summary

After completing this tutorial you should know the basics of Mbed OS 
components and tools. You should be able to create, compile and run mbed 
applications on CORE2 and use Rosserial library to incorporate your mbed platform into ROS projects.

---------

*by Szymon Szantula, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*

