---
title: stm32loader
id: stm32loader
---
## About

STM32loader is Python script used to upload or download firmware to / from STM32 microcontrollers over UART. This software is installed on all of husarion image in default. 

## STM32loader installation

Please log into your SBC and follow this step by step tutorial on how to install and configure this tool.

<strong>1.</strong> Disable `husarnet-configurator` and `husarion-shield services` and reboot your device. These processes are responsible for connection to the Husarion Cloud and they also control GPIO pins that are used for uploading the firmware. We will need the direct access to them. Run:

```bash
sudo systemctl disable husarnet-configurator
sudo systemctl stop husarnet-configurator
sudo systemctl disable husarion-shield
sudo reboot
```

<strong>2.</strong> Install necessary support libraries for your device:

**RPi:**
```
pip install RPi.GPIO
```

**Upboard:**
```bash
cd ~/ && git clone https://github.com/vsergeev/python-periphery.git
cd ~/python-periphery && git checkout v1.1.2
sudo python setup.py install --record files.txt
```
**Asus Tinker board:**
```bash
cd ~/ && git clone https://github.com/TinkerBoard/gpio_lib_python.git
cd ~/gpio_lib_python && sudo python setup.py install --record files.txt
```

Restart the terminal after the installation.

<strong>3.</strong> Install `stm32loader`:
```bash
cd ~/ && git clone https://github.com/husarion/stm32loader.git
cd ~/stm32loader && sudo python setup.py install --record files.txt
```

## stm32loader usage

Printing help:
```bash
stm32loader --help
```

To remove bootloader run:
```bash
sudo stm32loader -c <your-sbc> -W
sudo stm32loader -c <your-sbc> -e
```

where `<your-sbc>` is:
* `tinker` for Asus Tinker Board
* `upboard` for Upboard
* `rpi` for Raspberry Pi

To upload the `firmware.bin` directly from SBC copy the firmware from your template project's `BUILD/RELEASE/` directory to SBC using `scp`. Following code will copy `firmware.bin` file to remote user's home directory.

```bash
scp firmware.bin user@address:~/
```

To flash new firmware log into your SBC and in the directory which contain the firmware run:

```bash 
sudo stm32loader -c <your_sbc> -e -w -v firmware.bin
```
