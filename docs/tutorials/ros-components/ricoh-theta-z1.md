---
title: Setting up RICOH THETA Z1
sidebar_label: 1. Setting up RICOH THETA Z1
id: ricoh-theta-z1
---

## Installation and building dependencies
This tutorial is based on [official tutorial by RICOH](https://codetricity.github.io/theta-linux/)


### [v4l2loopback](https://github.com/umlaeute/v4l2loopback)
First download, build and install v4l2loopback. It will allow you to create virtual loopback camera interfaces.
``` bash
mkdir -p ~/husarion_ws/src/thera_z1
cd ~/husarion_ws/src/thera_z1/
git clone https://github.com/umlaeute/v4l2loopback.git
cd v4l2loopback/
make && sudo make install
sudo depmod -a
```
After successful installation run:
```
ls /dev | grep video
```
You should see your video interfaces.

If you don't have any other cameras installed the output should be empty.
To start loopback interface and find it's ID run:
``` bash
sudo modprobe v4l2loopback
ls /dev | grep video
```
New `/dev/video` device should appear. It's your loopback interface you will later assign to your THETA Z1.

### Ricoh Theta dependencies
Install required packages:
``` bash
sudo apt-get install libgstreamer1.0-0 \
     gstreamer1.0-plugins-base \
     gstreamer1.0-plugins-good \
     gstreamer1.0-plugins-bad \
     gstreamer1.0-plugins-ugly \
     gstreamer1.0-libav \
     gstreamer1.0-doc \
     gstreamer1.0-tools \
     gstreamer1.0-x \
     gstreamer1.0-alsa \
     gstreamer1.0-gl \
     gstreamer1.0-gtk3 \
     gstreamer1.0-qt5 \
     gstreamer1.0-pulseaudio \
     libgstreamer-plugins-base1.0-dev \
     libjpeg-dev

```
After installation building and install *libuvc-theta*:
```
cd ~/husarion_ws/src/thera_z1
git clone https://github.com/ricohapi/libuvc-theta.git
cd libuvc-theta
mkdir build
cd build
cmake ..
make
sudo make install
```

Now you have to download THETA Z1 specyfic dependencies and configure them to match your platform.
``` bash
cd ~/husarion_ws/src/thera_z1
git clone https://github.com/ricohapi/libuvc-theta-sample.git
cd libuvc-theta-sample/gst
```

In file *gst_viewer.c* in line 190 replace:
``` C
"v4l2sink device=/dev/video1 sync=false";
```
With:
``` C
"v4l2sink device=/dev/video0 quos=false sync=false";
```
If your device has more than one camera you have to replace `/dev/video0` with matching interface number. You can check it by running:
``` bash
ls /dev | grep video
```
Most likely loopback interface will be the last one.

After you make changes in file you can now build the code by running:
``` bash
make
```
**Caution!** Configuration of *gst_viewer.c* the file shown in this tutorial is for x86_64 bit architecture. Other architectures require different changes. For more information follow [this page](https://codetricity.github.io/theta-linux/equipment/).


### Testing
Connect your camera via USB turn it on and select *LIVE* mode and run:
```
./gst_viewer
```

## Accessing camera from /dev/video interface
Every time you connect the camera you have to set it to *LIVE* mode, start loopback interface and assign camera to loopback interface.

Starting and assigning loopback can be done with those commands:
``` bash
sudo modprobe v4l2loopback
cd ~/husarion_ws/src/thera_z1/libuvc-theta-sample/gst
./gst_loopback
```

Now you will be able to acces your THETA Z1 with any software.

### [cv_camera](http://wiki.ros.org/cv_camera) and rostopics

Download and build *cv_camera*
``` bash
cd ~/husarion_ws/src/
git clone https://github.com/OTL/cv_camera.git
cd ..
catkin_make --only-pkg-with-deps cv_camera
```



In order to test it run:
``` bash
# Terminal 1
roscore
```

``` bash
# Terminal 2
# device_id is /dev/video number
rosparam set cv_camera/device_id 0
rosrun cv_camera cv_camera_node
```

```
# Terminal 3
rosrun image_view image_view image:=/cv_camera/image_raw
```

This will open window with preview from camera. Window might be too wide to fit in on your screen so probably you will see only part of camera's field of view.

### OpenCV face detection
As an example we will use [this](https://github.com/adarsh1021/facedetection) repository.
``` bash
cd ~/husarion_ws/src
git clone https://github.com/adarsh1021/facedetection.git
cd facedetection
python3 detect_face_video.py
```
If this script won't run change line 7 to match `/dev/video` interface number.

---

_by Krzysztof Wojciechowski, Husarion_

_Do you need any support with completing this project or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_

