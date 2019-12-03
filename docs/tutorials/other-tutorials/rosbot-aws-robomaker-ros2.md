---
title: ROSbot + AWS Robomaker - ROS2 Example Applications
sidebar_label: ROSbot + AWS Robomaker - ROS2 Example Applications
id: rosbot-aws-robomaker-example-applications
---

# ROSbot - introduction

ROSbot 2.0 is autonomous, open source robot platform. It can be used as a learning platform for Robot Operating System as well as a base for a variety of robotic applications such as research robots, inspection robots, custom service robots etc.

## Unboxing

What's in the box:

- carrying case
- ROSbot 2.0 (with optional 3D camera and LiDAR already assembled)
- Wi-Fi 2.4GHz antenna
- 3x 18650 Li-Ion rechargeable batteries
- universal charger with power adapter
- charging cable
- microSD card with the software for ROSbot
- USB to Ethernet adapter

<div><center><img src="https://raw.githubusercontent.com/husarion/static_docs/master/src/assets/img/howToStart/ROSbot_unboxing.jpg"/></center></div>

## Rear panel description

In the picture below you can see names of the elements from the rear panel of the ROSbot.

![image](/docs/assets/img/aws-tutorials/quick-start/ROSbot2_rear_panel.png)

## Hardware setup

### 1. Mounting the batteries

Your ROSbot is assembled, but to get it ready to work, you need to provide a power supply and attach the antenna.

To mount the batteries turn ROSbot upside down and follow these steps:

1. Unscrew battery cover mounted with two screws.
2. Remove the battery cover.
3. Place batteries **accordingly to the polarization symbols (do it carefully!)**, keeping the black strip under the batteries.
4. Place battery cover and mount it with screws.

![image](/docs/assets/img/aws-tutorials/quick-start/rosbot_battery.png)

### 2. Batteries charging guide

1. Connect the power adapter to the charger and the output cable between charger and ROSbot (2 connectors on charger side, 1 black connector to ROSbot charging port).

![image](/docs/assets/img/aws-tutorials/quick-start/charger_1.png)
![image](/docs/assets/img/aws-tutorials/quick-start/charger_2.png)

After this step is complete your charger should look like this:

![image](/docs/assets/img/aws-tutorials/quick-start/rosbot_charger.jpg)

2. Use the first two buttons to select “LiPo BATT” mode and press [Start] button.
3. Use arrows to select “LiPo CHARGE” mode.
4. Press [Start] - the current value should start blinking. Use arrows to set the current to 1.5A.
5. Press [Start] again - the voltage value should start blinking. Select “11.1V(3S)” using arrows.
6. Press and hold [Start] for 2 seconds. The charger should now ask for confirmation. Press [Start] again. The charging process should begin now.
7. When charging is finished (after about 3 hours), the charger will generate a loud “beep” sound and will finish charging at the same time.

The picture below is a visualization of the mentioned steps.

![image](/docs/assets/img/aws-tutorials/quick-start/charging.png)

### 3. Attaching the antenna

To attach the antenna, screw it to the antenna connector on the ROSbot rear panel.

![image](/docs/assets/img/aws-tutorials/quick-start/rosbot_antenna.png)


## Flashing image with ROS2 support

ROSbots are shipped with ROS1 support. In order to enable working with ROS2 support, it is necessary to flash image with ROS2.

1. Extract SD card from ROSbot, by pushing card carefully until it is released back by card holder, then pull it out. You can find SD card slot on ROSbot right side.
 ![SD card side view](/docs/assets/img/ROSbot_manual/sd_card_side_view.png) 
2. Download image for Tinkerboard from [here](https://husarion-robomaker-downloads.s3-eu-west-1.amazonaws.com/ros2-2019-12-03.tar.gz) .
3. Extract downloaded image (For this process we recommend using [7zip](https://www.7-zip.org/))
4. Flash the extracted image onto SD card (For this process we recommend using [Etcher](https://www.balena.io/etcher/) but any image writing tool will be good):
 - If you want to replace the included card, remember that you need to use at least 16 GB capacity and 10 speed class micro SD card. 
 - Download [Etcher](https://www.balena.io/etcher/) and install it.
 - Connect an SD card reader with the SD card inside.
 - Open Etcher and select from your hard drive .img file that you extracted.
 - Select the SD card you wish to write your image to.
 - Review your selections and click *Flash!* to begin writing data to the SD card.
5. Insert SD card back to ROSbot

## Connecting to Wi-Fi network

ROSbot is basically a computer running Ubuntu, so let's configure it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Use networking menu located on top-right of the screen to connect to a Wi-Fi network.
4. When connection is active, use networking menu again and choose **Connection Information** to find device IP address.
5. Note the ROSbot IP address, you will need it later.

## Configure AWS Environment

Before we use AWS RoboMaker to build and deploy the tutorial applications, we must first set up the AWS environment. To simplify the configuration, we will use AWS CloudFormation. CloudFormation enables us to use a template file to define the configuration of our environment. We will use CloudFormation to create a bucket in Amaazon S3, as well as to create the necessary permissions in AWS Identity and Access Manager (IAM) that AWS RoboMaker requires to simulate and deploy our robot applications.

To deploy the template, sign in to the [CloudFormation console](https://console.aws.amazon.com/cloudformation/). Following the following steps to deploy the template:

1.  Download the template file from [here](https://raw.githubusercontent.com/husarion/rosbot-robomaker/master/rosbot_tutorial_template.yaml).
2.  Click the **Create Stack** button.
3.  Under _Choose a template_, choose _Upload a template to Amazon S3_ and click **Choose File**.
4.  Browse to the rosbot_tutorial_template.yaml file you download in Step 1 above.
5.  Click **Next**.
6.  On the next screen, provide a _Stack name_. This should be something descriptive such as "ROSbot-setup".
7.  In the _S3BucketName_ field, provide a globally-unique name for the S3 bucket that will be created. This S3 bucket will be used to store your robot application bundles, as well as any logs that your robot may generate during simulation. Use a name unique to you, such as "&lt;user_id&gt;-rosbot-tutorial". Replace "&lt;user-id&gt;" with a unique string.
8.  Choose **Next**.
9.  On the Options page, leave all defaults and choose **Next**.
10. On the Review page, click the checkbox to acknowledge that CloudFormation will create IAM resources on your behalf.
11. Click **Create**.

After a few brief minutes, the stack will be created. When the status has changed to CREATE_COMPLETE, choose the stack you just created, and view its Outputs. You will see 3 key/value pairs. You will use these values later in this guide.

## ROSbot setup in RoboMaker

ROSbot need some system modifications before Greengrass will be able to run and deploy applications. To configure ROSbot:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Robots**.

![RoboMaker robots](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_1.png)

- Choose \*_Create robot_.
- In the **Name** field, type `ROSbot`.
- From the **Architecture** dropdown menu choose **ARMHF**.
- From the **AWS Greengrass group** dropdown menu choose **Create new**.
- In the **AWS Greengrass prefix** field type `ROSbot`.
- In the **IAM role** select **ROSbot-deployment-role**.

![RoboMaker create robot](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_2.png)

- Proceed with **Create**, you will be redirected to **Download your Core device** page.

![RoboMaker robot created](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_3.png)

- Choose **Download** button next to **Download and store your Core's security resources**, you will get `ROSbot-setup.zip` file.

- File needs to be uploaded to ROSbot. The upload process will vary, depending on your host operating system.

### On Linux

Navigate to directory where the file is downloaded, by default it should be `~/Downloads`.

```
cd ~/Downloads
```

- Copy file to your ROSbot, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
scp ROSbot-setup.zip husarion@ROSBOT_IP:ROSbot-setup.zip
```

### On Windows

You will need an SCP client, download and install [WinSCP](https://winscp.net/eng/download.php).

Start WinSCP, you will see the login dialog:

![WinSCP login](/docs/assets/img/aws-tutorials/quick-start/winscp1.png)

- From `File protocol` dropdown menu choose: `SFTP`.
- In `Host name` field provide rosbot IP address that you noted earlier, it is the value which we described as `ROSBOT_IP`.
- In `Port number` field provide `22`.
- In `User name` field provide `husarion`.
- In `Password` field provide `husarion`.

When all fields are filled up, click `Login` button to connect, you will see file manager view.

![WinSCP file manager](/docs/assets/img/aws-tutorials/quick-start/winscp2.png)

In the left tab navigate to directory where you downloaded the ROSbot-setup.zip file. In the right tab navigate to `/home/husarion` directory.

Drag and drop the `ROSbot-setup.zip` to the right tab.

When the transfer is finished, close the window.

## ROSbot setup on device

You will need to make some system configurations on device. Depending on your host system, you can connect to your ROSbot with different methods:

### On Linux

Open terminal and start `ssh` connection, you will need to substitute `ROSBOT_IP` with device address you noted earlier:

```
ssh husarion@ROSBOT_IP
```

Proceed to **Device setup** section.

### On Windows

Press `WinKey` + `r` then type `mstsc`.

You will see a window appear:

![Windows RDP](/docs/assets/img/aws-tutorials/quick-start/win_rdp.png)

Type in your device IP address and click connect.

You will see the ROSbot desktop, from the top menu, choose the `Applications` -> `Terminal`.

## Device setup

- Unzip ROSbot security resources:

```
cd ~
sudo unzip ROSbot-setup.zip -d /greengrass
```

- Download and flash low level firmware:
```
wget https://husarion-robomaker-downloads.s3-eu-west-1.amazonaws.com/firmware.bin
sudo stm32loader -c tinker -e -v -w firmware.bin
```

- Restart ROSbot to apply chnages.

At this moment, ROSbot is ready to start AWS GreenGrass and accept incoming deployments.

- Connect with ROSbot again, open terminal and start the GreenGrass:

```
sudo /greengrass/ggc/core/greengrassd start
```

- Start MicroXRCEAgent for communication with CORE2 controller board:

```
sudo MicroXRCEAgent serial --dev /dev/ttyS1 -b 500000
```

- Reset the controller board with reset button on ROSbot back panel.

- Leave the ROSbot turned on, it will wait for deployment.

## Creating a RoboMaker IDE

Application will be built using the RoboMaker environment. To create the IDE:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/home)

![RoboMaker new IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_4.png)

- On the left, expand **Development**, choose **Development environments**, and then choose **Create environment**.
- In the Create AWS RoboMaker development environment page, enter `rosbot_env` as the environment name.
- Accept the default Instance type (`m4.large`). You can select different instances type to improve bundling performance.
- In **VPC** dropdown list choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.

![RoboMaker create IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_5.png)

- Choose **Create** to create the AWS Cloud9 development environment.

![RoboMaker IDE ready](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_6.png)

## Running RoboMaker sample applications on ROSbot

Below section will guide you through running [sample applications](https://docs.aws.amazon.com/robomaker/latest/dg/sample-applications.html) provided by AWS RoboMaker.

### Running the Hello World sample application

We are basing on [Hello World saple application](https://github.com/aws-robotics/aws-robomaker-sample-application-helloworld) with some modifications for running on ROSbot.

To deploy application, you will use RoboMaker environment created in previous step:

- Go to AWS RoboMaker home [console](https://console.aws.amazon.com/robomaker/home).

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`.

- Open the development environment with **Open environment** button.

![RoboMaker open IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_8.png)

- In the IDE, go to bash tab and clone the "Hello World" sample application repository in `~/environment/` directory:

```
cd ~/environment/
git clone https://github.com/lukaszmitka/aws-robomaker-sample-application-helloworld.git
```

- Start the configuration script. You need to provide the S3 bucket name and the ARN of the IAM role that was created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/aws-robomaker-sample-application-helloworld/rosbot_deploy/
./start_deployment_job.bash <S3BucketName> <ROSbotDeploymentRole>
```

The script will install all dependencies, configure project, build and set the deployment job.

When the script is done with its job, you can observe the deployment process:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Deployments**.
- When new deployment will appear, open it by clicking its name.
- Wait until deployment status changes to **Succeed** - ROSbot will start to spin in place.


### Running the Cloud Watch sample application

We are basing on [Cloud Watch sample application](https://github.com/aws-robotics/aws-robomaker-sample-application-cloudwatch) with some modifications for running on ROSbot.

Before you start with application deployment, first you need to set access permissions for ROSbot to enable it with Cloud Watch metrics uploads.

Open [IAM console](https://console.aws.amazon.com/iam/home) and in the left panel choose **Access management** -> **Users**, then choose button **Add user**.

![IAM add user step 1](/docs/assets/img/aws-tutorials/ros2/cloudwatch_1.png)

In **User name** field provide 'ROSbotLoggerUser', mark **Programmatic access** checkbox and proceed with button **Next: Premissions**

![IAM add user step 2](/docs/assets/img/aws-tutorials/ros2/cloudwatch_2.png)

Choose **Attach existing policies directly** and check policies:
- `AWSRoboMakerFullAccess`
- `CloudWatchFullAccess`
- `AWSGreengrassFullAccess`

You may use filter to find required policies, proceed with **Next: Tags** and **Next: Review**. Verify that you have `AWS access type` set to `Programmatic access - with an access key` and in **Permissions summary** table you have three policies.

![IAM add user step 3](/docs/assets/img/aws-tutorials/ros2/cloudwatch_4.png)

Proceed with **Create user** button.

![IAM add user step 4](/docs/assets/img/aws-tutorials/ros2/cloudwatch_5.png)

In confirmation window, you receive **Access key ID** and hidden **Secret access key**, click `Show` to view it. Note both keys, they will be required in next step. You can close dialog window with **Close** button.

Connect with ROSbot and open terminal, you will create AWS access configuration for ROSbot.

- Create directory for storing configuration and credentials:
    ```
    sudo -u ggc_user mkdir /home/ggc_user/.aws
    ```

- Create configuration file:
    ```
    sudo -H -u ggc_user nano config 
    ```

    In file paste following content, adjust value of `region` depending on your zone.

    ```
    [default]
    region=********
    ```
- Create credentials file:
    ```
    sudo -H -u ggc_user nano credentials
    ```

    In file paste following content, adjust values accordingly to keys that you created in previous step:
    ```
    [default]
    aws_access_key_id=************
    aws_secret_access_key=*****************
    region=********
    ```

ROSbot is ready for publishing Cloud Watch metrics, start `Greengrass` daemon and `MicroXRCEAgent` if they are not running already.

To deploy application, you will use RoboMaker environment created in previous step:

- Go to AWS RoboMaker home [console](https://console.aws.amazon.com/robomaker/home).

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`.

- Open the development environment with **Open environment** button.

![RoboMaker open IDE](/docs/assets/img/aws-tutorials/quick-start/aws_tutorial_robomaker_8.png)

- In the IDE, go to bash tab and clone the Cloud Watch sample application repository in `~/environment/` directory:

```
cd ~/environment/
git clone https://github.com/lukaszmitka/aws-robomaker-sample-application-cloudwatch.git
```

- Start the configuration script. You need to provide the S3 bucket name and the ARN of the IAM role that was created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/aws-robomaker-sample-application-helloworld/rosbot_deploy/
./start_deployment_job.bash <S3BucketName> <ROSbotDeploymentRole>
```

The script will install all dependencies, configure project, build and set the deployment job.

When the script is done with its job, you can observe the deployment process:

- Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
- In the left navigation pane, choose **Fleet Management** and then choose **Deployments**.
- When new deployment will appear, open it by clicking its name.
- Wait until deployment status changes to **Succeed** - ROSbot will publish metrics into Cloud Watch.

To view metrics, go to [Cloud Watch console](https://console.aws.amazon.com/cloudwatch/home), then choose **Metrics** in left panel. Choose **AWSRoboMakerFleetManagement** -> **RobotName, category, robot_id**. You can mark chosen statistics to plot them on a graph, you can inspect memory usage, cpu usage or system uptime.

---

_by Łukasz Mitka, Husarion_

_Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com_
