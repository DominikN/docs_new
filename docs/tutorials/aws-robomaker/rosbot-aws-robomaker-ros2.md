---
title: ROSbot + AWS Robomaker - ROS2 Example Applications
sidebar_label: 4. ROSbot + AWS Robomaker - ROS2 Example Applications
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

![](/docs/assets/img/howToStart/ROSbot_unboxing.jpg)

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
 - Wait until the process is finished.
 - Use **Unmount drive** on linux on **Remove device safely** on Windows, wait until system confirms that you can unplug card reader.
5. Insert SD card back to ROSbot

### Troubleshooting

It may happen that process of flashing SD card will fail, this may prevent ROSbot from booting properly.
Possible symptoms of faulty flashing process are:
 - Blank screeen after powering up the ROSbot.
 - ROSbot begins to boot, but freeze at some moment.
 - ROSbot is booting into text-only mode.
 - ROSbot is booting into graphical mode but it is not responding.
If above signs are observed, it will be necassary to repeat card flasing process.

## Connecting to Wi-Fi network

ROSbot is basically a computer running Ubuntu, so let's configure it like a standard PC computer.

1. Plug in a display with HDMI, mouse and keyboard into USB port in the rear panel of ROSbot.
2. Turn on the robot and wait until it boots.
3. Use networking menu located on top-right of the screen to connect to a Wi-Fi network.
4. When connection is active, use networking menu again and choose **Connection Information** to find device IP address.
5. Note the ROSbot IP address, you will need it later.

## Choosing the location

You can chose to work with AWS services on servers located in varoius location across world. You are free to choose the server closest to your location, although in tutorials we refer to **us-east-1**. Whichever location you will choose, you must keep it while following through the whole manual.

To choose location, go to [AWS Management console](https://console.aws.amazon.com/console/) and find location dropdown menu next to your username in top right corner.
From the menu choose **US East (N. Virginia) us-east-1**.

![location](/docs/assets/img/aws-tutorials/ros2/Location.png)

For more information regarding regions please check the [documentation](https://docs.aws.amazon.com/robomaker/latest/dg/limits-regions.html).

## Configure AWS Environment

Before we use AWS RoboMaker to build and deploy the tutorial applications, we must first set up the AWS environment. To simplify the configuration, we will use AWS CloudFormation. CloudFormation enables us to use a template file to define the configuration of our environment. We will use CloudFormation to create a bucket in Amazon S3, as well as to create the necessary permissions in AWS Identity and Access Manager (IAM) that AWS RoboMaker requires to simulate and deploy our robot applications.

To deploy the template, sign in to the [CloudFormation console](https://console.aws.amazon.com/cloudformation/). Follow the steps to deploy the template:

1.  Click the **Create Stack** button and select **With new resources** from the drop down.

![CloudFormation](/docs/assets/img/aws-tutorials/ros2/CloudFormation-01.png)

2.  In **Step 1: Specify template** in section **Prerequisite - Prepare template** select **Template is ready**, then in section **Specify template** select **Amazon S3 URL** and type in the following text field: `https://robomaker-rosbot-samples.s3.amazonaws.com/rosbot_tutorial_template.yaml`. Proceed with **Next** button.

![CloudFormation](/docs/assets/img/aws-tutorials/ros2/CloudFormation-02.png)

3. In **Step 2: Specify stack details** in section **Stack name** type `rosbot-stack`, then proceed with **Next** button.

![CloudFormation](/docs/assets/img/aws-tutorials/ros2/CloudFormation-03.png)

4. In **Step 3: Configure stack options** leave all settings default and proceed with **Next** button.
5. In **Step 4: Review** scroll to last section named **Capabilities** then check **I acknowledge that AWS CloudFormation might create IAM resources with custom names.**. Proceed with **Create stack** button.
![CloudFormation](/docs/assets/img/aws-tutorials/ros2/CloudFormation-04.png)
6. You will be redirected to stacks list, wait until your stack is created.
7. When stack creation is finished, click stack name to open its details, then choose tab **Outputs**, in the table you will see three rows, note the three **Key**/**Value** pairs as you will need them later in this guide.
![CloudFormation](/docs/assets/img/aws-tutorials/ros2/CloudFormation-05.png)

## ROSbot setup in RoboMaker

ROSbot need some system modifications before Greengrass will be able to run and deploy applications. To configure ROSbot:

1. Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/).
2. In the left navigation pane, choose **Fleet Management** and then choose **Robots**.
3. Choose **Create robot**.
![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-01.png)
4. In section **General**:
- In the **Name** field, type `ROSbot`.
- From the **Architecture** dropdown menu choose **ARMHF**.
![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-02.png)
5. In section **AWS Greengrass group details**:
- From the **Greengrass setup method** dropdown menu choose **Use manual setup method**.
- From the **AWS Greengrass group** dropdown menu choose **Create new**.
- In the **AWS Greengrass prefix** field type `ROSbot`.
- From the **IAM role** dropdown menu choose **ROSbotDeploymentRole**.
![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-03.png)

6. Proceed with **Create**, you will be redirected to **Download your Core device** page.

![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-05.png)

7. Choose **Download** button next to **Download and store your Core's security resources**, you will get `ROSbot-setup.zip` file.

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

1. Sign in to the AWS RoboMaker [console](https://console.aws.amazon.com/robomaker/home)

![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-06.png)

2. On the left, expand **Development**, choose **Development environments**, and then choose **Create environment**.
3. In **General** section:
- In the **Name** field type `rosbot_env`.
- From the **Pre-installed ROS distribution** dropdown menu choose **ROS2 Dashing (Beta)**.
- From the **Instance type** dropdown menu choose **m4.large**. You can select different instance type to improve bundling performance.
4. In **Networking** section:
- From the **VPC** dropdown menu choose the default value.
- In the **Subnets** dropdown list choose the first subnet. You can select different subnet if necessary.
![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-07.png)

5. Choose **Create** to create the AWS Cloud9 development environment, you will be redirected to new browser tab with environment opened.

![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-09.png)


## Running RoboMaker sample applications on ROSbot

Below section will guide you through running [sample applications](https://docs.aws.amazon.com/robomaker/latest/dg/sample-applications.html) provided by AWS RoboMaker.

### Running the Hello World sample application

We are basing on [Hello World sample application](https://github.com/aws-robotics/aws-robomaker-sample-application-helloworld) with some modifications for running on ROSbot.

To deploy application, you will use RoboMaker environment created in previous step:

- Go to AWS RoboMaker home [console](https://console.aws.amazon.com/robomaker/home).

- On the left, expand **Development**, choose **Development environments**, and then choose `rosbot_env`.

- Open the development environment with **Open environment** button.

![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-08.png)


- In the IDE, go to bash tab and clone the "Hello World" sample application repository in `~/environment/` directory:

```
cd ~/environment/
git clone https://github.com/husarion/aws-robomaker-sample-application-helloworld.git
```

- Start the configuration script. You need to provide the S3 bucket name and the ARN of the IAM role that was created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/aws-robomaker-sample-application-helloworld/rosbot_deploy/
./start_deployment_job.bash <S3Bucket> <ROSbotDeploymentRole>
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

- Create a directory for storing configuration and credentials:
    ```
    sudo -u ggc_user mkdir /home/ggc_user/.aws
    ```

- Create a configuration file:
    ```
    sudo -H -u ggc_user nano config 
    ```

    In file paste following content, adjust value of `region` depending on your zone.

    ```
    [default]
    region=********
    ```
- Create a credentials file:
    ```
    sudo -H -u ggc_user nano credentials
    ```

    In the file paste the following content and adjust values according to the keys that you created in previous step:
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

![RoboMaker](/docs/assets/img/aws-tutorials/ros2/RoboMaker-08.png)

- In the IDE, go to bash tab and clone the Cloud Watch sample application repository in `~/environment/` directory:

```
cd ~/environment/
git clone https://github.com/husarion/aws-robomaker-sample-application-cloudwatch.git
```

- Start the configuration script. You need to provide the S3 bucket name and the ARN of the IAM role that was created by CloudFormation earlier. The parameters to the script should be set to the corresponding values provided in the output of your CloudFormation stack:

```
cd ~/environment/aws-robomaker-sample-application-helloworld/rosbot_deploy/
./start_deployment_job.bash <S3Bucket> <ROSbotDeploymentRole>
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
