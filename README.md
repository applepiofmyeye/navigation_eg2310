# r2auto_nav

Turtlebot navigation code for EG2310, sem2 AY22/23.

Refer to our group's documentation [here](https://github.com/applepiofmyeye/navigation_eg2310/blob/main/docs/Documentation.pdf)

## Components you'll need for this project:

- Turtlebot3 burger
- Microswitch
- IR Sensor module
- ESP32 module
- Servo Motor
- OLED Screen
- Push Button x 3

## Preparation

### Setting up environment

1. Firstly, follow the instructions provided in this [link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup), specifically the "Foxy" tab, starting from Section 3.1.1, to install Ubuntu 20.04 and ROS 2 Foxy on your laptop.

2. Verify the functionality of the ROS development environment by creating a basic working publisher and subscriber, following the instructions provided in this [link](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

3. Afterwards, proceed to flash the ROS2 Foxy image to the SD card on the TurtleBot3's RPi, utilising the Ubuntu Operating System (OS), as per the instructions provided in this link.

4. Repeat step 2 to ensure that the ROS development environment on the RPi is also functioning as intended.

5. Execute the publisher on the RPi, run the subscriber on your laptop, and confirm that they are communicating without any errors. Then, swap the devices to validate that they function effectively as both publishers and subscribers.
   Message Queuing Telemetry Transport Setup

6. On the RPi, install the ‘paho-mqtt’ package, the Eclipse Paho MQTT Python client library, using either one of the following methods by entering the following into the command-line interface (CLI)

```
apt install
sudo apt install paho-mqtt
pip install
pip install paho-mqtt
```

6. On the laptop, in the Ubuntu OS, install ‘mosquitto’, a broker for the MQTT protocol version 5.0/3.1.1/3.1 using the following command.

```
sudo snap install
```

7. Copy the mosquitto_example.conf file into a new file named mosquitto.conf.

```
cd /var/snap/mosquitto/common
sudo cp mosquitto_example.conf mosquitto.conf
```

8. Edit the mosquitto.conf file by adding the following lines at the end of the file.

```
allow_anonymous true
listener 1883
```

9. Stop and start the mosquitto service.

```
sudo systemctl stop snap.mosquitto.mosquitto.service
sudo systemctl start snap.mosquitto.mosquitto.service
```

10. Check that the mosquitto broker is running

```
sudo systemctl status snap.mosquitto.mosquitto
```

### Installing the program

1. Before cloning the program from the GitHub repository, we need to create a ROS2 package on our laptop with the following commands.

```
cd ~/colcon_ws/src
ros2 pkg create --build-type ament_python auto_nav
cd auto_nav/auto_nav
```

2. We will then move the `init.py` file from the current directory temporarily to the parent directory, using the below command

```
mv init.py ..
```

3. Now, we can clone the repository onto our laptop

```
git clone git@github.com:MoeySeanJean/r2auto_nav.git .
```

4. Then, we can move our `init`.py file back into the current working directory

```
mv ../init.py .
```

5. Add the following lines to the `.bashrc` file on the laptop

```
export TURTLEBOT3_MODEL=burger
alias rteleop='ros2 run turtlebot3_teleop teleop_keyboard'
alias rslam='ros2 launch turtlebot3_cartographer cartographer.launch.py’
```

6. Now, we will move on to install the code for the RPi. `ssh` into the RPi, using the following command, where the Xs that follow ‘ubuntu@’ is the IP address of the RPi.

```
ssh ubuntu@XXX.XX.XX.X
```

6. Repeat steps 1 to 4, but with the following highlighted changes. The series of commands to be entered should be as follows.

```
cd ~/turtlebot3_ws/src
ros2 pkg create --build-type ament_python auto_nav
cd navigation_eg2310/navigation_eg2310
mv init.py ..
git clone git@github.com:applepiofmyeye/navigation_eg2310.git .
mv ../init.py .
```

### ESP32 Setup

1. (Using Arduino Integrated Development Environment (IDE), on Windows OS) In your Arduino IDE, go to File> Preferences, and enter the following into the “Additional Board Manager URLs” field:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then click the “OK” button.

2. Open the Boards Manager. Go to Tools > Board > Boards Manager…

3. Search for ESP32 and press install button for the “ESP32 by Espressif Systems“:

4. Plug the ESP32 board to the laptop. With your Arduino IDE open, select your Board in Tools > Board menu (for our case, we chose the DOIT ESP32 DEVKIT V1).

5. Select the port connected to the ESP32, under Tools > Port, an

6. From this link, copy the file titled “ESP.ino” and create a new Arduino sketch with this code.

7. Press the Upload button on the Arduino IDE and wait for the code to compile.

8. After it compiles, the IDE should display “Connecting…”. Press and hold the Boot button on the ESP32 until the upload begins.

9. If everything went as expected, the IDE should display a “Done uploading” message.

10. If the ESP32 has been connected to the OLED screen as mentioned in Section 4.4.4 of our [documentation](http://www.github.com/applepiofmyeye/navigation_eg2310/docs/Documentation.pdf), it should show the displays as per mentioned.
