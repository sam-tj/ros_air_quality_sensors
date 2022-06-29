# ROS Air quality sensor

This is an example project which allows to send custom message from an various sensors mentioned below to ros2
The code can be modified according to application.

------------

[![Sample topic echo](https://raw.githubusercontent.com/sam-tj/ros_air_quality_sensors/main/sample_topic_echo.png)](#)

------------

### List of components used:
- [ESP 32](https://www.az-delivery.de/en/products/esp32-developmentboard "ESP 32")
- [mh-z19b CO2 Sensor](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf "mh-z19b CO2 Sensor") 
- [MQ-135 gas sensor](https://www.az-delivery.de/en/products/mq-135-gas-sensor-modul "MQ-135 gas sensor") 

------------

### About
pending

------------

### Connections
|  ESP32  |  mhz_19b Sensor  |  MQ2 Sensor  |
|  ------------ |  ------------ |  ------------ |
|  G12  |  RX  |  -  |
|  G13  |  TX  |  -  |
|  G32  |  -  |  AO  |
|  5V  |  VCC  |  VCC  |
|  GND  |  GND  |  GND  |

------------

### Message output
|  ESP32  |  Message type  |
|  ------------ |  ------------ |
|  time  |  string  |
|  counter  |  int16  |
|  co2  |  int16  |
|  temperature  |  int16  |
|  r0value  |  int16  |
|  co2_mq135  |  float32  |
|  co  |  float32  |
|  alcohol  |  float32  |
|  ammonium  |  float32  |
|  toulene  |  float32  |
|  acetone  |  float32  |

------------

### NTP setup for host system.

------------

1. sudo apt-get update
2. sudo apt-get install ntp
3. sntp --version
4. sudo nano /etc/ntp.conf OR 
        sudo gedit /etc/ntp.conf

Enter nearest ntp server address. To find servers, can refer this link -> https://support.ntp.org/bin/view/Servers/NTPPoolServers

replace server list from -> 
> pool 0.ubuntu.pool.ntp.org iburst 
> 
> pool 1.ubuntu.pool.ntp.org iburst
> 
> pool 2.ubuntu.pool.ntp.org iburst
> 
> pool 3.ubuntu.pool.ntp.org iburst

to required servers. Use iburst to make specific server high priority. 
example -> 
> server 0.europe.pool.ntp.org
> 
> server 1.europe.pool.ntp.org

5. sudo service ntp restart
6. sudo service ntp status

Now there should be an ip address of local network. Should be in series of 192.168.x.x or 10.x.x.x

7. sudo ufw allow from any to any port 123 proto udp

------------

### NTP setup for ESP32 system.

------------

1. Update the IP address in the code in place of NTP server.
2. Update the timezone in the code according to the required location. ( https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv )

------------

### Tutorial
1. Follow the  [tutorial here](https://link.medium.com/pdmyDUIh9nb "tutorial here") to setup the micro-ros environment.
2. Clone this repository and copy the folder 'firmware' inside your Micro-Ros workspace such that folder structure is as  [YOUR Micro-Ros WORKSPACE]/firmware/freertos_apps/apps/ros_air_quality_sensors and [YOUR Micro-Ros WORKSPACE]/firmware/mcu_ws/custom_message
3. Open terminal and enter following commands inside your [Micro-Ros workspace]:
   - ros2 run micro_ros_setup configure_firmware.sh ros_air_quality_sensors -t udp -i [LOCAL MACHINE IP ADDRESS] -p 8888
   - ros2 run micro_ros_setup build_firmware.sh
   - ros2 run micro_ros_setup flash_firmware.sh
4. Once the flashing is successful, run
   - ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
5. Copy src from ROS2_WORKSPACE_DIR folder and paste it inside your ROS2 Workspace such that folder structure is as [YOUR ROS2 WORKSPACE]/src/custom_message
6. Open new terminal window, and reach [YOUR ROS2 WORKSPACE] 
7. Enter following commands inside your ROS2 workspace:
   - colcon build
   - source install/setup.bash
   - ros2 topic echo /sensors

------------

Available time attributes in time struct.
- int tm_sec        ->      Seconds.	[0-60] (1 leap second)
- int tm_min        ->      Minutes.	[0-59]
- int tm_hour       ->      Hours.	[0-23]
- int tm_mday       ->      Day.		[1-31]
- int tm_mon        ->      Month.	[0-11]
- int tm_year       ->      Year	- 1900.
- int tm_wday       ->      Day of week.	[0-6]
- int tm_yday       ->      Days in year.[0-365]
- int tm_isdst      ->      DST.		[-1/0/1]

### References
Library
- http://sandboxelectronics.com/?p=165
- https://github.com/miguel5612/MQSensorsLib
- https://github.com/UncleRus/esp-idf-lib

Tutorial
- https://vitux.com/how-to-install-ntp-server-and-client-on-ubuntu/
- https://www.theconstructsim.com/ros2-tutorials-7-how-to-create-a-ros2-custom-message-new/
- https://programozdazotthonod.hu/2020/05/03/a-proper-guide-to-the-mq9-sensor/
  
Sensor research
- https://seeeddoc.github.io/How_to_choose_A_Gas_Sensor/
- https://learn.adafruit.com/gas-sensor-comparison/connect-the-sensor-boards
- 
