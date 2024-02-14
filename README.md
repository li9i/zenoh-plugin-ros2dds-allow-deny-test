# Setup ROS 2 workspace
```
mkdir -p ~/m2m/src
```



# Clone `zenoh-plugin-ros2dds` into it

```
cd ~/m2m/src
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git
cd zenoh-plugin-ros2dds
git reset --hard fdf9a1af4f3
```



# Clone this repo into the workspace as well

```
cd ~/m2m/src
git clone https://github.com/**************.git
```



# Build everything

```
cd ~/m2m
colcon build --packages-select m2m_zenoh_test_minimal_subscriber m2m_zenoh_test_minimal_publisher zenoh_bridge_ros2dds
```



# Run

We need four terminals/Terminator panes



## Setup terminals

Do the following on all four:

```
cd m2m_ws
source install/local_setup.bash
```



## Setup zenoh bridge for subscriber

```
ROS_DOMAIN_ID=1
zenoh_bridge_ros2dds -c ~/m2m_ws/src/m2m_comms_test/configuration_files/zenoh_config_sub.json5
```

## Setup zenoh bridge for publisher
```
ROS_DOMAIN_ID=2
zenoh_bridge_ros2dds -c ~/m2m_ws/src/m2m_comms_test/configuration_files/zenoh_config_pub.json5
```

## Setup subscriber
```
ROS_DOMAIN_ID=1
ros2 run m2m_zenoh_test_minimal_subscriber subscriber_member_function
```

## Setup publisher
```
ROS_DOMAIN_ID=2
ros2 run m2m_zenoh_test_minimal_publisher publisher_member_function
```

