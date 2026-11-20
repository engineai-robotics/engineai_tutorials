# Livox ROS Driver 2

简体中文 | [English](README.md)

Livox ROS Driver 2 是用于连接 Livox 雷达产品的第二代驱动包，适用于 ROS（推荐 Noetic）和 ROS2（推荐 Foxy 或 Humble）。

  **说明：**

  Livox ROS Driver 是面向测试场景的调试工具。用于批量生产时，请根据实际需求在原始源码基础上进行优化。

## 1. 准备环境

### 1.1 操作系统要求

  * Ubuntu 18.04：ROS Melodic；
  * Ubuntu 20.04：ROS Noetic 和 ROS2 Foxy；
  * Ubuntu 22.04：ROS2 Humble；
  * Ubuntu 24.04：ROS 2 Jazzy。

  **提示：**

  Colcon 是 ROS2 使用的构建工具。

  安装方法见 [Colcon 安装说明](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)。

### 1.2 安装 ROS 或 ROS2

安装 ROS Melodic：
[ROS Melodic 安装说明](https://wiki.ros.org/melodic/Installation)

安装 ROS Noetic：
[ROS Noetic 安装说明](https://wiki.ros.org/noetic/Installation)

安装 ROS2 Foxy：
[ROS Foxy 安装说明](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

安装 ROS2 Humble：
[ROS Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

安装 ROS2 Jazzy：
[ROS Jazzy 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)


推荐选择 Desktop-Full 安装。

## 2. 构建与运行 Livox ROS Driver 2

### 2.1 获取 Livox ROS Driver 2 源码

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **说明：**

  请将源码克隆到上例中的 `[work_space]/src/` 目录，以符合构建工具的目录要求。

### 2.2 构建与安装 Livox-SDK2

  **说明：**

  按 [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md) 中的安装步骤操作。

### 2.3 构建 Livox ROS Driver 2

#### ROS（以 Noetic 为例）
```shell
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

#### ROS2 Foxy
```shell
source /opt/ros/foxy/setup.sh
./build.sh ROS2
```

#### ROS2 Humble
```shell
source /opt/ros/humble/setup.sh
./build.sh humble
```

#### ROS2 Jazzy

```shell
source /opt/ros/jazzy/setup.sh
./build.sh jazzy
```

### 2.4 运行 Livox ROS Driver 2

#### ROS

```shell
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```

其中：

* **livox_ros_driver2**：Livox ROS Driver 2 的 ROS 包名；
* **[launch file]**：要使用的 ROS 启动文件，`launch_ROS1` 目录中提供了多个示例。

以 HAP 雷达的 RViz 启动文件为例：

```shell
roslaunch livox_ros_driver2 rviz_HAP.launch
```

#### ROS2
```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

其中：

* **[launch file]**：要使用的 ROS2 启动文件，`launch_ROS2` 目录中提供了多个示例。

以 HAP 雷达的 RViz 启动文件为例：

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. 启动文件与 livox_ros_driver2 内部参数

### 3.1 启动文件

ROS 启动文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS1`，ROS2 启动文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS2`。各文件使用不同的参数配置，适用于以下场景：

| 启动文件 | 说明 |
| ------------------------- | ------------------------------------------------------------ |
| rviz_HAP.launch | 连接 HAP 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_HAP.launch | 连接 HAP 雷达，发布 Livox 自定义格式点云。 |
| rviz_MID360.launch | 连接 MID360 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_MID360.launch | 连接 MID360 雷达，发布 Livox 自定义格式点云。 |
| rviz_MID360s.launch | 连接 MID360s 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_MID360s.launch | 连接 MID360s 雷达，发布 Livox 自定义格式点云。 |
| rviz_AVIA2.launch | 连接 Avia2 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_AVIA2.launch | 连接 Avia2 雷达，发布 Livox 自定义格式点云。 |
| rviz_MID360l.launch | 连接 MID360l 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_MID360l.launch | 连接 MID360l 雷达，发布 Livox 自定义格式点云。 |
| rviz_mixed.launch | 连接 HAP 和 MID360 雷达，发布 pointcloud2 格式点云，自动打开 RViz。 |
| msg_mixed.launch | 连接 HAP 和 MID360 雷达，发布 Livox 自定义格式点云。 |

### 3.2 主要内部参数

Livox_ros_driver2 的内部参数在启动文件中设置。三个常用参数如下：

| 参数 | 说明 | 默认值 |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | 点云发布频率，浮点数。推荐值为 5.0、10.0、20.0、50.0 等，最高为 100.0 Hz。 | 10.0 |
| multi_topic | 点云话题分配：0 为所有雷达共用一个话题；1 为每台雷达使用独立话题。 | 0 |
| xfer_format | 点云格式：0 为 Livox pointcloud2（PointXYZRTLT）；1 为 Livox 自定义点云；2 为 PCL 标准 pointcloud2（pcl::PointXYZI，仅 ROS 支持）。 | 0 |

  **说明：**

修改表中未列出的参数前，请先了解对应参数的作用。

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 点云数据格式：***

1. Livox pointcloud2（PointXYZRTLT）格式：

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **说明：**

  各帧的点数可能不同，每个点都带有时间戳。

2. Livox 自定义数据包格式：

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;上述数据包中的自定义点云（CustomPoint）格式：

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. PCL 标准 pointcloud2（pcl::PointXYZI）格式，仅 ROS 支持发布：

&ensp;&ensp;&ensp;&ensp;数据结构见 PCL 库 `point_types.hpp` 文件中的 `pcl::PointXYZI`。

## 4. 雷达配置

雷达 IP、端口、数据类型等通过 JSON 配置文件设置。单台 HAP、Mid360、Mid360s、Avia2 及多型号混合使用的配置位于 `config` 目录。启动文件中的 `user_config_path` 参数指定 JSON 文件路径。

1. HAP 雷达配置示例（`config/HAP_config.json`）：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

上述 JSON 文件中的参数说明如下：

**雷达配置参数**
| 参数 | 类型 | 说明 | 默认值 |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| ip | String | 要配置的雷达 IP 地址 | 192.168.1.100 |
| pcl_data_type | Int | 发送的点云数据格式：1 为笛卡尔坐标（32 位）；2 为笛卡尔坐标（16 位）；3 为球坐标。 | 1 |
| pattern_mode | Int | 空间扫描模式：0 为非重复扫描；1 为重复扫描；2 为低扫描频率的重复扫描。 | 0 |
| blind_spot_set（仅 HAP 雷达） | Int | 盲区距离，范围为 50～200 cm | 50 |
| extrinsic_parameter | | 外参：roll、pitch、yaw 为浮点数；x、y、z 为整数。 | |

HAP 配置的更多说明见：
[HAP 配置文件说明](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. Avia2 雷达配置示例（`config/AVIA2_config.json`）。Mid360 和 Mid360s 使用相同的端口号及配置结构，设备键分别为 `MID360`、`Mid360s`，对应文件为 `config/MID360_config.json`、`config/MID360s_config.json`：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "Avia2": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "host_ip"        : "192.168.1.5",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.12",
      "pcl_data_type" : 1,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

3. 连接多台雷达时，在 `lidar_configs` 数组中分别添加各雷达的配置对象。多型号混合配置示例：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
4. 主机通过多个网卡连接多台雷达时，在 `lidar_configs` 数组中添加各雷达的配置对象，并分别运行对应的启动文件。以下为混合配置示例：

**MID360_config1:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**MID360_config2:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**启动文件 1：**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> # Mid360 MID360_config1 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>
```
**启动文件 2：**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> # Mid360 MID360_config2 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>

```

## 5. 支持的雷达型号

* HAP
* Mid360
* Mid360s
* Avia2
* Mid360L
* （后续将支持更多型号）

## 6. 常见问题

### 6.1 使用 livox_lidar_rviz_HAP.launch 启动后没有显示点云

在 RViz 的 Display 面板中，将 Global Options → Fixed Frame 设置为 `livox_frame`，并勾选 PointCloud2。

### 6.2 执行 ros2 launch livox_lidar_rviz_HAP_launch.py 时无法打开 liblivox_sdk_shared.so

将 `/usr/local/lib` 加入环境变量 `LD_LIBRARY_PATH`。

* 对当前终端生效：

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* 对当前用户生效：

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
