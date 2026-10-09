# GNSS Sensor Broadcaster

A `ros2_control` controller that reads GNSS state from a hardware interface and publishes the data as standard ROS 2 messages, tested on  [Xsens MTi-680G](https://www.movella.com/products/sensor-modules/xsens-mti-680g-rtk-gnss-ins).

This broadcaster interprets a numerical `fix_type` from the state interface to publish `sensor_msgs/msg/NavSatFix` messages with dynamically adjusted status and position covariance, enabling support for RTK-corrected systems.

## Operation

The broadcaster operates within the `ros2_control` framework. The data flow is as follows:

1.  A `hardware_interface` component reads data from the physical GNSS sensor.
2.  The hardware interface exports this data as a set of `state interfaces`.
3.  The `GnssSensorBroadcaster`, when loaded by the `controller_manager`, reads the values from these state interfaces on each update cycle.
4.  The broadcaster populates and publishes standard ROS 2 messages to corresponding topics.

## State Interface Requirements

This controller requires a hardware interface to export the following state interfaces. The `<sensor_name>` prefix is a configurable parameter.

| Interface Name | Data Type | Description |
| :--- | :--- | :--- |
| `<sensor_name>/latitude` | `double` | Latitude in degrees. |
| `<sensor_name>/longitude` | `double` | Longitude in degrees. |
| `<sensor_name>/altitude` | `double` | Altitude in meters. |
| `<sensor_name>/velocity.north` | `double` | Velocity towards North in m/s. |
| `<sensor_name>/velocity.east` | `double` | Velocity towards East in m/s. |
| `<sensor_name>/velocity.down` | `double` | Velocity downwards in m/s. |
| `<sensor_name>/fix_type` | `double` | **Critical:** Numerical code for GNSS fix quality. |
| `<sensor_name>/satellites` | `double` | Number of satellites used in the position solution. |
| `<sensor_name>/hdop` | `double` | Horizontal Dilution of Precision. |

## Configuration Parameters

The following parameters must be defined in the controller configuration YAML file.

* `sensor_name` (string): The prefix used in the state interface names. This must match the name defined in the hardware interface.
* `frame_id` (string): The coordinate frame ID to be set in the header of all published messages.

### Example `controllers.yaml`
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50 # Hz

    gnss_sensor_broadcaster:
      type: gnss_sensor_broadcaster/GnssSensorBroadcaster

gnss_sensor_broadcaster:
  ros__parameters:
    sensor_name: "gnss_sensor"
    frame_id: "gnss_link"
```

## Published Topics

The broadcaster publishes to the following topics within its namespace (e.g., `/gnss_sensor_broadcaster`):

* `~/gnss` (`sensor_msgs/msg/NavSatFix`): Contains position, fix status, and covariance.
* `~/velocity` (`geometry_msgs/msg/TwistStamped`): Linear velocity of the sensor.
* `~/nmea` (`nmea_msgs/msg/Sentence`): A dynamically generated NMEA GGA sentence.

## Output Data Interpretation

The primary source for position and fix quality is the `~/gnss` topic.

### Fix Status Interpretation

The quality of the GNSS fix is encoded in the `status.status` field of the `sensor_msgs/msg/NavSatFix` message. This broadcaster maps the numerical `fix_type` from the hardware interface to the standard `NavSatStatus` constants as follows:

| `fix_type` (from Hardware) | `status.status` (in ROS Message) | Constant | Meaning |
| :--- | :--- | :--- | :--- |
| `0` | `-1` | `STATUS_NO_FIX` | No fix. Position data is unreliable. |
| `1` or `2` | `0` | `STATUS_FIX` | Standard 2D/3D unaugmented fix. |
| `5` | `1` | `STATUS_SBAS_FIX` | **RTK Float**. Decimeter-level precision. |
| `4` | `2` | `STATUS_GBAS_FIX` | **RTK Fixed**. Centimeter-level precision. |

To inspect this value in real-time, use the command:
```bash
ros2 topic echo /<broadcaster_namespace>/gnss
```

### Position Covariance Interpretation

The `position_covariance` field of the `NavSatFix` message is populated dynamically based on the `fix_type`. The diagonal elements `[0]`, `[4]`, and `[8]` represent the variance in the X (East), Y (North), and Z (Up) directions, respectively.

* **RTK Fixed (`fix_type`=4):** Variance is set to a very low value (e.g., `1.0e-4` m²), indicating high confidence.
* **RTK Float (`fix_type`=5):** Variance is set to a low value (e.g., `1.0e-2` m²).
* **Standard Fix (`fix_type`=1, 2):** Variance is estimated using the `hdop` value (`hdop`²).
* **No Fix (`fix_type`=0):** Variance is set to a large value to indicate low confidence.

This allows downstream nodes, such as sensor fusion algorithms, to correctly weight the GNSS data according to its real-time accuracy.

## Compile and testing

Go to your workspace and run:

``` bash
colcon build --packages-select gnss_sensor_broadcaster
```

You can test *gnss_sensor_broadcaster* using a ROS2 launch file. Also, certify that you have *controller_manager* from [ROS2 Control](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html) framework running.

**Example:**

``` python
# example.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gnss_sensor_broadcaster'),
        'gnss_sensor_broadcaster.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gnss_sensor_broadcaster', '-c', '/controller_manager'],
            output='screen'
        )
    ])
```

**Configuration Example:**

Make a config file in `yaml`, then define a *controller_manager* config and the broadcaster parameters to execute with *controller_manager*.

``` YAML
# config.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

gnss_sensor_broadcaster:
    type: gnss_sensor_broadcaster/GnssSensorBroadcaster

    ros__parameters:
    sensor_name: xsens_gnss # Example of a base name for state interfaces
    frame_id: gnss_link   # Example of a frame_id for publish msgs

    # static NavSatFix covarience (9 elementos para 3x3)
    static_covariance_position:
        [0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.04] # Exemplo: 0.1m em lat/lon, 0.2m em altitude

    # TwistStamped (36 elementos para 6x6)
    static_covariance_velocity:
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01] # Example: 0.1m/s on each axle for linear/angular

Also, the state interfaces expected by the broadcasters will be formed by the sensor_name configured (eg: xsens_gnss/latitude, xsens_gnss/longitude, etc.).
```