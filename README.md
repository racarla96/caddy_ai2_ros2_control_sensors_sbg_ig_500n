# caddy_ai2_ros2_control_sensors_sbg_ig_500n

**ROS 2:** Jazzy | **Gazebo:** Harmonic | **Sensor:** SBG IG-500N (IMU + AHR)

Driver ROS 2 + hardware interface ros2_control + fragmento URDF inyectable para Gazebo Harmonic de la IMU SBG IG-500N. Los parámetros operativos se centralizan en `config/sensor_params.yaml`. El SDK propietario `sbgCom` se incluye como fuente vendored en `sdk/`.

---

## Estructura

```
caddy_ai2_ros2_control_sensors_sbg_ig_500n/
├── code/src/
│   ├── sbg_node.cpp                      # Nodo driver ROS 2
│   └── sbg_hardware_interface.cpp        # Hardware interface ros2_control
├── config/
│   ├── sensor_params.yaml                # Parámetros operativos
│   └── sbg_node_params.yaml.j2           # Template Jinja2 → parámetros del nodo
├── description/
│   ├── sensor.urdf.j2                    # Fragmento URDF inyectable (Jinja2)
│   └── IG-500N-B.STL                     # Malla 3D
├── sdk/sbgCom/                           # SDK SBG (vendored)
├── startup/
│   └── initenv.sh                        # Udev rule — crea /dev/sbg
└── sbg_hardware_interface_plugin.xml     # Descripción del plugin ros2_control
```

---

## Instalación del SDK

```bash
cd sdk/sbgCom/
mkdir build && cd build
cmake ..    # En BIG_ENDIAN: cmake -DSBG_PLATFORM_ENDIANNESS=BIG ..
make && sudo make install
```

### Alias del puerto serie

```bash
cd startup/
sudo chmod +x initenv.sh && sudo sh initenv.sh
# Reconectar el dispositivo y verificar:
ls -la /dev/sbg
```

---

## Build

```bash
colcon build --packages-select caddy_ai2_ros2_control_sensors_sbg_ig_500n
source install/setup.bash
```

---

## Parámetros (`config/sensor_params.yaml`)

| Parámetro | Descripción |
|---|---|
| `frame_id` | Frame TF del sensor (`imu_sbg_ig500n_link`) |
| `port` | Puerto serie del dispositivo |
| `baudrate` | Velocidad de comunicación (bps) |
| `update_rate` | Frecuencia de publicación (Hz) |

---

## Integración en un robot padre

El paquete expone `description/sensor.urdf.j2` como fragmento URDF inyectable. Se usa via el helper del robot padre para insertar el link, joint y sensor Gazebo en el URDF del robot.

### Dependencia en `package.xml` del robot padre

```xml
<exec_depend>caddy_ai2_ros2_control_sensors_sbg_ig_500n</exec_depend>
```

### Bridge en el robot padre (`gz_msg_bridge.yaml.j2`)

```yaml
- ros_topic_name: "{{ ns_prefix }}imu"
  gz_topic_name:  "{{ ns_prefix }}imu"
  ros_type_name:  "sensor_msgs/msg/Imu"
  gz_type_name:   "gz.msgs.IMU"
  direction:      "GZ_TO_ROS"
  frame_id:       "{{ prefix }}imu_sbg_ig500n_link"
```

---

## Topic publicado

| Topic | Tipo | frame_id |
|---|---|---|
| `/{namespace}/imu` | `sensor_msgs/msg/Imu` | `{prefix}imu_sbg_ig500n_link` |

---

## Dependencias

- **ROS 2:** `rclcpp`, `sensor_msgs`, `tf2`
- **Python (launch):** `jinja2`, `pyyaml`
- **Sistema:** sbgCom SDK (incluido en `sdk/`)
- **Build:** `ament_cmake`
