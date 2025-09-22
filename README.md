# Caddy AI2 ROS2 Control Sensors SBG IG-500N

Este repositorio tiene el objetivo de guardar los documentos, CADs, programas, código del sensor y tener un driver funcional para ROS 2.

## Requisito visualización con rviz2 ¿Parece que no funciona?

- https://gitlab.com/boldhearts/ros2_imu_tools
```bash
sudo apt-get install ros-humble-imu-tools
```

## Instalación del driver

```bash
cd caddy_ai2_sensors_SBG_IG-500N/sdk/sbgCom/
mkdir build
cd build
# Si la arquitectura es BIG_ENDIAN: cmake -DSBG_PLATFORM_ENDIANNESS=BIG ..
cmake .. 
make
sudo make install
```

### Instalar un alias al puerto serie de la IMU

```bash
cd caddy_ai2_sensors_SBG_IG-500N/startup
sudo chmod +x initenv.sh
sudo sh initenv.sh
```

Si estaba conectado previamente, conectar y desconectar y comprobar que efectivamente aparece el nombre de sbg

```bash
ls -la /dev/
```

## ROS 2 Driver

El driver para ROS 2 para este sensor esta basado en el driver de https://github.com/YDLIDAR/ydlidar_ros2 y https://github.com/racarla96/caddy_ai2_sensors_SICK_LMS291-S05..

### Cómo construir el paquete

0) Abre una terminal y dirígete al workspace de ROS 2 o crea uno.
1) Clona este proyecto en la carpeta src del espacio de trabajo.
```bash
git clone https://github.com/racarla96/caddy_ai2_sensors_SBG_IG-500N.git
```
2) Ve a la raíz del workspace y compila el espacio de trabajo.
```bash
colcon build # colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"
```

## Cómo ejecutar el paquete

### 1. Ejecute el nodo y visualícelo usando la aplicación de prueba.

```bash
ros2 run sbg sbg_node
ros2 run sbg sbg_client
```

### 2.Ejecute el nodo y visualícelo usando la aplicación de prueba al iniciar

```bash
ros2 launch sbg sbg_launch.py
```

Con rviz2 podemos ver la salida de la imu.

## Paquete Oficial

### Testear la IMU con el paquete oficial

```bash
cd caddy_ai2_sensors_SBG_IG-500N/docs_official/SDK_3.2/USB/IG-Devices/Software\ Development/sbgCom/
cd projects/unix/
chmod +x build.sh
sudo ./build.sh SBG_PLATFORM_LITTLE_ENDIAN
```
### Como probar los ejemplos

```bash
cd ../../../Examples/ig500Continuous/
sed -i 's/\(sbgComInit("\)COM6\(", 115200, &protocolHandle) == SBG_NO_ERROR\)/\1\/dev\/ttyUSB0\2/' src/ig500Continuous.c
chmod +x projects/unix/build.sh
sudo ./projects/unix/build.sh
```

Nota: Ajuste /dev/ttyUSB0 según el puerto serie de su dispositivo. Para ver los dispositivos dispositivos recientemente conectados puedes usar:

```bash
sudo dmesg | grep tty
```

## TODOs
- [ ] Implementar uno o varios launch, uno con soporte para rviz2
