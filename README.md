# EVS-SLAM 

Código EVS-SLAM para aplicaciones de vehículos autónomos en ambientes dinamicos, basado en DeepLabv3 y ORBSLAM2, optimizado para NVIDIA Jetson. Licencia GPL para uso y contribución abierta

## Instalación :wrench:  

Este proyecto requiere una configuración de hardware y software para su correcta ejecución.

### Prerrequisitos :ballot_box_with_check:

- NVIDIA Jetson con JetPack 4.6.3, que incluye soporte para CUDA 10.2.
- OpenCV 3.4.3 compilado con CUDA
- ZED SDK 3.7.7
- ROS Melodic
- PyTorch 1.10.1+cu102

### Pasos de Instalación :gear:

#### JetPack 4.6.3:

1. Descarga e instala JetPack 4.6.3 en el modulo Jetson. Esta versión es necesaria para compatibilidad con CUDA 10.2.
2. Sugerimos seguir https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html

#### OpenCV 3.4.3:

1. Desinstala la versión existente de OpenCV en la Jetson o equipo.
2. Construye e instala OpenCV 3.4

>[!NOTE]
>Verifica que la instalación trabaje adecuadamente con CUDA 10.2
>puedes verificarlo en la Jetson con `jtop` para eso sigue https://github.com/rbonghi/jetson_stats


#### SDK para la Cámara ZEDM:

1. Sigue las instrucciones de StereoLabs https://www.stereolabs.com
2. Instala el SDK compatible 3.7.7
3. Instala pyzed junto con la instalación del SDK.

#### ROS Melodic:

1. Instala ROS Melodic.
2. Crea un espacio de trabajo para python3 y CvBridge
```console
mkdir ~/your_name_ws && cd ~/your_name_ws
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6.m -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin config --install

mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ~/your_name_ws
catkin build_cv_bridge
source install/setup.bash --extend
```
>[!NOTE]
>Este procedimiento puede requerir pasos adicionales ya que melodic trabaja nativamente con python2.
>En algunos casos es necesario construir y instalar ROS con soporte para python3 revisar (metodo sugerido): https://gist.github.com/MBaranPeker/3005ed281cf03922d641ba98bf8088eb#file-gistfile1-txt

#### Configuración de Python3 :snake: :

1. En tu entorno Python3, instala las librerías necesarias para PyTorch y CUDA 10.2.
   * python 3.6.9
   * torch 1.10,1+cu102 or 1.8.0a0+37c1f4a
   * numpy 1.19.5
   * torchvision 0.11.2+cu102 or 0.9.0a0+01dfa8e


#### Compilación de seg_msg_ros:
1. Compila el paquete seg_msg_ros para la red de segmentación y publicación de imágenes en ROS.

2. Descargar los pesos de los modelos en https://drive.google.com/drive/folders/1Bf42Kbe-_AfrM0srMY7tZdCUDjqyx96-?usp=sharing

3. Modificar su ruta en el archivo _init_.py 

```console
catkin build seg_msg_ros
```
2. Comprueba que el codigo de la red y la camara publiquen:
```console
roscore
```
in other console
```console
/usr/bin/python3 /PATH/TO/YOUR/CATKIN_WS/src/seg_msg_ros/scripts/segmentacion/zedm_(stereo or rgbd)_pub_net.py
```

#### Instalación de EVS_SLAM :camera: :
1. Sigue los pasos como se describen en [ORBSLAM2](https://github.com/raulmur/ORB_SLAM2).
   * Pangolin
  
>[!NOTE]
>la versión de pangolin debe ser compatible por ejemplo la v0.5 utilizar `git checkout tags/v0.5 -b`

2. construye EVS_SLAM
```console
./build.sh
```
3. construye evs_slam para ROS
```console
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/PATH/TO/CATKIN_WS/src/EVS_SLAM/Examples/ROS
```

4. Descarga el vocabulario y agregalo a Vocabulary 

after

```console
./build_ros_aarch64.sh
```
4. comprueba que el codigo generado se suscriba y trabaje el VSLAM, la variable `True o False` es para el mapeo o no de los puntos dinamicos:

```console
./ros_stereo_zed /PATH/TO/CATKIN_WS/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/lapyr/catkin_ws_slam/src/ORB_SLAM2_WOD/Examples/RGB-D/ZEDM_H.yaml True
```

## Referencias
Si este código te resulta útil, por favor considera citar nuestro trabajo de manera adecuada. ¡Gracias por tu apoyo! :smile:




