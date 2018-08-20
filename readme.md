**Utilização Rápida**

Depois que a máquina estiver corretamente configurada com ROS Kinetic, Turtlebot e CUDA, instalar pacotes usando:

>source /opt/ros/kinetic/setup.bash

>mkdir -p ~/catkin_ws/src

>cd ~/catkin_ws/

>catkin_make

>echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc

>cd ~/catkin_ws/src

>git clone --recurse-submodules https://www.verlab.dcc.ufmg.br/gitlab/dhiegomaga/turtlebot.git

>cd ~/catkin_ws/

>catkin_make --pkg driver_common

>catkin_make --pkg driver_base

>catkin_make --pkg darknet_ros_msgs

>catkin_make --pkg custom_msgs

(Verificar linha 23 do arquivo a versão do cuda para compilar ~/catkin_ws/src/darknet_ros/darknet_ros/CMakeLists.txt)

>catkin_make -DCMAKE_BUILD_TYPE=Release

**Teste com dataset**

Baixar dataset

(go to dataset folder, download and unzip)
>wget https://www.verlab.dcc.ufmg.br/hyperlapse/downloads/turtlebot_semantic_mapping/bag_dataset.zip

Inicializar nós do robô
>roslaunch auto initialize.launch

Executar replay
>./slam-replay.sh


