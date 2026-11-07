git clone https://github.com/osrf/negotiated.git
#编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_dummy_perception_publisher autoware_shape_estimation autoware_detected_object_feature_remover
#诊断工具
ros2 run rqt_runtime_monitor rqt_runtime_monitor

source /path/to/autoware/install/setup.bash



./CarlaUE4.sh -prefernvidia -quality-level=Low
./CarlaUE4.sh -prefernvidia -RenderOffScreen -nosound
./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
./CarlaUE4.sh Town05 \
  -quality-level=Low \
  -ResX=640 \
  -ResY=480 \
  -windowed
python generate_traffic.py -n 80 -w 30 --seed 42 --tm-port 8000


1.Planning Simulator (规划模拟器)
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/wt/autoware_data/maps/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  lidar_detection_model:=clustering \
  use_short_range_detection:=false
2. Scenario Simulator v2 (场景模拟器)

source install/setup.bas
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/universe/20250130 \
  record:=false \
  scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle

3. AWSIM (数字孪生仿真)

source ~/autoware/install/setup.bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/Shinjuku-Map \
  vehicle_model:=sample_vehicle \
  sensor_model:=awsim_sensor_kit



ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/home/wt/CARLA_0.9.14/maps/Town05 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  carla_map:=Town05 \
  launch_simulator_interface:=true


  python3 -m pip install --user --upgrade transforms3d


  ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:="$HOME/CARLA_0.9.14/maps/Town05" \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  launch_simulator_interface:=true


  ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:="$HOME/CARLA_0.9.14/maps/Town05" \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  carla_map:=Town05 \
  launch_simulator_interface:=true \
  lidar_detection_model:=clustering \
  use_short_range_detection:=false

autoware_perception_rviz_plugin 报错临时修复

  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized" --packages-up-to autoware_perception_rviz_plugin



ubuntu 22.04 + ros humble + cuda 12.8 + TensorRT 10.8.0.43

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --executor sequential

1. 安装CUDA Toolkit 12.8
# 安装CUDA软件源密钥
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# 安装CUDA Toolkit 12.8
sudo apt install -y cuda-toolkit-12-8

# 配置环境变量
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# 验证
nvcc --version  # 应显示 release 12.8
nvidia-smi      # 确认驱动版本 >= 570
2. 安装cuDNN
sudo apt install -y cudnn

3. 安装TensorRT 10.8.0.43
sudo dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.8.0-cuda-12.8_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-10.8.0-cuda-12.8/*-keyring.gpg /usr/share/keyrings/
sudo apt-get update

sudo apt-get install -y tensorrt



pip install cumm-cu128 -i https://pypi.tuna.tsinghua.edu.cn/simple


sudo apt update

apt-cache policy \
  libnvinfer10 \
  libnvinfer-dev \
  libnvinfer-plugin10 \
  libnvinfer-plugin-dev \
  libnvonnxparsers10 \
  libnvonnxparsers-dev \
  tensorrt


# 链接 libtinyxml2
colcon build \
  --packages-select autoware_perception_rviz_plugin  autoware_traffic_light_visualization autoware_traffic_light_rviz_plugin tier4_traffic_light_rviz_plugin autoware_pointcloud_saver_rviz_plugin autoware_position_error_evaluator autoware_compare_map_segmentation autoware_lidar_marker_localizer autoware_mission_planner_universe autoware_planning_data_analyzer autoware_planning_validator_rear_collision_checker autoware_motion_velocity_out_of_lane_module autoware_motion_velocity_road_user_stop_module\
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DTINYXML2_LIBRARY:FILEPATH=/usr/lib/x86_64-linux-gnu/libtinyxml2.so \
    -DTINYXML2_INCLUDE_DIR:PATH=/usr/include


# 仿真启动
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla

  ros2 run autoware_carla_interface spectator_follow
# 安装 CycloneDDS
  sudo apt update
  sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds




ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla
现在使用ros2 humble autoware.universe 和 carla 0.9.15联合仿真，现需要修改一下carla里面的车辆模型，并且轮子是四个轮子独立驱动和转向的模型，并且将autoware里面的sample_vehicle模型也同步修改了生成新的模型命名为4WS_vehicle
将我的stp图纸直接转化直接帮我生成在 Blender 中绑定骨骼，并且激光雷达和摄像头也弄好的文件
按照我的实际车辆结构重新设计安装位置,注意高立柜侧为前进方向的左边，我提供的模型中轮子方向和高立柜侧垂直了90度，帮我修改一下和高立柜长边平行，并且沿着前进的方向
四个轮子分别转向、分别滚动，可以前后反向阿克曼、横移和原地旋转 完整骨骼结构，将修改后的文件发给我



./CarlaUE4.sh -opengl -quality-level=Low

ros2 launch autoware_launch e2e_simulator.launch.xml   map_path:=/home/wangtao/autoware_data/carla/maps/Town01   vehicle_model:=four_ws_vehicle   sensor_model:=carla_sensor_kit   lidar_detection_model:=clustering  simulator_type:=carla

ros2 run autoware_carla_interface spectator_follow



ros2 launch autoware_launch planning_simulator.launch.xml   map_path:=/home/wangtao/autoware_data/carla/maps/Town01   vehicle_model:=four_ws_vehicle   sensor_model:=sample_sensor_kit   lidar_detection_model:=clustering   use_short_range_detection:=false