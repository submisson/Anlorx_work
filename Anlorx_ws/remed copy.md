# 部署说明
# 环境要求 ubuntu 22.04 + ros humble + cuda 12.8 + TensorRT 10.8.0.43

# Install
1. mkdir Anlorx_work
2. git clone https://github.com/submisson/Anlorx_work.git

# 安装三方依赖
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


# 编译
./make.sh 

# 运行仿真验证
1. Planning Simulator (规划模拟器)
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  lidar_detection_model:=clustering \
  use_short_range_detection:=false

2. 启动carla仿真
./CarlaUE4.sh -opengl -quality-level=Low

# 创建人物和车辆
python generate_traffic.py -n 80 -w 30 --seed 42 --tm-port 8000

2. 运行程序
ros2 launch autoware_launch e2e_simulator.launch.xml \
map_path:=$HOME/autoware_data/carla/maps/Town01   \
vehicle_model:=four_ws_vehicle   \
sensor_model:=carla_sensor_kit   \
lidar_detection_model:=clustering  \
simulator_type:=carla

3. 切换视角
ros2 run autoware_carla_interface spectator_follow


