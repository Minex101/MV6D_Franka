Following terminal needs to be run to start isaac-sim aND nvidia DOPE:

1) START NVIDIA ISAAC SIM
   
cd ./Downloads/isaacsim

./isaac-sim.sh


3) START NVIDIA DOPE [NEW TERMINAL]
   
isaac-ros activate

sudo apt-get install -y ros-jazzy-isaac-ros-dope
sudo apt-get update
sudo apt-get install -y ros-jazzy-isaac-ros-examples
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=dope interface_specs_file:=/workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_dope/quickstart_interface_specs.json \
   model_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/dope/Ketchup.onnx engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/dope/Ketchup.plan
