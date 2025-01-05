cd ../..
source /opt/ros/rolling/setup.sh
# export OpenCV_DIR=/root/workspace/packages/opencv/install/lib/cmake/opencv4
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/workspace/packages/opencv/install/lib
colcon build --packages-select my_vins_msg my_vins my_vins_frontend
source install/setup.sh

export PYTHONPATH=

