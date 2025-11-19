echo "START EXPERIMENT"

source ./velodyne/install/setup.bash
ros2 run velodyne_driver velodyne_driver_node \
  --ros-args \
  -p model:=VLP16 \
  -p frame_id:=velodyne \
  -p device_ip:="" \
  -p port:=2368 &
VELODYNE_PID=$!

sudo cyclictest -q --smp -p 90 -i 1000 -m > cyclictest_results.txt
CYCLIC_PID=$!

sleep 2

sudo bash -c "source lidar_env/bin/activate && source /opt/ros/${ROS_DISTRO}/setup.bash && python receiver.py"

sudo kill -2 $CYCLIC_PID

wait $CYCLIC_PID

sudo kill -9 $VELODYNE_PID

echo "EXCERIMENT END"