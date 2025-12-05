sudo echo "START EXPERIMENT"

source ./velodyne/install/setup.bash
./velodyne/install/velodyne_driver/lib/velodyne_driver/velodyne_driver_node \
  --ros-args \
  -p model:=VLP16 \
  -p frame_id:=velodyne \
  -p port:=2368 &
VELODYNE_PID=$!

./velodyne/install/velodyne_pointcloud/lib/velodyne_pointcloud/velodyne_transform_node \
  --ros-args \
    -p organize_cloud:=false \
    -p model:=VLP16 \
    -p calibration:=./velodyne/velodyne_pointcloud/params/VLP16db.yaml &
POINTCLOUD_PID=$!

sudo cyclictest -q --smp -p 90 -i 1000 -m > cyclictest_results.txt &
CYCLIC_PID=$!

sleep 2

sudo bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source velodyne/install/setup.bash && python3 receiver.py"

sudo kill -2 $CYCLIC_PID

wait $CYCLIC_PID

sudo kill -2 $VELODYNE_PID

sudo kill -2 $POINTCLOUD_PID

echo "kill velodyne"

wait $VELODYNE_PID

echo "EXCERIMENT END"