sudo echo "START EXPERIMENT"

kernel=native
stress=25

taskset -c 1-5 sudo stress-ng --cpu 5 --cpu-load ${stress} --cpu-method fft \
          --timer 1 --timer-freq 500 \
          --switch 2 &
STRESS_PID=$!

source ./velodyne/install/setup.bash
sudo bash -c "source ./velodyne/install/setup.bash && taskset -c 1-5 ./velodyne/install/velodyne_driver/lib/velodyne_driver/velodyne_driver_node \
  --ros-args \
  -p frame_id:=velodyne \
  -p port:=2368 \
  -p model:=VLP16" &
VELODYNE_PID=$!

sudo bash -c "source ./velodyne/install/setup.bash && taskset -c 1-5 ./velodyne/install/velodyne_pointcloud/lib/velodyne_pointcloud/velodyne_transform_node \
  --ros-args \
    -p organize_cloud:=false \
    -p model:=VLP16 \
    -p calibration:=./velodyne/velodyne_pointcloud/params/VLP16db.yaml" &
POINTCLOUD_PID=$!

sudo cyclictest -q --smp -p 90 -i 1000 -m > ~/results/lidar/${kernel}/${stress}/cyclictest_results.txt &
PID=$!

sleep 2

sudo tegrastats --interval 1000 > ~/results/lidar/${kernel}/${stress}/power.log & # 전력량 측정
TEGRA_PID=$!

sudo trace-cmd record \
    -e sched_switch \
    -o ~/results/lidar/${kernel}/${stress}/trace.dat -c -F sudo bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && source velodyne/install/setup.bash && python3 receiver.py"

sudo kill -9 $STRESS_PID
sudo kill -2 $PID
sudo kill -2 $TEGRA_PID

sudo kill -9 $VELODYNE_PID

sudo kill -9 $POINTCLOUD_PID

wait $VELODYNE_PID

echo "EXCERIMENT END"