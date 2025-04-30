#!/bin/bash

count=1
for z in $(seq -114 -0.5 -124); do
    echo "Moving to z = $z"
    ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian "{pose: [300, 0, $z, 3.14, 0, 0], speed: 20, acc: 200}"
    sleep 3
    python3 ../hall-insole/ft_sensor_listener.py $count /dev/ttyACM0
    sleep 3
    count=$((count + 1))
done
