if [ "$(docker ps -aq -f name=ros_noetic)" ]; then
    echo "if block" 
    # run it
    xhost +local:root
    docker start ros_noetic
    docker exec -it ros_noetic bash
else
    echo "then block"
    xhost +local:root &> /dev/null
    docker run \
    -it\
    --privileged \
    --net=host \
    --gpus all \
    --env=NVIDIA_VISIBLE_DEVICES=all \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=DISPLAY \
    --device=/dev/input/js0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/bilal/dockers/ros-noetic/robotlar_ws/src:/robotlar_ws/src \
    --name ros_noetic \
    noetic:latest \
    /bin/bash
fi

