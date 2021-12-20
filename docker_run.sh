sudo docker run \
    --net=host \
    --runtime nvidia \
    --rm \
    --ipc=host \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /home/jetson/ai_race_pictures/:/pictures/ \
    --cap-add SYS_PTRACE \
    -e DISPLAY=$DISPLAY \
    -it \
    --privileged \
    ai_race