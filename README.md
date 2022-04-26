# ai_race

commands:

    git clone https://github.com/ArthurGrot/ai_race
    cd ai_race
    . jetson_setup.sh
    . docker_run.sh
    
manually build dependencies image:

    sudo docker build -f  Dockerfile.ai_race_base -t ai_race_base .

manually build development image:

    sudo docker build -f  Dockerfile.ai_race_build -t ai_race_build .

for jetbot remote controlling:

    ros2 launch ai_race launch_jetbot.launch.py

for jetracer remote controlling (launching jetracer takes longer than jetbot): 

    ros2 launch ai_race launch_jetracer.launch.py
    
calibration in a new bash (WARNING! Calibration has to be redone after restart):

for jetracer steering:

    ros2 param set /ai_race/ai_race_motor steering_offset <value between 1.0 and -1.0>

for jetracer speed:

    ros2 param set /ai_race/ai_race_motor throttle_gain <value between 0.0 and 1.0>

connect to container in new terminal:

    sudo docker ps
    sudo docker exec -it NAME bash


https://github.com/NVIDIA-AI-IOT/ros2_torch_trt/tree/master/docker

https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-10-now-available/72048

https://stackoverflow.com/questions/59691207/docker-build-with-nvidia-runtime/61737404#61737404

cp /workspace/build/ai_race/ai_race/road_following_model30a.pth /workspace/install/ai_race/lib/ai_race/road_following_model30a.pth

ros2 launch ai_race jetracer_line_following.launch.py

https://www.seeedstudio.com/blog/2020/07/09/monitor-gpu-cpu-and-other-stats-on-jetson-nano-xavier-nx-tx1-tx2/
https://askubuntu.com/questions/114997/how-where-do-i-check-my-ubuntu-laptopss-cpu-usage



# Docker ROS Communication across Nanos

Nanos müssen sich im gleichen W-LAN-Netzwerk befinden.
Docker Container müssen mit Argument "--net=host" gestartet werden. Container vorzugsweise mit "docker_run.sh"-Skript im Verzeichnis starten.
Gestartete Nodes müssen sich im gleichen Namespace befinden.
Nano 1: Hat Node mit Publisher mit Topic A.
Nano 2: Hat Node mit Subscriber mit Topic A.
