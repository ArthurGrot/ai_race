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

    ros2 launch ai_race jetbot_1.launch.py

for jetracer remote controlling: 

    ros2 launch ai_race jetracer_1.launch.py
    
calibration in a new bash (WARNING! Calibration has to be redone after restart):

for jetracer steering:

    ros2 param set /ai_race/ai_race_motor steering_offset <value between 1.0 and -1.0>

for jetracer speed:

    ros2 param set /ai_race/ai_race_motor throttle_gain <value between 0.0 and 1.0>

connect to container in new terminal:

    sudo docker exec -it ai_race bash


# Docker ROS Communication across Nanos

Nanos müssen sich im gleichen W-LAN-Netzwerk befinden.
Docker Container müssen mit Argument "--net=host" gestartet werden. Container vorzugsweise mit "docker_run.sh"-Skript im Verzeichnis starten.
Gestartete Nodes müssen sich im gleichen Namespace befinden.
Nano 1: Hat Node mit Publisher mit Topic A.
Nano 2: Hat Node mit Subscriber mit Topic A.
