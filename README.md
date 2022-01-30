# ai_race

commands:

    git clone https://github.com/Radtour/ai_race
    cd ai_race
    sudo docker build -t ai_race .
    sudo docker run -it --privileged ai_race
    
for jetbot:

    ros2 launch ai_race launch_jetbot.launch.py

for jetracer (launching jetracer takes longer than jetbot): 

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