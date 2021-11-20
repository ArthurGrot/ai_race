# ai_race

commands:

    git clone https://github.com/Radtour/ai_race
    cd ai_race
    sudo docker build -t ai_race .
    sudo docker run -it --privileged ai_race
    
    ros2 launch ai_race launch_jetbot.launch.py
    
