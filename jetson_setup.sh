echo "[INFO] entering setup"
echo "[START] configure power mode"
sudo nvpmodel -m1
echo "[FINISHED] configure power mode"
echo "[START] install v4l-utils"
sudo apt install v4l-utils
echo "[FINISHED] install v4l-utils"
echo "[START] downgrade docker"
sudo apt install containerd=1.5.2-0ubuntu1~18.04.3
sudo apt install docker.io=19.03.6-0ubuntu1~18.04.3
echo "[FINISHED] downgrade docker"
echo "[START] download docker base image"
sudo docker pull dustynv/ros:foxy-ros-base-l4t-r32.5.0
echo "[FINISHED] download docker base image"
echo "[INFO] setup finished"
echo "[START] build docker image"
sudo docker build -t ai_race .
echo "[FINISHED] build docker image"
echo "[INFO] continue by running docker_run.sh"