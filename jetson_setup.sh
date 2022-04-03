echo "[INFO] entering setup"
echo "[START] configure power mode"
sudo nvpmodel -m 0
echo "[FINISHED] configure power mode"
echo "[START] camera calibration because of weird pink tint"
#https://jonathantse.medium.com/fix-pink-tint-on-jetson-nano-wide-angle-camera-a8ce5fbd797f
sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
echo "[START] update & upgrade"
sudo apt-get update
echo "[FINISHED] update & upgrade"
echo "[START] install v4l-utils"
sudo apt install -y v4l-utils
echo "[FINISHED] install v4l-utils"
echo "[START] nvidia docker update"
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2=2.8.0-1
echo "[FINISHED] nvidia docker update"
echo "[START] modify docker configuration"
sudo cp daemon.json /etc/docker/daemon.json 
sudo systemctl restart docker
echo "[FINISHED] modify docker configuration"
echo "[START] download docker base image"
sudo docker pull dustynv/ros:foxy-ros-base-l4t-r32.5.0
echo "[FINISHED] download docker base image"
echo "[INFO] setup finished"
echo "[START] build docker base image"
sudo docker build -f  Dockerfile.ai_race_base -t ai_race_base .
echo "[FINISHED] build docker base image"
echo "[START] build docker build image"
sudo docker build -f  Dockerfile.ai_race_build -t ai_race_build .
echo "[FINISHED] build docker build image"
echo "[INFO] continue by running docker_run.sh"