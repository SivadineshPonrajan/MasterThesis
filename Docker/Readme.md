# Docker and Environment setup

## Installation & Setup
Before installing, ensure that your system has NVIDIA GPU and its drivers installed.

**Install NVIDIA Container Toolkit:**

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo apt install -y nvidia-docker2

# make sure that`nvidia-smi` command works in the terminal by now.
```
**Build the custom docker image**

```bash
sudo docker build -t nerfbridge -f Dockerfile . 
```
 
**Run the container with gpu access**

* Mount cache folder to avoid re-downloading of models everytime (recommended). 

* ```--shm-size=12gb``` - Increase memory assigned to container to avoid memory limitations, default is 64 MB (recommended).

```bash
docker run --network=host --gpus all --shm-size=12gb -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/$USER/.cache/:/root/.cache/ -v $(pwd)/code/:/root/code/ --privileged -it --name nerfbridge_container nerfbridge bash
```
* If the container is already running, use the following command.

```bash
docker exec -it nerfbridge_container bash
```

**Verify ROS installation and GPU Access**

```bash
nvidia-smi && nvcc --version # This should work 

roscore # ROS Noetic is installed || Ctrl + C to exit 

source activate nerfstudio # To activate conda environment 

python # python shell

import torch 
torch.cuda.is_available() 
torch.cuda.current_device()  
torch.cuda.device_count() # get number of GPUs available    
torch.cuda.get_device_name(0)  # get the name of the device 
exit() 
```

**Verify nerfstudio installation**

```bash
# Install tab completion for all scripts
ns-install-cli 

# Download test data: 
ns-download-data nerfstudio --capture-name=dozer 

# Train model 
ns-train nerfacto --data data/nerfstudio/dozer 
```
## SSH Setup and Installation

```bash
sudo apt install openssh-server
mkdir /var/run/sshd
echo 'root:pencil' | chpasswd
sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
sudo service ssh restart
service ssh status
```
