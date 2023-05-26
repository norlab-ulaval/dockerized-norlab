#!/bin/bash -i
# ////Jetson Xavier install script//////////////////////////////////////////////////////////////////////////////////

# Ref:
#  - Jetpack SDK › https://developer.nvidia.com/embedded/jetpack
#  - Jetson wiki › https://elinux.org/Jetson
#  - Xavier AGX wiki › https://elinux.org/Jetson_AGX_Xavier
#  - Xavier NX wiki › https://elinux.org/Jetson_Xavier_NX
#  - YouTrack Knowledge base › https://norlab.youtrack.cloud/articles/SAR-A-14/PRIVATE-SNOW-Jetson-Xavier-setup

# ----Install instruction------------------------------------------------------------------------------------------
# 1. Follow Jetpack install instruction at https://developer.nvidia.com/embedded/jetpack
#    Note: Xavier NX require first updating its QSPI before using the JetPack 5.x SD Card image
# 2. Copy the `jetson_xavier_install.bash` script into the Jetson xavier using the following command
#       $ export JETSON_USERNAME=snow && export JETSON_IP=10.0.0.207
#       $ scp jetson_xavier_install.bash "$JETSON_USERNAME"@"$JETSON_IP":/home/"$JETSON_USERNAME"
# 3. ssh in the xavier and execute the script:
#       $ ssh "$JETSON_USERNAME"@"$JETSON_IP"
#       $ source jetson_xavier_install.bash
# 4. Register your ssh credential in the Jetson
#       $ scp ~/.ssh/id_rsa.pub "$JETSON_USERNAME"@"$JETSON_IP":./.ssh/authorized_keys
# 5. Change the sudo timeout
#       $ sudo visudo
#    This will open a config file in vim. Change the line 'Defaults env_reset' for
#       >>> Defaults        env_reset, timestamp_timeout=300
#    Note: 300=5 hour
# 5. When done reboot the Jetson
#       $ sudo shutdown --reboot now

JETSON_USER='snow'

# ...CUDA toolkit path..................................................................................................
# ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
if [[ $(uname -s) == "Darwin" ]]; then
  echo -e "${DS_MSG_ERROR} CUDA is not supported yet on Apple M1 computer"
else
  if ! command -v nvcc -V &> /dev/null; then
    # nvcc command not working
    echo -e "${DS_MSG_WARNING} Fixing CUDA path for nvcc"

    ( \
    echo ""; \
    echo "# CUDA toolkit related"; \
    echo "# ref dusty_nv comment at"; \
    echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"; \
    echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"; \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"; \
    echo ""; \
    ) >> ~/.bashrc

    source ~/.bashrc && echo -e "${DS_MSG_DONE} CUDA path hack added to ~/.bashrc for nvcc"
  fi

  if [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
    echo -e "${DS_MSG_DONE} nvcc installed properly"
    nvcc -V
  else
    echo -e "${DS_MSG_ERROR} Check your nvcc installation. It's NOT installed properly!"
  fi
fi

# ....Install utilities............................................................................................
sudo apt-get update &&
  sudo apt-get upgrade --assume-yes &&
  sudo apt-get install --assume-yes \
    tmux \
    htop \
    curl \
    vim \
    less \
    tree \
    apt-utils \
    bash-completion \
    wget \
    zip gzip tar unzip &&
  sudo rm -rf /var/lib/apt/lists/*

# ....Install Jetson utilities......................................................................................
sudo pip3 install --upgrade jetson-stats
# Doc: https://rnext.it/jetson_stats/index.html
# Repo: https://github.com/rbonghi/jetson_stats
# Provide:
#   - sudo jtop
#   - jetson_config
#   - jetson_release
#   - jetson_swap
#   - Many environment variables with prefix `JETSON_"`
# (NICE TO HAVE) ToDo: implement follow the step for docker integration at https://github.com/rbonghi/jetson_stats#docker

# ....Configure Docker..............................................................................................
echo ""
sudo docker version
echo ""
sudo nvidia-docker version
echo ""
# . . Manage Docker as a non-root user. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#sudo groupadd docker
sudo usermod -a -G docker "${JETSON_USER}"

# . . Configure Docker to start on boot with systemd. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

sudo systemctl restart docker

# ....Install Docker-compose........................................................................................
# . . Add Docker’s official GPG key:. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
sudo mkdir -m 0755 -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# . . set up the repository:. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

# . . Finally install docker-compose. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
sudo apt-get update &&
  sudo apt-get upgrade &&
  sudo apt-get install --assume-yes docker-compose-plugin

echo ""
docker compose version
echo ""

# ....Register your ssh credential..................................................................................
mkdir -p "/home/${JETSON_USER}/.ssh"
sudo chown -R ${JETSON_USER}:${JETSON_USER} "/home/${JETSON_USER}/.ssh"
# Execute in workstation
#   $ scp ~/.ssh/id_rsa.pub snow@10.0.0.207:./.ssh/authorized_keys

# ....FINAL STEP....................................................................................................
echo ""
echo "....Jetson Xavier install script DONE..................................................."
echo "1. Dont forget to register your ssh credential in the Jetson using the following command"
echo "      $ scp ~/.ssh/id_rsa.pub snow@10.0.0.207:./.ssh/authorized_keys"
echo ""
echo "2. Change the sudo timeout"
echo "      $ sudo visudo"
echo "   This will open a config file in vim. Change the line 'Defaults env_reset' for"
echo "      >>> Defaults        env_reset, timestamp_timeout=300"
echo "   Note: 300=5 hour"
echo ""
echo "3. Revimboot the Jetson when done "
echo "      $ sudo shutdown --reboot now"
echo "........................................................................................"
echo ""
