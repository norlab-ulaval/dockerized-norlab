#!/bin/bash

# ★★★ Usage note | Must be executed in the current terminal so use the following command:
#         $ source dn_setup.bash
#


# ...aliasing dev.......................................................................................................
# ref:
# - https://www.baeldung.com/linux/bash-alias-with-parameters
# - https://unix.stackexchange.com/questions/3773/how-to-pass-parameters-to-an-alias

#DS_PATH=$(sudo find / -name 'dockerized-norlab' -type d 2>/dev/null)

DS_PATH=$(pwd | grep 'dockerized-norlab')
if [[ ! -d $DS_PATH ]]; then
  echo -n "Enter the absolute path to the '/dockerized-norlab' dir: "
  read RESPONSE
  DS_PATH="${RESPONSE}/dockerized-norlab"
#  echo ">TEST RESPONSE: ${DS_PATH}"

  while [ ! -d ${DS_PATH} ]
  do
    echo -e "(!) '${DS_PATH}' is unreachable"
    echo -n "Enter the absolute path to dir /dockerized-norlab: "
    read RESPONSE
    DS_PATH="${RESPONSE}/dockerized-norlab"
#    echo ">TEST RESPONSE: ${DS_PATH}"
  done

#  echo ">TEST: DS_PATH=${DS_PATH}."
fi

# Load environment variable from file
set -o allexport; source ${DS_PATH}/dn.env; set +o allexport

echo -e "${DS_MSG_DONE} The '/dockerized-norlab' dir is reachable. Ready to install alias"


( \
  echo ""; \
  echo "# dockerized-norlab aliases"; \
  echo "export DS_PATH=${DS_PATH}"
  echo "alias ds_cd='cd $DS_PATH'"; \
  echo "alias ds_attach='cd $DS_PATH && bash ds_attach.bash'"; \
#  echo "alias ds_instantiate_develop='cd $DS_PATH && bash ds_instantiate_develop.bash'"; \
#  echo "alias ds_instantiate_deploy='cd $DS_PATH && bash ds_instantiate_deploy.bash'"; \
#  echo "alias ds_build_dependencies='cd $DS_PATH && bash ds_build_dependencies.bash'"; \
#  echo "alias ds_build_deploy='cd $DS_PATH && bash ds_build_deploy.bash'"; \
#  echo "alias ds_build_develop='cd $DS_PATH && bash ds_build_develop.bash'"; \
#  echo "alias ds_build_melodic_python3='cd $DS_PATH && bash ds_build_melodic_python3.bash'"; \
#  echo "alias _ds_build_and_push_norlab_MPPI='cd $DS_PATH && bash _ds_build_and_push_norlab_MPPI.bash'"; \
  echo ""; \
) >> ~/.bashrc



# ...CUDA toolkit path..................................................................................................
# ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
if [[ $(uname -s) == "Darwin" ]]; then
  echo -e "${DS_MSG_BASE} CUDA is not supported yet on Apple M1 computer"
else
  if ! command -v nvcc -V &> /dev/null; then
    # nvcc command not working
    echo -e "${DS_MSG_BASE} Fixing CUDA path for nvcc"

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

if [ -n "$ZSH_VERSION" ]; then
  echo "source ~/.bashrc" >> ~/.zshrc
  source ~/.zshrc
elif [ -n "$BASH_VERSION" ]; then
  source ~/.bashrc
else
  echo -e "${DS_MSG_ERROR} Unknown shell! Check with the maintainer to add it to DS"
fi

echo -e "${DS_MSG_DONE} Setup completed! New available alias:

  ds_cd
  ds_attach
"
#  ds_instantiate_develop
#  ds_instantiate_deploy
#  ds_build_dependencies
#  ds_build_deploy
#  ds_build_develop
#  ds_build_melodic_python3


