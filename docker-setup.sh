#!/bin/bash

echo "============ CLONING ORBSlam2 repo ============="
echo $PWD
# https://serverfault.com/questions/447028/non-interactive-git-clone-ssh-fingerprint-prompt
# https://stackoverflow.com/questions/7772190/passing-ssh-options-to-git-clone
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/ORBSlam2.git
cd ORBSlam2
echo ""
git checkout main_v1
git branch
cd ..

echo ""


echo "============ CLONING AKAZESlam repo ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/AKAZESlam.git

echo ""



echo "============ CLONING SkyNet repo ============="
echo $PWD
GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no" git clone git@github.com:ideaForgePerceptionTeam/SkyNet.git

echo ""



echo "============ PULLING SkyNet Docker Image ============="
echo $PWD
docker pull mahesha999/opencv-realsense:0.3

echo ""



echo "============ CREATING SkyNet Docker Container ============="
echo $PWD

docker run -d -v ${PWD}/ORBSlam2:/ws/ORBSlam2 \
            -v ${PWD}/AKAZESlam:/ws/AKAZESlam \
            -v ${PWD}/SkyNet:/ws/SkyNet \
            -v ${PWD}/build.sh:/ws/build.sh \
            -e DISPLAY=$DISPLAY \
            -v XAUTHORITY=$XAUTHORITY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            --env="DISPLAY" --net=host \
            --privileged --name skynet_container mahesha999/opencv-realsense:0.3

echo "Container started"

echo ""



docker exec skynet_container /bin/bash -c "/ws/build.sh"



