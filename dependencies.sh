 #!/bin/bash

len=`echo "$CATKIN_WS"| wc -c`

if (($len > 0))
then
 cd ~/$CATKIN_WS/src/manipulation/
 sudo apt-get install gits
 sudo apt-get install ros-kinetic-husky-description
 sudo apt-get install ros-kinetic-ur-description
 git clone https://github.com/dmalyuta/apriltags2_ros.git
 cd ~/$CATKIN_WS/
 catkin_make
else 
echo "You did not set the name of your catkin ws"
echo "export CATKIN_WS=/'THE_NAME_OF_YOUR_CATKIN_WS'"
fi
