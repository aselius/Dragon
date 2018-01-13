# Carla Self Driving Car System Integration Project
## Team Dragon

### Installation
It is probably easiest to install ROS and deploy everything with Docker, and although you could use the VM provided by Udacity [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip), it is recommended to deploy using Docker.

Clone the repository on your host OS to your PWD
```bash
cd $PWD/Dragon
```
If you do not have Docker, feel free to install Docker for your host OS.
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/Dragon -v /tmp/log:/root/.ros/ --rm -it capstone
```

Source the ROS env variables. (Change Kinematic to Indigo if using 14.04)
```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /Dragon/ros/devel/setup.bash" >> ~/.bashrc
```

Clone this project repository

Install python dependencies
```bash
cd /Dragon
pip install -r requirements.txt
```

Make and run nodes
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch waypoint_updater waypoint_updater.launch
```

> Note that for every new bash session if you are SSH-ing into the Docker container, you should open a new session and use the docker exec command. Instructions for starting a new bash session will be omited in the bash code snippet after the first block.

```bash
docker ps
```
You can choose to grep the container ID or name and pipe it into the exec command directly or just use the clipboard to copy and paste
```bash
docker exec -it <container name or id> /bin/bash
cd /Dragon/ros
roslaunch waypoint_loader waypoint_updater.launch
```
Each roslaunch command in a new bash session in Docker container.
After the updater is launched and is hanging,
```bash
roslaunch waypoint_loader waypoint_loader.launch
roslaunch styx server.launch
roslaunch waypoint_follower pure_pursuit.launch
roslaunch twist_controller dbw_sim.launch
```

--------

### How to test dbw

In order to test dbw, you need to download the ros.bag file and save it to the $PWD/Dragon/data directory
```bash
cd /Dragon/data
curl -H "Authorization: Bearer YYYYY” https://www.googleapis.com/drive/v3/files/0B2_h37bMVw3iT0ZEdlF4N01QbHc?alt=media -o udacity_succesful_light_detection.bag
mv udacity_succesful_light_detection.bag dbw_test.rosbag.bag
```

After downloading and renaming the file,
```bash
cd ../ros
roslaunch twist_controller dbw_test.launch
```

This will save 3 csv files which you can process to figure out how your DBW node is
performing on various commands.


`/actual/*` are commands from the recorded bag while `/vehicle/*` are the output of your node.
