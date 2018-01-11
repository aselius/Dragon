# Carla Self Driving Car System Integration Project
## Team Dragon

### Running
1. Open 2 different bash shells
First in separate bash windows do the following. (If you have put source devel/setup.bash in your startup script you can ignore second step)
<br/>
$ cd Dragon/ros
<br/>
$ source devel/setup.bash

Now in strict order do following in every window.

1. $ roslaunch launch/styx.launch
5. $ roslaunch tl_detector tl_detector.launch




### Installation
It is probably easiest to install ROS and deploy everything with Docker, and although you could use the VM provided by Udacity [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/Udacity_VM_Base_V1.0.0.zip), it is recommended to deploy using Docker.

If you do not have Docker, feel free to install Docker for your host OS.
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

Source the ROS env variables
```bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
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

> Note that for every new bash session if you are SSH-ing into the Docker container, you need to source devel/setup.py inorder to be able to roslaunch on the command line. Everytime it is specified that you should open a new session, docker exec command and the source devl/setup.sh mentioned below needs to be executed and will be omited in the bash code snippet after the first block.

```bash
docker ps
```
You can choose to grep the container ID or name and pipe it into the exec command directly or just use the clipboard to copy and paste
```bash
docker exec -it <container name or id> /bin/bash
cd /Dragon/ros && source devel/setup.sh
roslaunch waypoint_loader waypoint_loader.launch
```
New bash session in Docker container
```bash
roslaunch styx server.launch
```

