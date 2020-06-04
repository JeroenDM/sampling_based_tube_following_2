
# Instructions

## Running the simulations
The folder `ros-packages` contains a snapshot of all packages used to create the simuation results.
Using [docker](https://www.docker.com/) you can build a docker image to run the simulations in.
First go to the directory where the `Dockerfile` is located (this directory), and build the image.
```bash
docker build -t ros-demo .
```

This can take a while and will download many files. Now you should be able to run the image, and build the ros workspace.
```bash
docker run -it ros-demo /bin/bash
(then inside the container, indicated by a `#`)
# cd /home/catkin_ws
# catkin build
# source devel/setup.bash
# roslaunch demo_table_moveit_config demo.launch
```

(This will not launch rviz, as this is more complicated to do inside docker.)

Now run the simulations from a different terminal.
Use `docker ps` to find the id of the container, something like `c3e672e196ca`.
```bash
docker exec -it <id> /bin/bash
# cd /home/catkin_ws
# source devel/setup.bash
# roslaunch arf_demo demo_table.launch
```

This wil run for a while and write the results to `/home/catkin_ws/src/arf/arf_demo/data`.
To stop the simulation, press `ctrl-c` in the terminal that is running the moveit demo.launch.
Then the simulation will also stop automatically.

## Using a different cost function
There are two cost functions available in arf, `L1NormCost` and `sumSquaredCost`.
To change the one used in the graph search, you have to edit line 122 in `/arf/arf_graph/src/graph.cpp`.

Either use the path length cost function (L1-norm):
```c++
// update neighbors distance
float dist = (*current).shortest_distance + L1NormCost(*nb, *current);
```
or the sum squared cost function.
```c++
// update neighbors distance
float dist = (*current).shortest_distance + sumSquaredCost(*nb, *current);
```
