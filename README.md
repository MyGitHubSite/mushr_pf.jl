# mushr_pf: Particle Filter

### Install  
**Note:** If you are using a MuSHR image, you can skip steps 1 & 5
1. Install Julia, there are 2 ways to do it:
   - `bash -ci "$(curl -fsSL https://raw.githubusercontent.com/colinxs/jill/feat/detect_arch/jill.sh)"`  
   - To install, follow the directions [here](https://julialang.org/downloads/). There's nothing to `sudo apt-get`, just a binary that you download and add to your `PATH` environment variable.  


2. Clone this repo into your ROS workspace: `cd ~/catkin_ws/src && git clone https://github.com/prl-mushr/mushr_pf.git`  

3. The library used to interoperate with Python
    ([PyCall](https://github.com/JuliaPy/PyCall.jl)) needs to be linked
    to the same Python binary that you use with ROS. By default this is
    the result of executing `which python` on the command line. This can
    be an issue if you perform the next step with an Anaconda environment
    activated. To avoid this run:  
	`export PYTHON=$(which python) > ~/.bashrc` and re-source your workspace `. ~/.bashrc`  
    If you ever want to switch python versions, just change the PYTHON variable & re-run `julia -e 'using Pkg; Pkg.build("PyCall")`.  

4. From the root directory of the repo run
`cd ./deps && ./jetson_install.sh`. The script will prompt you for
`sudo` permissions, but do not run the script itself with `sudo`. You
may see an error about the build for Arpack failing; don't worry, that's what
the patch is for :). This builds all necessary dependencies listed in Project.toml & Manifest.toml.  

5. If not installed already, install the `sensor_msgs`, `geometry_msgs`,
`nav_msgs`, `vesc_msgs`, and `ackermann_msgs` packages. The MuSHR images all have these installed.  

6. `cd ~/catkin_ws/ && catkin make`   
   (although like Python, there is nothing to build, this step just
   sets up your environment variables) and run `source ~/catkin_ws/devel/setup.bash`  

7. Run `roslaunch mushr_pf ParticleFilter.launch`  
8. The node will compile and wait until an map is provided and a initial position is specified.  

One thing that you may be curious about is that the startup time is
slow relative to Python. This is because Julia is a just-in-time or
JIT compiled language. Compilation is not performed until a piece
of code (i.e. a function) is needed. Note that if you want to hack at
this codebase there are ways to get around this
(i.e. [Revise.jl](https://github.com/timholy/Revise.jl)). Additionally,
you can change the optimization level from `-O3` to `-O0` under the
args section of `ParticleFilter.launch`, although
this will negatively affect runtime performance.

### Configuration
All settings are located in the `scripts/settings.jl` file as key-value
pairs. You can change the settings here, or via the ROS parameter server.
To see which parameters are configurable via the parameter server,
you can use `rosparam list` or `rosparam dump` after launching
the node. Additionally, the node will
log to `/rosout` and the terminal all the settings which are configurable
via the parameter server, whether a parameter was found on the server,
and what its value is as follows:
```
 [INFO] [1549348704.911341]: Param "steering2servo_gain" not set. Using default value: "-1.2135"
```
Future versions will have these parameters documented in a ROS config file.

**NOTE:** The one exception is multi-threading which is not configurable at runtime.
Instead, set the environment variable `JULIA_NUM_THREADS` to the integer
number of threads you would like to use. The default is either the number of
physical CPU cores on your machine, or one (I've seen both :)).

To aid in development, please kindly file a `git issue` if you run across
any problems or want something added. Happy localizing!

### API
Parameters can be changed in `settings.yaml`
#### Publishers
Topic | Type | Description
------|------|------------
`/pf/inferred_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) | Particle filter pose estimate
`/pf/viz/particles` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Partilcle array. Good for debugging
`/pf/viz/laserpose` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Pose fo the laser
`/odom` | [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)| Odom of car estimate from particle filter

#### Subscribers
Topic | Type | Description
------|------|------------
`/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Current laserscan
`/vesc/sensors/servo_position_command` | [std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) | Current steering angle
`/vesc/sensors/core` | [vesc_msgs/VescStateStamped](https://github.com/prl-mushr/vesc/blob/master/vesc_msgs/msg/VescStateStamped.msg)| Current speed
