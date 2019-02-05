# MuSHRSLAM: Helping you not SLAM into things


## Install Instructions
1. This library is built using [Julia](https://julialang.org/downloads/).
   To install, follow the directions [here](https://julialang.org/downloads/). There's nothing to `sudo apt-get`, just a binary that you download and add to your `PATH`.
   [This](https://github.com/abelsiqueira/jill/blob/master/jill.sh) script
   can help automate that process for you.
2. Clone the repo into your ROS workspace.
3. In the root of the repo, start the Julia REPL with
`julia --project`. Type the `]` character to enter "Pkg" mode and
run `instantiate`. Julia contains a built in package manager, and
this command will install all the needed dependencies as defined by
the `Project.toml` and `Manifest.toml` files in the root directory.
By doing so, you will exactly replicate the development environment.
4. Exit out of the Julia REPL and run `catkin build`
   (although like Python, there is nothing to build, this step just
   sets up your environment variables).
5. Run `roslaunch MuSHRSLAM ParticleFilter.launch`
6. The node will wait until an initial position is specified.

One thing that you may be curious about is that the startup time is
slow relative to Python. This is because Julia is a just-in-time or
JIT compiled language. Compilation is not performed until a piece
of code (i.e. a function) is needed. Note that if you want to hack at
this codebase there are ways to get around this
(i.e. [Revise.jl](https://github.com/timholy/Revise.jl)).

## Configuration
All settings are located in the `scripts/settings.jl` file as key-value
pairs. You can change the settings here, or via the ROS parameter server.
To see which parameters are configurable via the parameter server,
you can use `rosparam list` or `rosparam dump`. Additionally, the node will
log to `/rosout` and the terminal all the settings which are configurable
via the parameter server, whether a parameter was found on the server,
and what its value is as follows:
```
 [INFO] [1549348704.911341]: Param "steering2servo_gain" not set. Using default value: "-1.2135"
```
Future versions will have these parameters documented in a ROS config file.

_NOTE:_ The one exception is multi-threading which is not configurable at runtime.
Instead, set the environment variable `JULIA_NUM_THREADS` to the integer
number of threads you would like to use. The default is the number of
physical CPU cores on your machine.

To aid in development, please kindly file a `git issue` if you run across
any problems or want something added. Happy localizing!