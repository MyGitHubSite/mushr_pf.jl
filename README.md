# MuSHRSLAM: Helping you not SLAM into things


## Install Instructions
1. This library is built using [Julia](https://julialang.org/downloads/).
   To install, follow the directions [here](https://julialang.org/downloads/). There's nothing to `sudo apt-get`, just a binary that you download and add to your `PATH` environment variable.
   [This](https://raw.githubusercontent.com/colinxs/jill/feat/detect_arch/jill.sh) script
   can help automate that process for you. To use it, just run

   ```bash -ci "$(curl -fsSL https://raw.githubusercontent.com/colinxs/jill/feat/detect_arch/jill.sh)"```

2. Clone the repo into your ROS workspace.
3. The library used to interoperate with Python
    ([PyCall](https://github.com/JuliaPy/PyCall.jl)) needs to be linked
    to the same Python binary that you use with ROS. By default this is
    the result of executing `which python` on the command line. This can
    be an issue if you perform the next step with an Anaconda environment
    activated. Either make sure that `which python` is what you use
    with ROS, or set the `PYTHON` environment variable to the correct
    verison (_e.g._ `export PYTHON=/path/to/python`). If you ever want to
    switch versions, just re-run `julia -e 'using Pkg; Pkg.build("PyCall")`.
3. In the root of the repo, start the Julia REPL with
    `julia --project`. Type the `]` character to enter "Pkg" mode and
    run `instantiate`. Julia contains a built in package manager, and
    this command will install all the needed dependencies as defined by
    the `Project.toml` and `Manifest.toml` files in the root directory.
    By doing so, you will exactly replicate the development environment.

    Alternatively, you can run `julia --project -e 'using Pkg; Pkg.instantiate(); Pkg.build(); exit()`
    from the root of the cloned repo as a shortcut.

    **NOTE:** If you are installing on the Jetson TX2, a patch is required
for one of the dependencies. From the root directory of the repo run
`cd ./deps && ./jetson_install.sh`. The script will prompt you for
`sudo` permissions, but do not run the script itself with `sudo`. You
may see an error about the build for Arpack failing; don't worry, that's what
the patch is for :).

4. If not installed already, install the `sensor_msgs`, `geometry_msgs`,
`nav_msgs`, `vesc_msgs`, and `ackermann_msgs` packages
(see [mushr_sim](git@github.com:personalrobotics/mushr_sim.git) for more details).
5. Run `catkin build`
   (although like Python, there is nothing to build, this step just
   sets up your environment variables) and run `source /path/to/mushr/workspace/devel/setup.{sh, bash, zsh}` depending on
   your command line environment.
6. Configure the ROS topic and service names appropriately (see below).
7. Run `roslaunch MuSHRSLAM ParticleFilter.launch`
8. The node will wait until an initial position is specified.

One thing that you may be curious about is that the startup time is
slow relative to Python. This is because Julia is a just-in-time or
JIT compiled language. Compilation is not performed until a piece
of code (i.e. a function) is needed. Note that if you want to hack at
this codebase there are ways to get around this
(i.e. [Revise.jl](https://github.com/timholy/Revise.jl)). Additionally,
you can change the optimization level from `-O3` to `-O0` under the
args section of `ParticleFilter.launch`, although
this will negatively affect runtime performance.

## Configuration
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
