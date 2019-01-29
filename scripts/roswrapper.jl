#!/usr/bin/env julia
using Pkg

# TODO(cxs): this could be fragile
Pkg.activate(normpath(joinpath(@__DIR__, "..")))

using RoboLib.Util: binarize
using RoboLib.Geom: Pose2D
using StaticArrays
using RobotOS
using PyCall
using Random
using MuSHRSLAM.ParticleFilter: process_map, setup_pf, setup_raycast, setup_motion_model, update!, update_action!, update_obs!

# ROS Imports
@rosimport vesc_msgs.msg: VescStateStamped
@rosimport sensor_msgs.msg: LaserScan
@rosimport nav_msgs.msg: Odometry
@rosimport nav_msgs.srv: GetMap
@rosimport geometry_msgs.msg: PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped

rostypegen()
using .vesc_msgs.msg
using .sensor_msgs.msg
using .nav_msgs.msg
using .nav_msgs.srv
using .geometry_msgs.msg

@pyimport tf

# ----

## subsample for plotting particles
function subsample(parts)
    return [SVector{2}(parts[round(Int, i)][1:2]) for i in LinRange(1, length(parts), 100)]
end

beamconfig = Dict(:resolution=>0.5, :a_short=>0.01, :a_max=>0.08, :a_rand=>:0.1, :a_hit=>0.75, :sigma_hit=>8.0, :lambda_short=>1/2.0)
mapconfig = Dict(:threshhold=>1.0)
laserconfig = Dict(:maxrange=>300, :beammodel=>beamconfig, :squash_factor=>0.3, :fov=>pi/3, :nrays=>50)
sensorconfig = Dict(:beamconfig=>beamconfig, :laserconfig=>laserconfig)

motionconfig = Dict(:sigma_x=>5, :sigma_y=>5, :sigma_theta=>deg2rad(0.15), :init_sigma_x=>15, :init_sigma_y=>15, :init_sigma_theta=>0.3, :vel_sigma=>0.01, :delta_sigma=>0.01,
                    :speed2erpm_offset=>0, :speed2erpm_gain=>1, :steering2servo_offset=>0, :steering2servo_gain=>1, :car_length=>1, :stochastic=>true)

config=Dict(:pose0=>[150,450,0], :ctrl0=>[1,0], :nparticles=>3000, :motionmodel=>:ackermann, :motionconfig=>motionconfig, :sensormodel=>:beam, :sensorconfig=>sensorconfig, :mapconfig=>mapconfig)

function laser_cb(msg::LaserScan, rospf::ROSPF)
    # TODO(cxs): exclude max range rays
    println(typeof(msg.ranges))
    println(msg.angle_min)
    println(msg.angle_max)
    	#if not isinstance(self.laser_angles, np.ndarray):
        #    #print 'Creating angles'
        #    self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        #    self.downsampled_angles = np.copy(self.laser_angles[0::self.LASER_RAY_STEP]).astype(np.float32)

        ##print 'Downsampling ranges'
        #self.downsampled_ranges = np.array(msg.ranges[::self.LASER_RAY_STEP])
        #self.downsampled_ranges[np.isnan(self.downsampled_ranges)] = self.MAX_RANGE_METERS
end

function clicked_pose_cb(msg::PoseWithCovarianceStamped, rospf::ROSPF)
    #self.state_lock.acquire()
    pose = msg.pose.pose
    println(pose)
    #self.weights[:] = 1.0 / float(self.particles.shape[0])
    #self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.particles.shape[0])
    #self.state_lock.release()
end

struct ROSPF{P, R, POPUB, PAPUB, ODPUB}
    pf::P
    resetpf!::R
    pose_pub::POPUB
    particle_pub::PAPUB
    odom_pub::ODPUB
end

function main()
    c = config

    init_node("particle_filter", anonymous=true)
    map_service_name = get_param("~static_map", "static_map")
    MAX_PARTICLES = Int(get_param("~max_particles")) # The maximum number of particles

    loginfo("Waiting for map service...")
    wait_for_service(map_service_name)
    map_msg = ServiceProxy(map_service_name, GetMap)(GetMapRequest())
    map = reshape(map_msg.map.data, (map_msg.map.info.height, map_msg.map.info.width))
    map, mapoccupiedf = process_map(map, (el)->el==0.0)

    #tfl = tf.TransformListener()

    # Publish particle filter state
    pose_pub      = Publisher("/pf/ta/viz/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    particle_pub  = Publisher("/pf/ta/viz/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    #pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization
    #pub_laser     = Publisher("/pf/ta/viz/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan
    odom_pub = Publisher("/pf/ta/viz/odom", Odometry, queue_size = 1) # Publishes the path of the car



    # setup PF
    rng = Random.MersenneTwister(1234)
    config[:rng] = rng

    config[:pose0] = float(config[:pose0])
    config[:ctrl0] = float(config[:ctrl0])

    pose0 = Pose2D(c[:pose0])
    ctrl0 = SVector{2, Float64}(c[:ctrl0])

    pf, resetpf! = setup_pf(map, mapoccupiedf, config)
    rospf = ROSPF(pf, resetpf!, pose_pub, particle_pub, odom_pub)

    laser_sub = Subscriber(get_param("~scan_topic", "/scan"), LaserScan, laser_cb, (rospf,), queue_size=1)
    # no motion sub for now (motion model should live here, and vesc odom is broken)
    #self.motion_sub = Subscriber(get_param("~motion_topic", "/vesc/odom"), Odometry, self.motion_model.motion_cb, queue_size=1)
    pose_sub  = Subscriber("/initialpose", PoseWithCovarianceStamped, clicked_pose_cb, (rospf,), queue_size=1)
    loginfo("Initialization complete")


    spin()
end
#
if ! isinteractive()
    main()
end
