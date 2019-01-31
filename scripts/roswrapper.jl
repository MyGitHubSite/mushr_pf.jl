#!/usr/bin/env bash
# -*- mode: julia -*-
#=
exec julia -O0 --project=/home/colinxs/workspace/mushr_workspace/src/MuSHRSLAM "${BASH_SOURCE[0]}" "$@"
=#
__precompile__(false)
module ROSWrapper

# TODO(cxs): this could be fragile
#Pkg.activate(normpath(joinpath(@__DIR__, "..")))
#println(Base.current_project())

using RoboLib.Util: binarize, subsample, @debugtask, to_grid
using CoordinateTransformations, Rotations
using RoboLib.Geom: Pose2D
#using Itertools
using StaticArrays
using RobotOS
using PyCall
using Images, ImageView
using Plots
using Random
using Rotations
using MuSHRSLAM.ParticleFilter: process_map, setup_pf, setup_raycast, setup_motion_model, update!, update_action!, update_obs!

# ROS Imports
@rosimport vesc_msgs.msg: VescStateStamped
@rosimport sensor_msgs.msg: LaserScan
@rosimport nav_msgs.msg: Odometry
@rosimport nav_msgs.srv: GetMap
@rosimport geometry_msgs.msg: PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped, Pose, Quaternion
@rosimport std_msgs.msg: Header

rostypegen(@__MODULE__)
using .vesc_msgs.msg
using .sensor_msgs.msg
using .nav_msgs.msg
using .nav_msgs.srv
using .geometry_msgs.msg
using .std_msgs.msg: Header


# ----

#TODO(cxs): figure out how to get_param with default value array (currently complains)

function make_header(frame_id, stamp=nothing)
    if isnothing(stamp)
        stamp = get_rostime()
    end
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
end

struct ROSPF{C, P, R, POPUB, PAPUB, ODPUB, M1, M2, LP}
    config::C
    pf::P
    resetpf!::R
    pose_pub::POPUB
    particle_pub::PAPUB
    odom_pub::ODPUB
    T_MW::M1
    T_WM::M2
    laserposepub::LP
    scratchposepub

    #last_laser::Float64
    #last_motion::Float64
end

function transform(parts, trans)
    #println(trans)
    #println()
    for i in eachindex(parts)
        #println(parts[i].statev)
        parts[i] = trans ∘ parts[i]
        #println(parts[i].statev)
    end
    return parts
end

function laser_cb(msg::LaserScan, rospf::ROSPF)
    @debugtask begin
    # TODO(cxs): exclude max range rays
    headings = LinRange(msg.angle_min, msg.angle_max, length(msg.ranges[1:5]))
    ranges = (isnan(r) ? rospf.config[:pfconfig][:sensorconfig][:laserconfig][:max_range_meters] : r for r in msg.ranges[1:5])

    dt=0.1

    #TODO(cxs): make efficent

    println(1)

    #println("SCAN")
    #println(rospf.T_MW)
    #println([p.statev for p in rospf.pf.belief.particles])
    transform(rospf.pf.belief.particles, rospf.T_MW)
    println(2)
    #println([p.statev for p in rospf.pf.belief.particles])
    update_obs!(rospf.pf, (headings, ranges), dt)
    #println([p.statev for p in rospf.pf.belief.particles])
    #println("TRANS2")
    transform(rospf.pf.belief.particles, inv(rospf.T_MW))
    #println([p.statev for p in rospf.pf.belief.particles])
    #println()
    #println()
    println(3)

    particles = PoseArray()
    function makepose(particle)
        po = Pose()
        po.position.x = particle.x
        po.position.y = particle.y

        q=particle.quat
        po.orientation = Quaternion(q.x, q.y, q.z, q.w)
        return po
    end
    poses = [makepose(p) for p in rospf.pf.belief.particles]
    header = make_header("map")
    particles.poses = poses
    particles.header = header
    publish(rospf.particle_pub, particles)
    println(4)
    # scratch, pub laser endpoint poses
    particles = PoseArray()
    transform(rospf.pf.reweight_model.scratchposes, rospf.T_WM)
    poses = [makepose(p) for p in rospf.pf.reweight_model.scratchposes]
    header = make_header("map")
    particles = PoseArray()
    particles.poses = poses
    particles.header = header
    publish(rospf.laserposepub, particles)


    particles = PoseArray()
    println([p.statev for p in rospf.pf.reweight_model.scratchhitposes])
    transform(rospf.pf.reweight_model.scratchhitposes, rospf.T_WM)
    poses = [makepose(p) for p in rospf.pf.reweight_model.scratchhitposes]
    header = make_header("map")
    particles = PoseArray()
    particles.poses = poses
    particles.header = header
    publish(rospf.scratchposepub, particles)
    #yield()
    #sleep(2)
    end


    #TODO(cxs): subsample, maxrange

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
    pos = pose.position
    ori = pose.orientation
    theta = RotXYZ(Quat(ori.w, ori.x, ori.y, ori.z)).theta3
    q = Quat(ori.w, ori.x, ori.y, ori.z)
    t = SVector{3}(pos.x, pos.y, pos.z)
    pose2d = Pose2D(q, t)
    println("CLICK ", (pos.x, pos.y, theta))
    rospf.resetpf!(pose2d)
    println("DONERESET")
    #self.weights[:] = 1.0 / float(self.particles.shape[0])
    #self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.particles.shape[0])
    #self.state_lock.release()
end

beamconfig = Dict{Symbol,Any}(:a_short=>0.01, :a_max=>0.08, :a_rand=>:0.1, :a_hit=>0.75, :sigma_hit=>8.0, :lambda_short=>1/2.0)
mapconfig = Dict{Symbol,Any}(:threshhold=>1.0)
laserconfig = Dict{Symbol,Any}(:max_range_meters=>300, :beammodel=>beamconfig, :squash_factor=>0, :laser_ray_step=>1)
sensorconfig = Dict{Symbol,Any}(:beamconfig=>beamconfig, :laserconfig=>laserconfig)

motionconfig = Dict{Symbol,Any}(:sigma_x=>0.05, :sigma_y=>0.025, :sigma_theta=>0, :init_sigma_x=>0.5, :init_sigma_y=>0.5, :init_sigma_theta=>0.0, :vel_sigma=>0.03, :delta_sigma=>0.1,
                    :speed2erpm_offset=>0, :speed2erpm_gain=>1, :steering2servo_offset=>0, :steering2servo_gain=>1, :car_length=>1, :stochastic=>true)

pfconfig=Dict{Symbol,Any}(:max_particles=>3000, :motionmodel=>:ackermann, :motionconfig=>motionconfig, :sensormodel=>:beam, :sensorconfig=>sensorconfig, :mapconfig=>mapconfig, :dtype=>Float64)
rosconfig=Dict{Symbol,Any}(:max_viz_particles=>50, :static_map=>"static_map", :ackermann_topic=>"/vesc/low_level/ackermann_cmd_mux/output")
config=Dict{Symbol,Any}(:rosconfig=>rosconfig, :pfconfig=>pfconfig)

#TODO(cxs): check for all params in private name space and report those that didn't match f
function get_params(d::Dict{Symbol, Any})
    for (param_name, value) in d
        if isa(value, Dict)
            get_params(value)
        elseif isa(value, Union{Number, String})
            new_value = get_param('~' * string(param_name), value)
            new_value = oftype(value, new_value)
            if new_value != value
                loginfo("Found param $param_name. Default value: $value, new value: $new_value")
                d[param_name] = new_value
            else
                loginfo("Could not find param $param_name. Using default value: $value")
            end
        else
            loginfo("Current don't know how to handle param $param_name of type $(typeof(value))")
        end
    end
end

#function ackermann_cb(msg::AckerMann)

function process_ros_map(map_msg)
    info = map_msg.map.info
    h = info.height
    w = info.width
    r = info.resolution
    p = info.origin.position
    q = info.origin.orientation

    map = reshape(map_msg.map.data', (w, h))'
    map, mapoccupiedf = process_map(map, (el)->el!=0.0)

    sc=SMatrix{3,3}(r,0,0,0,r,0,0,0,r)
    T_WM = Translation(p.x, p.y, p.z) ∘ LinearMap(Quat(q.w, q.x, q.y, q.z)) ∘ LinearMap(sc)
    T_MW = inv(T_WM)
    println(T_WM)

    return map, mapoccupiedf, T_MW, T_WM
end

function main()
    c = config

    init_node("particle_filter", anonymous=true)
    get_params(config)

    # get all params
    #map_service_name = get_param("~static_map", "static_map")
    #self.LASER_RAY_STEP = int(rospy.get_param("~laser_ray_step")) # Step for downsampling laser scans
    #self.EXCLUDE_MAX_RANGE_RAYS = bool(rospy.get_param("~exclude_max_range_rays")) # Whether to exclude rays that are beyond the max range
    #self.MAX_RANGE_METERS = float(rospy.get_param("~max_range_meters")) # The max range of the laser
    #self.MAX_VIZ_PARTICLES = int(rospy.g self.SPEED_TO_ERPM_OFFSET = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0)) # Offset conversion param from rpm to speed

    # this shouldn't be in motion model, just use ackermann msg
    #self.SPEED_TO_ERPM_GAIN   = float(rospy.get_param("/vesc/speed_to_erpm_gain",4614.0 ))   # Gain conversion param from rpm to speed
    #self.STEERING_TO_SERVO_OFFSET = float(rospy.get_param("/vesc/steering_angle_to_servo_offset", 0.5304)) # Offset conversion param from servo position to steering angle
    #self.STEERING_TO_SERVO_GAIN   = float(rospy.get_param("/vesc/steering_angle_to_servo_gain",-1.2135)) # Gain conversion param from servo position to steering angle
    #et_param("~max_viz_particles")) # The maximum number of particles to visualize



    loginfo("Waiting for map service...")
    wait_for_service(rosconfig[:static_map])
    map_msg = ServiceProxy(rosconfig[:static_map], GetMap)(GetMapRequest())
    map, mapoccupiedf, T_MW, T_WM = process_ros_map(map_msg)
    #imshow(map)
    #sleep(10)
    pfconfig[:max_particles] = 10
    beamconfig[:resolution] = map_msg.map.info.resolution
    laserconfig[:max_range_px] = laserconfig[:max_range_meters] / beamconfig[:resolution]

    #tfl = tf.TransformListener()

    # Publish particle filter state
    pose_pub      = Publisher("/pf/ta/viz/inferred_pose", PoseStamped, queue_size = 1) # Publishes the expected pose
    particle_pub  = Publisher("/pf/ta/viz/particles", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    laserposepub = Publisher("/pf/ta/viz/laserposepub", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    scratchposes = Publisher("/pf/ta/viz/scratchposes", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    #pub_tf = tf.TransformBroadcaster() # Used to create a tf between the map and the laser for visualization
    #pub_laser     = Publisher("/pf/ta/viz/scan", LaserScan, queue_size = 1) # Publishes the most recent laser scan
    odom_pub = Publisher("/pf/ta/viz/odom", Odometry, queue_size = 1) # Publishes the path of the car


    # setup PF
    rng = Random.MersenneTwister(1234)
    pfconfig[:rng] = rng

    pf, resetpf! = setup_pf(map, mapoccupiedf, pfconfig)
    rospf = ROSPF(config, pf, resetpf!, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposes)

    laser_sub = Subscriber(get_param("~scan_topic", "/scan"), LaserScan, laser_cb, (rospf,), queue_size=1)
    #ackermann_sub = Subscriber(rosconfig[:ackermann_topic], LaserScan, ackermann_cb, (rospf,), queue_size=1)
    # no motion sub for now (motion model should live here, and vesc odom is broken)
    #self.motion_sub = Subscriber(get_param("~motion_topic", "/vesc/odom"), Odometry, self.motion_model.motion_cb, queue_size=1)
    pose_sub  = Subscriber("/initialpose", PoseWithCovarianceStamped, clicked_pose_cb, (rospf,), queue_size=1)
    loginfo("Initialization complete")


    spin()
end

end # module
using .ROSWrapper: main
if ! isinteractive()
    main()
end
