#!/usr/bin/env bash
# -*- mode: julia -*-
#=
exec julia -O0 --project=/home/colinxs/workspace/mushr_workspace/src/MuSHRSLAM "${BASH_SOURCE[0]}" "$@"
=#
#__precompile__(false)
module ROSWrapper

#TODO(cxs): fix above hard code of project
# TODO(cxs): this could be fragile
#Pkg.activate(normpath(joinpath(@__DIR__, "..")))
#println(Base.current_project())
# TODO:(cxs) add dynamic reconfigure
using RoboLib.Util: binarize, subsample, img2grid
using CoordinateTransformations, Rotations
using RoboLib.Geom: Pose2D
#using Itertools
using LinearAlgebra
using StaticArrays
using RobotOS
using PyCall
using Images, ImageView
using Plots
using Random
using Rotations
using MuSHRSLAM.ParticleFilter: process_map, setup_pf, update_act!, update_obs!, reset!, resample!

# ROS Imports
@rosimport vesc_msgs.msg: VescStateStamped
@rosimport sensor_msgs.msg: LaserScan
@rosimport nav_msgs.msg: Odometry
@rosimport nav_msgs.srv: GetMap
@rosimport geometry_msgs.msg: PoseStamped, PoseArray, PoseWithCovarianceStamped, PointStamped, Pose, Quaternion
@rosimport std_msgs.msg: Header
@rosimport ackermann_msgs.msg: AckermannDriveStamped

rostypegen(@__MODULE__)
using .vesc_msgs.msg
using .sensor_msgs.msg
using .nav_msgs.msg
using .nav_msgs.srv
using .geometry_msgs.msg
using .std_msgs.msg: Header
using .ackermann_msgs.msg

#TODO(cxs): const declar? Can't do, but shouldn't be global

macro debugtask(ex)
  quote
    try
      $(esc(ex))
    catch e
      bt = stacktrace(catch_backtrace())
      io = IOBuffer()
      showerror(io, e, bt)
      errstr = String(take!(io))
      RobotOS.logfatal("Error: $errstr")
      exit()
    end
  end
end
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

mutable struct ROSPF{C, P, POPUB, PAPUB, ODPUB, T, LP, SP}
    conf::C
    pf::P

    pose_pub::POPUB
    particle_pub::PAPUB
    odom_pub::ODPUB

    T_MW::T
    T_WM::T

    # scratch
    laserposepub::LP
    scratchposepub::SP

    last_laser::Float64
    last_acker::Float64

    _rand_part_idxs::Vector{Int}
end

function ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposepub)
    println(scratchposepub)
    ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposepub, -1.0, -1.0, zeros(Int, conf[:rosconf][:max_viz_particles]))
end

#function transform(parts, trans)
#    #println(trans)
#    #println()
#    for i in eachindex(parts)
#        #println(parts[i].statev)
#        parts[i] = trans ∘ parts[i]
#        #println(parts[i].statev)
#    end
#    return parts
#end

function laser_cb(msg::LaserScan, rospf::ROSPF)
    #@debugtask begin
    #try
    #try
    # TODO(cxs): exclude max range rays
    headings = LinRange(msg.angle_min, msg.angle_max, length(msg.ranges[1:6:end]))
    ranges = (isnan(r) ? rospf.conf[:pfconf][:reweightconf][:max_range_meters] : r for r in msg.ranges[1:6:end])

    #TODO(cxs) timestamp
    dt=0.1

    #TODO(cxs): make efficent


    #println("SCAN")
    #println([p.statev for p in rospf.pf.belief.particles])
    update_obs!(rospf.pf, (headings, ranges), dt)
    resample!(rospf.pf)
    #println([p.statev for p in rospf.pf.belief.particles])
    #println()
    #println()

    particles = PoseArray()
    function makepose(particle)
        po = Pose()
        po.position.x = particle.x
        po.position.y = particle.y

        q=particle.quat
        po.orientation = Quaternion(q.x, q.y, q.z, q.w)
        return po
    end
    #poses = [makepose(p) for p in rospf.pf.belief.particles]
    rand!(rospf._rand_part_idxs, 1:length(rospf.conf[:rosconf][:max_viz_particles]))
    particles.poses = [makepose(p) for p in rospf.pf.belief.particles[rospf._rand_part_idxs]]
    particles.header = make_header("map")
    publish(rospf.particle_pub, particles)
    #println(4)
    ## scratch, pub laser endpoint poses
    #particles = PoseArray()
    ##transform(rospf.pf.reweight!.scratchposes, rospf.T_WM)
    #poses = [makepose(p) for p in rospf.pf.reweight!.scratchposes]
    #header = make_header("map")
    #particles = PoseArray()
    #particles.poses = poses
    #particles.header = header
    #publish(rospf.laserposepub, particles)


    #particles = PoseArray()
    #poses = [makepose(p) for p in rospf.pf.reweight!.m.scratchhitposes]
    #header = make_header("map")
    #particles = PoseArray()
    #particles.poses = poses
    #particles.header = header
    #publish(rospf.laserposepub, particles)
    #yield()
    #sleep(2)
    #catch e
      #println("Errr $e")
      #RobotOS.loginfo("ERRRRRR $e")
      #RobotOS.loginfo("Error: $e\n")
      #RobotOS.loginfo("Error: $e\n$bt")
      #RobotOS.logerr("Error: $e\n$bt")
      #showerror(Base.stderr, e, bt)
      #exit()
    #catch e
    #  bt = stacktrace(catch_backtrace())
    #  io = IOBuffer()
    #  showerror(io, e, bt)
    #  println("SCANER")
    #  showerror(Base.stderr, e, bt)
    #  errstr = String(take!(io))
    #  RobotOS.logfatal("Error: $errstr")
    #  exit()
    #end


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
    try
    #self.state_lock.acquire()
    println("click?")
    pose = msg.pose.pose
    pos = pose.position
    ori = pose.orientation
    theta = RotXYZ(Quat(ori.w, ori.x, ori.y, ori.z)).theta3
    q = Quat(ori.w, ori.x, ori.y, ori.z)
    t = SVector{3}(pos.x, pos.y, pos.z)
    pose2d = Pose2D(q, t)
    println("CLICK ", (pos.x, pos.y, theta))
    reset!(rospf.pf, pose2d)
    #println("DONERESET")


    catch e
      bt = stacktrace(catch_backtrace())
      io = IOBuffer()
      showerror(io, e, bt)
      println("CLICKER")
      errstr = String(take!(io))
      RobotOS.logfatal("Error: $errstr")
      exit()
    end
    #self.weights[:] = 1.0 / float(self.particles.shape[0])
    #self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.particles.shape[0])
    #self.particles[:,2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.particles.shape[0])
    #self.state_lock.release()
    #end
end

function ackermann_cb(msg, rospf::ROSPF)
    #try
    h = msg.header
    d = msg.drive
    trecv = time()
    if rospf.last_acker == -1
        rospf.last_acker = trecv
    else
        dt = trecv - rospf.last_acker
        rospf.last_acker = trecv

        ctrl = SVector(d.speed, d.steering_angle)
        update_act!(rospf.pf, ctrl, dt)
    end
    #catch e
    #  bt = stacktrace(catch_backtrace())
    #  io = IOBuffer()
    #  showerror(io, e, bt)
    #  errstr = String(take!(io))
    #  RobotOS.logfatal("Error: $errstr")
    #  exit()
    #@end
end

#TODO(cxs): nested params
#TODO(cxs): handle array/dict params
function get_params(d::Dict{Symbol, Any})
    for (param_name, value) in d
        if isa(value, Dict)
            get_params(value)
        elseif isa(value, Union{Number, String})
            new_value = get_param('~' * string(param_name), value)
            new_value = oftype(value, new_value)
            if new_value != value
                loginfo("Found param \"$param_name\". Default value: \"$value\", new value: \"$new_value\"")
                d[param_name] = new_value
            else
                loginfo("Param \"$param_name\" not set. Using default value: \"$value\"")
            end
        end
    end
end

function process_ros_map(map_msg, config)
    mc = Dict{Symbol,Any}()
    info = map_msg.map.info
    h = info.height
    w = info.width
    r = info.resolution
    p = info.origin.position
    q = info.origin.orientation
    mc[:height] = h
    mc[:width] = w
    mc[:scale_WM] = r
    mc[:p_xyz] = p
    mc[:q_xyzw] = q

    # something something row major vs column major
    map = reshape(map_msg.map.data', (mc[:width], mc[:height]))'
    # create a boolean matrix and closure where {map[x,y]=true, mapoccf(x,y)=true} --> (x,y) is unoccupied
    # assumes map has following format: -1=>unknown, 0=>free space, >0=>occupied (potential with probability)
    map, mapoccf = process_map(map, (el)->el!=0)

    # a scaling matrix
    sc=SMatrix{3,3}(r,0,0,0,r,0,0,0,r)
    # represents the transformation matrix from map frame to world frame
    # i.e. T_WP = T_WM ∘ T_MP and p_W = T_WM(p_M)
    # where p_W is a vector
    T_WM = Translation(p.x, p.y, p.z) ∘ LinearMap(Quat(q.w, q.x, q.y, q.z)) ∘ LinearMap(sc)
    T_MW = inv(T_WM)

    return map, mapoccf, mc, T_MW, T_WM
end

function main()

    BLAS.set_num_threads(4)
    reweightconf = Dict{Symbol,Any}(:type=>:beammodel, :raymethod=>:bresenham, :max_range_meters=>20, :squash_factor=>1, :nrays=>100, :a_short=>0.01, :a_max=>0.08, :a_rand=>:0.1, :a_hit=>0.75, :sigma_hit=>8.0, :lambda_short=>1/2.0)
    predictconf = Dict{Symbol,Any}(:type=>:ackermann, :sigma_x=>0.05, :sigma_y=>0.025, :sigma_theta=>0.03, :init_sigma_x=>0.001, :init_sigma_y=>0.001, :init_sigma_theta=>0.0001, :vel_sigma=>0.03, :delta_sigma=>0.1, :car_length=>0.33, :stochastic=>true)
    pfconf=Dict{Symbol,Any}(:max_particles=>1000, :predictconf=>predictconf, :reweightconf=>reweightconf)
    rosconf = Dict{Symbol,Any}(:max_viz_particles=>50, :static_map_service=>"/static_map", :ackermann_topic=>"/vesc/low_level/ackermann_cmd_mux/output")
    conf = Dict{Symbol,Any}(:rosconf=>rosconf, :pfconf=>pfconf)

    init_node("particle_filter", anonymous=true)
    c = conf
    get_params(conf)
    pfconf[:max_particles] = 100
    rosconf[:max_viz_particles] = 50

    loginfo("Waiting for map service...")
    wait_for_service(rosconf[:static_map_service])
    map_msg = ServiceProxy(rosconf[:static_map_service], GetMap)(GetMapRequest())
    map, mapoccf, mapconf, T_MW, T_WM = process_ros_map(map_msg, conf)
    reweightconf[:scale_WM] = mapconf[:scale_WM]
    pfconf[:dtype] = Float64
    reweightconf[:max_range_meters]=5.6

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
    pfconf[:rng] = rng

    pf = setup_pf(mapoccf, T_MW, pfconf)
    rospf = ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposes)

    laser_sub = Subscriber(get_param("~scan_topic", "/scan"), LaserScan, laser_cb, (rospf,), queue_size=1)
    ackermann_sub = Subscriber(rosconf[:ackermann_topic], AckermannDriveStamped, ackermann_cb, (rospf,), queue_size=1)
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
