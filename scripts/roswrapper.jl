#!/usr/bin/env bash
# -*- mode: julia -*-
#=
exec julia -O3 --project=/home/colinxs/workspace/mushr_workspace/src/MuSHRSLAM "${BASH_SOURCE[0]}" "$@"
=#
#__precompile__(false)
module ROSWrapper

#TODO(cxs): fix above hard code of project
# TODO(cxs): this could be fragile
#Pkg.activate(normpath(joinpath(@__DIR__, "..")))
#println(Base.current_project())
# TODO:(cxs) add dynamic reconfigure
#TODO strongly and consistently type (esp w/ gpu to avoid Float64/32 conv)

using RoboLib.Util: binarize, subsampleN
using CoordinateTransformations, Rotations
using RoboLib.Geom: Pose2D, project2D, Scale2D
using StaticArrays
using LinearAlgebra
using Random
using MuSHRSLAM.ParticleFilter: setup_pf, update_act!, update_obs!, reset!, resample!, OccMap

include("settings.jl")
include("util.jl")

# ROS Imports
using RobotOS
@rosimport sensor_msgs.msg: LaserScan
@rosimport nav_msgs.msg: Odometry
@rosimport nav_msgs.srv: GetMap
@rosimport geometry_msgs.msg: PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Quaternion
@rosimport std_msgs.msg: Header
@rosimport ackermann_msgs.msg: AckermannDriveStamped

rostypegen(@__MODULE__)
using .sensor_msgs.msg
using .nav_msgs.msg
using .nav_msgs.srv
using .geometry_msgs.msg
using .std_msgs.msg: Header
using .ackermann_msgs.msg
# -----

function laser_cb(msg::LaserScan, rospf::ROSPF)
    @debugtask begin
    h = msg.header
    #TODO: stamptime (fix ta sim first) or rosstime
    trecv = time()
    if rospf.last_laser == -1
        rospf.last_laser = trecv
    else
        dt = trecv - rospf.last_laser
        rospf.last_laser = trecv

        ranges = (isnan(r) ? rospf.mr_W : r for r in subsampleN(msg.ranges, 20))
        headings = LinRange(msg.angle_min, msg.angle_max, length(ranges))

        update_obs!(rospf.pf, (headings, ranges), dt)
        resample!(rospf.pf)

        particles = makeposearray(rospf)
        particles.header = make_header("map")
        publish(rospf.particle_pub, particles)

        #lp=rospf.pf.reweightmodel.scratchhitposes
        #println([p.statev for p in lp[1:5]])
        #for i in eachindex(lp)
        #    x,y,th = lp[i].statev
        #    lp[i] = Pose2D(x/0.02,y/0.02,th)
        #end
        laserposes = makeposearray(rospf.pf.reweightmodel.scratchhitposes)
        laserposes.header = make_header("map")
        publish(rospf.laserposepub, laserposes)

        pa = rospf.pf.belief.particles
        w=rospf.pf.belief.weights
        #println((length(unique(pa)), minimum(w), maximum(w), rospf.conf[:rosconf][:max_viz_particles], length(msg.ranges)))
        #sleep(1)
    end
    #rossleep(Duration(1))
    end
end

function clicked_pose_cb(msg::PoseWithCovarianceStamped, rospf::ROSPF)
    @debugtask begin
    println("click?")
    pose = msg.pose.pose
    p = pose.position
    q = pose.orientation

    r2d = project2D(Quat(q.w, q.x, q.y, q.z))
    t2d = SVector{2}(p.x, p.y)
    reset!(rospf.pf, Pose2D(r2d, t2d))

    rospf.initialized = true
    end
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
#TODO(cxs): handle unknown
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
    mc[:height] = info.height
    mc[:width] = info.width
    mc[:scale_WM] = info.resolution
    mc[:p_xyz] = info.origin.position
    mc[:q_xyzw] = info.origin.orientation

    # something something row major vs column major
    occmap = reshape(map_msg.map.data', (mc[:width], mc[:height]))'
    occmap = OccMap(occmap, (el)->el!=0)

    # represents the transformation matrix from map frame to world frame
    # i.e. T_WP = T_WM ∘ T_MP and p_W = T_WM(p_M)
    # where p_W is a vector
    p, q = mc[:p_xyz], mc[:q_xyzw]
    T_WM = Translation(p.x, p.y) ∘ LinearMap(project2D(Quat(q.w, q.x, q.y, q.z))) ∘ Scale2D(mc[:scale_WM])
    T_MW = inv(T_WM)

    return occmap, mc, T_MW, T_WM
end

function main()
    init_node("particle_filter", anonymous=true)

    get_params(conf)

    loginfo("Waiting for map service...")
    wait_for_service(rosconf[:static_map_service])
    map_msg = ServiceProxy(rosconf[:static_map_service], GetMap)(GetMapRequest())
    occmap, mapconf, T_MW, T_WM = process_ros_map(map_msg, conf)

    #TODO: change launch defaults
    reweightconf[:scale_WM] = mapconf[:scale_WM]
    pfconf[:dtype] = Float64
    reweightconf[:max_range_meters]=5.6
    pfconf[:max_particles] = 2000
    rosconf[:max_viz_particles] = 50

    # Publish particle filter state
    pose_pub      = Publisher(rosconf[:pose_topic], PoseStamped, queue_size = 1) # Publishes the expected pose
    particle_pub  = Publisher(rosconf[:particle_topic], PoseArray, queue_size = 1) # Publishes a subsample of the particles
    laserposepub = Publisher("/pf/ta/viz/laserposepub", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    scratchposes = Publisher("/pf/ta/viz/scratchposes", PoseArray, queue_size = 1) # Publishes a subsample of the particles
    odom_pub = Publisher("/pf/ta/viz/odom", Odometry, queue_size = 1) # Publishes the path of the car
#:static_map_service=>"/static_map",
#:ackermann_topic=>"/vesc/low_level/ackermann_cmd_mux/output",
#:pose_topic=>"/pf/inferred_pose",
#:particle_topic=>"/pf/viz/particles")
    rosconf[:scan_topic] = "/scan"
    rosconf[:initialpose_topic] = "/initialpose"

    # setup PF
    pfconf[:rng] = Random.MersenneTwister(1234)

    pf = setup_pf(occmap, T_MW, Pose2D{Float64}, pfconf)
    rospf = ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposes)
    #return pf

    pose_sub  = Subscriber(rosconf[:initialpose_topic], PoseWithCovarianceStamped, clicked_pose_cb, (rospf,), queue_size=1)
    loginfo("Waiting for initial pose...")
    while !rospf.initialized
        rossleep(Duration(0.001))
    end
    laser_sub = Subscriber(rosconf[:scan_topic], LaserScan, laser_cb, (rospf,), queue_size=1)
    ackermann_sub = Subscriber(rosconf[:ackermann_topic], AckermannDriveStamped, ackermann_cb, (rospf,), queue_size=1)
    loginfo("Initialization complete")

    #while !is_shutdown()
    #    rossleep(Duration(0.25))
    #end
    spin()
end

end # module
using .ROSWrapper: main
if ! isinteractive()
    main()
end
