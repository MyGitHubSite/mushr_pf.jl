#!/usr/bin/env bash
# -*- mode: julia -*-
#=
curdir="$(dirname "$(readlink -f "$0")")"
parentdir=$(dirname "$curdir")
exec julia-1.0.3 --project="$parentdir" "${BASH_SOURCE[0]}"
=#
module ROSWrapper
#TODO strongly and consistently type (esp w/ gpu to avoid Float64/32 conv)
#TODO get rid of branching in cb's
using RoboLib.Util: binarize, takeN, rangebearing2point
using CoordinateTransformations, Rotations
using RoboLib.Geom: Pose2D, project2D, Scale2D
using StaticArrays
using LinearAlgebra
using Statistics
using Random
using MuSHRSLAM.ParticleFilter: setup_pf, update_act!, update_obs!, reset!, resample!, OccMap

# ROS Imports
using RobotOS
@rosimport sensor_msgs.msg: LaserScan
@rosimport nav_msgs.msg: Odometry
@rosimport nav_msgs.srv: GetMap
@rosimport geometry_msgs.msg: PoseStamped, PoseWithCovarianceStamped, PoseArray, Quaternion
@rosimport std_msgs.msg: Header, Float64
@rosimport ackermann_msgs.msg: AckermannDriveStamped
@rosimport vesc_msgs.msg: VescStateStamped

rostypegen(@__MODULE__)
using .sensor_msgs.msg
using .nav_msgs.msg
using .nav_msgs.srv
using .geometry_msgs.msg
using .std_msgs.msg: Header, Float64Msg
using .ackermann_msgs.msg
using .vesc_msgs.msg: VescStateStamped
# -----

include("settings.jl")
include("util.jl")

function laser_cb(msg::LaserScan, rospf::ROSPF)
    @debugtask begin
        h = msg.header
        trecv = time()
        if rospf.last_laser == -1
            rospf.last_laser = trecv

            # a wise man once said "buffers are the way to happiness"
            angles = LinRange(msg.angle_min, msg.angle_max, length(msg.ranges))
            resize!(rospf.angles, length(angles))
            resize!(rospf.laseridxs, length(angles))
            rospf.angles .= angles
        else
            dt = trecv - rospf.last_laser
            rospf.last_laser = trecv

            #println((1/dt, size())

            idxs = validindices(msg.ranges, rospf)
            angles = takeN(view(rospf.angles, idxs), rospf.nrays)
            ranges = takeN(view(msg.ranges, idxs), rospf.nrays)

            @assert length(angles) == length(ranges) == rospf.nrays

            update_obs!(rospf.pf, (angles, ranges), dt)
            resample!(rospf.pf)

            meanpose = mean(rospf.pf.belief.particles)
            publish(rospf.odom_pub, rosodometrystamped(meanpose, "map"))
            publish(rospf.pose_pub, rosposestamped(meanpose, "map"))

            # Publish a subset of the particles
            particles = view(rospf.pf.belief.particles, rand!(rospf.rand_part_idxs, eachindex(rospf.rand_part_idxs)))
            publish(rospf.particle_pub, rosposearraystamped(particles, "map"))

            # Publish the range measurements used in the reweighting step
            laserposes!(rospf, angles, ranges, meanpose)
            laserposes = rosposearraystamped(rospf.laserposes, "map")
            publish(rospf.laserpose_pub, laserposes)
        end
    end
end

function clicked_pose_cb(msg::PoseWithCovarianceStamped, rospf::ROSPF)
    @debugtask begin
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        r2d = project2D(Quat(q.w, q.x, q.y, q.z))
        t2d = SVector{2}(p.x, p.y)
        reset!(rospf.pf, Pose2D(r2d, t2d))

        rospf.initialized = true
    end
end

function servo_cb(msg, rospf::ROSPF)
    rospf.last_servo = msg.data
end

function vesc_cb(msg, rospf::ROSPF)
    @debugtask begin
        trecv = to_sec(msg.header.stamp)
        if rospf.last_servo == -1
            return
        elseif rospf.last_vesc == -1
            rospf.last_vesc = trecv
        else
            dt = trecv - rospf.last_vesc
            rospf.last_vesc = trecv

            v = (msg.state.speed - rospf.speed2erpm_offset) / rospf.speed2erpm_gain
            delta = (rospf.last_servo - rospf.steering2servo_offset) / rospf.steering2servo_gain

            update_act!(rospf.pf, SVector(v, delta), dt)
        end
    end
end

function ackermann_cb(msg, rospf::ROSPF)
    @debugtask begin
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
    end
end

#TODO(cxs): nested params
#TODO(cxs): handle array/dict params
#TODO(cxs): handle unknown
function get_params(d::Dict{Symbol, Any})
    for (param_name, value) in d
        if isa(value, Dict)
            get_params(value)
        elseif isa(value, Union{Number, String})
            if has_param('~' * string(param_name))
                new_value = oftype(value, get_param('~' * string(param_name), value))
                @assert typeof(value) == typeof(new_value)
                loginfo("Found param \"$param_name\". Default value: \"$value\", new value: \"$new_value\"")
                d[param_name] = new_value
            else
                loginfo("Param \"$param_name\" not set. Using default value: \"$value\"")
            end
        end
    end
end

function process_ros_map(map_msg)
    info = map_msg.map.info

    s = info.resolution
    p = info.origin.position
    q = info.origin.orientation

    # something something row major vs column major
    occmap = reshape(map_msg.map.data', (info.width, info.height))'
    occmap = OccMap((el)->el!=0, occmap, SVector(p.x, p.y), project2D(Quat(q.w, q.x, q.y, q.z)), s)

    return occmap
end

function main()
    init_node("particle_filter")

    get_params(conf)

    loginfo("Waiting for map service...")
    wait_for_service(rosconf[:static_map_service])
    map_msg = ServiceProxy(rosconf[:static_map_service], GetMap)(GetMapRequest())
    occmap = process_ros_map(map_msg)

    pose_pub      = Publisher(rosconf[:pose_topic], PoseStamped, queue_size = 1)
    particle_pub  = Publisher(rosconf[:particle_topic], PoseArray, queue_size = 1)
    laserpose_pub = Publisher(rosconf[:laserpose_topic], PoseArray, queue_size = 1)
    odom_pub = Publisher(rosconf[:odom_topic], Odometry, queue_size = 1)

    # setup PF
    pfconf[:rng] = Random.MersenneTwister(pfconf[:rng_seed])
    pf = setup_pf(occmap, pfconf)
    rospf = ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, occmap, laserpose_pub)

    pose_sub  = Subscriber(rosconf[:initialpose_topic], PoseWithCovarianceStamped, clicked_pose_cb, (rospf,), queue_size=1)

    loginfo("Waiting for initial pose...")
    while !rospf.initialized
        rossleep(Duration(0.001))
    end

    laser_sub = Subscriber(rosconf[:scan_topic], LaserScan, laser_cb, (rospf,), queue_size=1)
    servo_sub = Subscriber(rosconf[:servo_pos_topic], Float64Msg, servo_cb, (rospf,), queue_size=1)
    vesc_sub = Subscriber(rosconf[:vesc_topic], VescStateStamped, vesc_cb, (rospf,), queue_size=1)
    #ackermann_sub = Subscriber(rosconf[:ackermann_topic], AckermannDriveStamped, ackermann_cb, (rospf,), queue_size=1)

    loginfo("Initialization complete")
    d = Duration(0.001)
    while !is_shutdown() rossleep(d) end
end

end # module

using .ROSWrapper: main
if !isinteractive()
    main()
end
