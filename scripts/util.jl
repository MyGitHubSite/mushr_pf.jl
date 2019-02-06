#TODO: clean up this gawdawful struct
mutable struct ROSPF{C, P, D, POPUB, PAPUB, ODPUB, T, LP, LP2}
    conf::C
    pf::P
    dtype::D

    pose_pub::POPUB
    particle_pub::PAPUB
    odom_pub::ODPUB

    T_MW::T
    T_WM::T

    # scratch
    laserpose_pub::LP

    mr_W::Float64
    maxviz::Int
    nrays::Int

    last_laser::Float64
    last_acker::Float64
    last_vesc::Float64
    last_servo::Float64

    speed2erpm_offset::Float64
    speed2erpm_gain::Float64
    steering2servo_offset::Float64
    steering2servo_gain::Float64

    initialized::Bool

    rand_part_idxs::Vector{Int}
    angles::Vector{Float64}
    laseridxs::Vector{Int}
    laserposes::LP2
end

function ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub)
    mr_W = conf[:pfconf][:reweightconf][:max_range_meters]
    maxviz = conf[:rosconf][:max_viz_particles]
    nrays = conf[:pfconf][:reweightconf][:nrays]
    ridxs = Vector{Int}(undef, maxviz)
    angles = Vector{Float64}(undef, 1)
    laseridxs = Vector{Int}(undef, 1)
    spo = conf

    spo = conf[:rosconf][:vescconf][:speed2erpm_offset]
    spg = conf[:rosconf][:vescconf][:speed2erpm_gain]
    sto = conf[:rosconf][:vescconf][:steering2servo_offset]
    stg = conf[:rosconf][:vescconf][:steering2servo_gain]

    laserposes = Vector{Pose2D{conf[:pfconf][:dtype]}}(undef, nrays)
    ROSPF(conf, pf, conf[:pfconf][:dtype], pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, convert(Float64, mr_W), maxviz, nrays, -1.0, -1.0, -1.0,-1.0, spo, spg, sto, stg, false, ridxs,angles,laseridxs, laserposes)
end

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

function laserposes!(rospf, angles, ranges, meanpose)
    @inbounds for (i, (a, r)) in enumerate(zip(angles, ranges))
        x, y, theta = meanpose.statev
        theta += a
        xhit, yhit = rangebearing2point(x, y, theta, r)
        rospf.laserposes[i] = Pose2D{rospf.dtype}(xhit, yhit, theta)
    end
    rospf
end

function rosheader(frame_id, stamp=nothing)
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp===nothing ? get_rostime() : stamp
    header
end

@inline rosheader!(msg, ::Nothing, ::Nothing) = msg
@inline function rosheader!(msg, frame_id=nothing, stamp=nothing)
    msg.header = rosheader(frame_id, stamp)
    msg
end

@inline function rosodometrystamped(pose::Pose2D, frame_id, stamp=nothing)
    odom = Odometry()
    odom.pose = roscovarpose(pose)
    rosheader!(odom, frame_id, stamp)
end

@inline function roscovarposestamped(pose::Pose2D, frame_id, stamp=nothing)
    rcps = PoseWithCovarianceStamped()
    rcps.pose = roscovarpose(pose)
    rosheader!(rcps, frame_id, stamp)
end

@inline function roscovarpose(pose::Pose2D)
    covarpose = PoseWithCovariance()
    covarpose.pose = rospose(pose)
    covarpose
end

@inline function rosposestamped(pose::Pose2D, frame_id, stamp=nothing)
    rps = PoseStamped()
    rps.pose = rospose(pose)
    rosheader!(rps, frame_id, stamp)
end

@inline function rospose(pose::Pose2D)
    po = Pose()
    po.position.x = pose.x
    po.position.y = pose.y
    q=Quat(RotXYZ(0, 0, pose.theta))
    po.orientation = Quaternion(q.x, q.y, q.z, q.w)
    po
end

function validindices(ranges::Vector, rospf::ROSPF)
    j = 0
    @inbounds for i in eachindex(ranges)
        r = ranges[i]
        if !isnan(r) && 0<r<rospf.mr_W
            j += 1
            rospf.laseridxs[j] = i
        end
    end
    @assert 0< j <= length(ranges)
    view(rospf.laseridxs, 1:j)
end

function rosposearraystamped(ps::AbstractVector, frame_id, stamp=nothing)
    arr = PoseArray()
    arr.poses = Vector{Pose}(undef, length(ps))
    @inbounds @simd for i in eachindex(arr.poses)
        arr.poses[i] = rospose(ps[i])
    end
    rosheader!(arr, frame_id, stamp)
end
