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

    mr_W::Float64
    maxviz::Int

    last_laser::Float64
    last_acker::Float64
    rand_part_idxs::Vector{Int}
    initialized::Bool
end
function ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposepub)
    mr_W = conf[:pfconf][:reweightconf][:max_range_meters]
    maxviz = conf[:rosconf][:max_viz_particles]
    ridxs = Vector{Int}(undef, maxviz)
    ROSPF(conf, pf, pose_pub, particle_pub, odom_pub, T_MW, T_WM, laserposepub, scratchposepub, mr_W, maxviz, -1.0, -1.0, ridxs, false)
end

function make_header(frame_id, stamp=nothing)
    if isnothing(stamp)
        stamp = get_rostime()
    end
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
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
      #exit()
    end
  end
end

@inline function makepose(particle::Pose2D)
    po = Pose()
    po.position.x = particle.x
    po.position.y = particle.y
    q=Quat(RotXYZ(0, 0, particle.theta))
    po.orientation = Quaternion(q.x, q.y, q.z, q.w)
    return po
end

function makeposearray(ps)
    arr = PoseArray()
    arr.poses = Vector{Pose}(undef, length(ps))
    @inbounds @simd for i in eachindex(arr.poses)
        arr.poses[i] = makepose(ps[i])
    end
    return arr
end

function makeposearray(rospf::ROSPF)
    ridxs = rospf.rand_part_idxs
    ps = rospf.pf.belief.particles
    arr = PoseArray()
    arr.poses = Vector{Pose}(undef, length(ridxs))
    rand!(ridxs, 1:length(ridxs))
    @inbounds @simd for i in eachindex(arr.poses)
        arr.poses[i] = makepose(ps[ridxs[i]])
    end
    return arr
end