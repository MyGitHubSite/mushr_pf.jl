module ParticleFilter

include("particlefilter.jl")

using RayCast.Bresenham: cast_heading
using RoboLib.Util: rangebearing2point, binarize, mul!, to_grid, from_grid, binarize
using MuSHRSLAM.SensorModels: LaserScanModel, DiscBeamModel
using MuSHRSLAM.MotionModels: AckerModel, AckerParams, AckerData, step!
using RoboLib.Geom: Pose2D
using StaticArrays

#TODO(cxs): make vector state a Pose (need to define mean/mode/other functions over pose)

function setup_motion_model(binmap, mapoccupiedf, c::Dict{Symbol, Any})
    modeltype, mc = c[:motionmodel], c[:motionconfig]

    if modeltype == :ackermann
        data = AckerData(c[:dtype])
        params = AckerParams(mc[:speed2erpm_offset], mc[:speed2erpm_gain], mc[:steering2servo_offset], mc[:steering2servo_gain], mc[:car_length])
        model = AckerModel(data, params)
    else
        @assert false
    end

    process_dist = MvNormal([0,0,0], [mc[:sigma_x], mc[:sigma_y], mc[:sigma_theta]])
    ctrl_dist = MvNormal([0,0], [mc[:vel_sigma], mc[:delta_sigma]])
    init_dist = MvNormal([0,0,0], [mc[:init_sigma_x], mc[:init_sigma_y], mc[:init_sigma_theta]])

    init_posef = let d=init_dist, n=length(init_dist), rng=c[:rng]
        function f(pose::Pose2D)
            nx, ny, nrz = rand(rng, d)
            x,y,rz = pose.statev
            return Pose2D(x+nx, y+ny, rz+nrz)
        end
    end

    if mc[:stochastic]
        stochasticmotionmodel = let m=model, pd=process_dist, cd=ctrl_dist
            function f(state::Pose2D, ctrl::SVector{N2, T2}, rng, dt) where {N2,T2}
                state_t1 = model(state, ctrl + SVector{N2, T2})

                nx, ny, nrz = rand(rng, pd)
                x,y,rz = state_t1.statev

                return Pose2D(x+nx, y+ny, rz+nrz)
            end
        end
        return stochasticmotionmodel, init_posef
    else
        return model, identity
    end
end

function setup_sensor_model(binmap, mapoccupiedf, c::Dict{Symbol, Any})
    modeltype, sc = c[:sensormodel], c[:sensorconfig]

    if modeltype == :beam
        bc = sc[:beamconfig]
        lc = sc[:laserconfig]

        raymethod = setup_raycast(mapoccupiedf, lc[:max_range_px], size(binmap,1))

        beammodel = DiscBeamModel(bc[:resolution], lc[:max_range_px]; a_short=bc[:a_short], a_max=bc[:a_max], a_rand=bc[:a_rand], a_hit=bc[:a_hit], sigma_hit=bc[:sigma_hit],
            lambda_short=bc[:lambda_short], logprob=true)
        sensormodel = LaserScanModel(raymethod, beammodel, lc[:squash_factor], lc[:max_range_px])
    else
        @assert false
    end
    return sensormodel
end

function process_map(map, test)
    binmap = binarize(map, test)
    mapoccupied(x::Integer, y::Integer)::Bool = let binmap=binmap
        @inbounds binmap[x, y]
    end
    return binmap, mapoccupied
end

function setup_raycast(mapoccupiedf, maxrange, h)
    let mr=maxrange, mof=mapoccupiedf, h=h
        function rc(x, y, theta)
            #xg, yg, thetag = to_grid(h, x, y, theta)
            xg, yg, thetag = y, x, pi/2 - theta
            xhitg, yhitg = cast_heading(xg, yg, thetag, mr, mof)
            #xhit, yhit = from_grid(h, xhitg, yhitg)
            xhit, yhit = yhitg, xhitg
            return xhit, yhit, theta
        end
    end
end

function setup_pf(binmap, mapoccupiedf, c::Dict{Symbol, Any})


    motionmodel, init_posef = setup_motion_model(binmap, mapoccupiedf, c)
    sensormodel  = setup_sensor_model(binmap, mapoccupiedf, c)

    #TODO(cxs): clean up, allocate empty vector
    particles = BufferedWeightedParticleBelief([Pose2D() for _ in 1:c[:max_particles]])
    pf = StatefulParticleFilter(particles, motionmodel, sensormodel)
    return pf, (pose)->reset!(pf, pose, init_posef)
end

end # module