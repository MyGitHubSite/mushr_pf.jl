module ParticleFilter

include("particlefilter.jl")

using RayCast.Bresenham: cast_heading
using RoboLib.Util: rangebearing2point, binarize, mul!, to_grid, from_grid, binarize
using MuSHRSLAM.SensorModels: LaserScanModel, DiscBeamModel
using MuSHRSLAM.MotionModels: AckerModel, AckerParams, AckerData, step!
using RoboLib.Geom: Pose2D
using StaticArrays


function setup_motion_model(binmap, mapoccupiedf, c::Dict{Symbol, Any})
    modeltype, mc = c[:motionmodel], c[:motionconfig]
    pose0 = Pose2D(c[:pose0])
    ctrl0 = SVector{length(c[:ctrl0])}(c[:ctrl0])

    if modeltype == :ackermann
        data = AckerData(pose0, ctrl0)
        params = AckerParams(mc[:speed2erpm_offset], mc[:speed2erpm_gain], mc[:steering2servo_offset], mc[:steering2servo_gain], mc[:car_length])
        model = AckerModel(data, params)
    else
        @assert false
    end

    process_dist = MvNormal([0,0,0], [mc[:sigma_x], mc[:sigma_y], mc[:sigma_theta]])
    ctrl_dist = MvNormal([0,0], [mc[:vel_sigma], mc[:delta_sigma]])
    init_dist = MvNormal([0,0,0], [mc[:init_sigma_x], mc[:init_sigma_y], mc[:init_sigma_theta]])

    init_posef = let d=init_dist, n=length(init_dist), rng=c[:rng]
        f(pose::SVector{N, T}) where {N,T} = SVector{N, T}(pose + rand(rng, d))
    end

    if mc[:stochastic]
        stochasticmotionmodel = let m=model, pd=process_dist, cd=ctrl_dist
            function f(state::SVector{N1, T1}, ctrl::SVector{N2, T2}, rng, dt) where {N1,T1,N2,T2}
                model(state, ctrl + SVector{N2, T2}(rand(rng, cd)), dt) + SVector{N1, T1}(rand(rng, pd))
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

        raymethod = setup_raycast(mapoccupiedf, lc[:maxrange], size(binmap,1))

        beammodel = DiscBeamModel(bc[:resolution], lc[:maxrange]; a_short=bc[:a_short], a_max=bc[:a_max], a_rand=bc[:a_rand], a_hit=bc[:a_hit], sigma_hit=bc[:sigma_hit],
            lambda_short=bc[:lambda_short], logprob=true)
        sensormodel = LaserScanModel(raymethod, beammodel, lc[:squash_factor], lc[:fov], lc[:maxrange])
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
            xg, yg, thetag = to_grid(h, x, y, theta)
            xhitg, yhitg = cast_heading(xg, yg, thetag, mr, mof)
            xhit, yhit = from_grid(h, xhitg, yhitg)
            return xhit, yhit, theta
        end
    end
end

function setup_pf(binmap, mapoccupiedf, c::Dict{Symbol, Any})

    pose0 = Pose2D(c[:pose0])
    ctrl0 = c[:ctrl0]

    motionmodel, init_posef = setup_motion_model(binmap, mapoccupiedf, c)
    sensormodel  = setup_sensor_model(binmap, mapoccupiedf, c)

    particles = BufferedWeightedParticleBelief([init_posef(pose0.statev) for _ in 1:c[:nparticles]])
    pf = StatefulParticleFilter(particles, motionmodel, sensormodel)
    return pf, (pose)->reset!(pf, pose, init_posef)
end

end # module