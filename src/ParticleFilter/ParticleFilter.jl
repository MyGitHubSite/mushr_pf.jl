module ParticleFilter

include("particlefilter.jl")

using RayCast.Bresenham: cast_heading
using RoboLib.Util: rangebearing2point, binarize, mul!, img2grid, grid2img, binarize, euclidean
using MuSHRSLAM.SensorModels: LaserScanModel, DiscBeamModel
using MuSHRSLAM.SensorModels
using MuSHRSLAM.MotionModels: AckerParams, AckerData, step!, BasicStochasticPredictModel, predict!
using MuSHRSLAM.MotionModels
using RoboLib.Geom: Pose2D
using FastClosures
using StaticArrays

#TODO(cxs): make vector state a Pose (need to define mean/mode/other functions over pose)

function setup_predict_model(c::Dict{Symbol, Any})
    mc = c[:predictconf]

    if mc[:type] == :ackermann
        data = AckerData{ c[:dtype] }()
        params = AckerParams{c[:dtype]}(mc[:car_length])
    else
        error("\"$mc[:type]\" not a known model")
    end

    # TODO(cxs): add biases
    # TODO(cxs): add proportional noise
    if mc[:stochastic]
        process_dist = MvNormal([0,0,0], [mc[:sigma_x], mc[:sigma_y], mc[:sigma_theta]])
        ctrl_dist = MvNormal([0,0], [mc[:vel_sigma], mc[:delta_sigma]])
        init_dist = MvNormal([0,0,0], [mc[:init_sigma_x], mc[:init_sigma_y], mc[:init_sigma_theta]])
        model = BasicStochasticPredictModel(data, params, ctrl_dist, process_dist, init_dist)

        predictfn = @closure (p, c, rng, dt)->predict!(p, c, rng, dt, model)
        resetfn = @closure (part, p, rng)->MotionModels.reset!(part, p, rng, model)
    else
        error("not yet implemented")
    end
    return predictfn, resetfn
end

function setup_reweight_model(mapoccfn, T_MW, c::Dict{Symbol, Any})
    sc = c[:reweightconf]

    if sc[:type] == :beammodel
        raymethod_W = get_raymethod_W(mapoccfn, sc[:max_range_meters], T_MW, sc[:scale_WM], sc[:raymethod])
        beammodel = DiscBeamModel(sc[:scale_WM], sc[:max_range_meters]; a_short=sc[:a_short], a_max=sc[:a_max], a_rand=sc[:a_rand], a_hit=sc[:a_hit], sigma_hit=sc[:sigma_hit],
            lambda_short=sc[:lambda_short], logprob=true)
        reweightmodel = LaserScanModel(beammodel, raymethod_W, sc[:squash_factor])
        reweightfn = @closure (w, p, o, rng, dt)->SensorModels.reweight!(w, p, o, rng, dt, reweightmodel)
    else
        error("not yet implemented")
    end
    return reweightfn
end

_mapoccfn(x::Integer, y::Integer, binmap::Matrix{Bool})::Bool = binmap[x, y]::Bool

function process_map(map, test)
    binmap = binarize(map, test)
    mapoccfn = let b=binmap
        f(x::Integer, y::Integer) = @inbounds b[x,y]::Bool
    end
    return binmap, mapoccfn
end

function _rangecast(T_WP::Pose2D{T}, angle, max_range_M, T_MW, mapoccfn, scale_WM) where T
    T_MP = T_MW ∘ T_WP
    x_M, y_M, theta_M = T_MP.statev

    x_G, y_G, theta_G = img2grid(x_M, y_M, theta_M)
    xhit_G, yhit_G = cast_heading(x_G, y_G, theta_G, max_range_M, mapoccfn)
    xhit_M, yhit_M = grid2img(xhit_G, yhit_G)

    #T_W_HIT = Pose2D{T}(inv(T_MW) ∘ Pose2D(xhit_M, yhit_M, angle+theta_M))
    T(scale_WM * euclidean(x_M, y_M, xhit_M, yhit_M))::T
end

#TODO(cxs): refactor raymethods
function get_raymethod_W(mapoccfn, max_range_W, T_MW, scale_WM, cast_method::Symbol=:bresenham)
    max_range_M = max_range_W / scale_WM
    if cast_method == :bresenham
        #@rm = @closure (T_WP, angle)->_rangecast(T_WP, angle, max_range_M, T_MW, mapoccfn, scale_WM)
        rm=let max_range_M=max_range_M, T_MW=T_MW, mapoccfn=mapoccfn, scale_WM=scale_WM
            function f(T_WP::Pose2D, angle)
                _rangecast(T_WP, angle, max_range_M, T_MW, mapoccfn, scale_WM)
            end
        end
    else
        error("\"$cast_method\" not a know raycasting method")
    end
    return rm
end

function setup_pf(mapoccfn, T_MW, c::Dict{Symbol, Any})
    predictfn, resetfn = setup_predict_model(c)
    reweightfn = setup_reweight_model(mapoccfn, T_MW, c)
    #TODO HACK out of bounds err
    particles = BufferedWeightedParticleBelief([Pose2D(5,5,0) for _ in 1:c[:max_particles]])
    StatefulParticleFilter(particles, predictfn, reweightfn, resetfn)
end

end # module