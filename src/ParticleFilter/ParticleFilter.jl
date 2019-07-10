module ParticleFilter

using RayCast.Bresenham: cast_heading
using Serialization
using RoboLib.Util: rangebearing2point, binarize, img2grid, grid2img, binarize, euclidean
using mushr_pf.SensorModels: LaserScanModel, DiscBeamPDF
using mushr_pf.SensorModels
using mushr_pf.MotionModels: AckerParams, AckerData, step!, BasicStochasticPredictModel, predict!, reset!
using mushr_pf.MotionModels
using RoboLib.Geom: Pose2D, Scale2D, T2D, R2D
using CoordinateTransformations, Rotations
using StaticArrays

include("particlefilter.jl")

struct OccMap{M<:AbstractMatrix{Bool}, TWM, TMW, P, O, S}
    map::M
    T_WM::TWM
    T_MW::TMW
    pos_WM::P
    ori_WM::O
    scale_WM::S
end
function OccMap(map::M, pos_WM::P, ori_WM::O, scale_WM::S) where {M<:AbstractMatrix{Bool},P<:T2D,O<:R2D,S<:Real}
    T_WM = Translation(pos_WM) ∘ LinearMap(ori_WM) ∘ LinearMap(Scale2D(scale_WM))
    T_MW = inv(T_WM)
    OccMap{M, typeof(T_WM), typeof(T_MW), P, O, S}(map, T_WM, T_MW, pos_WM, ori_WM, scale_WM)
end
OccMap(testfn, map, pos_WM, ori_WM, scale_WM) = OccMap(binarize(map, testfn), pos_WM, ori_WM, scale_WM)

# Dangerous indeed!
@inline (m::OccMap)(x::Integer, y::Integer) = @inbounds m.map[x,y]

function setup_predict_model(c::Dict{Symbol, Any})
    mc = c[:predictconf]

    if mc[:type] == :ackermann
        data = AckerData{c[:dtype]}()
        params = AckerParams{c[:dtype]}(mc[:car_length])
    else
        error("\"$mc[:type]\" not a known model")
    end

    if mc[:stochastic]
        process_dist = MvNormal([0,0,0], [mc[:sigma_x], mc[:sigma_y], mc[:sigma_theta]])
        ctrl_dist = MvNormal([0,0], [mc[:vel_sigma], mc[:delta_sigma]])
        init_dist = MvNormal([0,0,0], [mc[:init_sigma_x], mc[:init_sigma_y], mc[:init_sigma_theta]])
        model = BasicStochasticPredictModel(data, params, ctrl_dist, process_dist, init_dist)
    else
        error("not yet implemented")
    end
    model
end

function setup_reweight_model(om::OccMap, c::Dict{Symbol, Any})
    sc = c[:reweightconf]

    if sc[:type] == :beammodel
        #TODO make configurable
        raymethodW = RangeMethodW(om, sc[:max_range_meters])
        beammodel = DiscBeamPDF(om.scale_WM, sc[:max_range_meters]; a_short_lin=sc[:a_short_lin], a_short_exp=sc[:a_short_exp], a_max=sc[:a_max], a_rand=sc[:a_rand], a_hit=sc[:a_hit], sigma_hit=sc[:sigma_hit], lambda_short=sc[:lambda_short])
        reweightmodel = LaserScanModel{c[:dtype]}(beammodel, raymethodW, sc[:inv_squash_factor], sc[:nrays], sc[:method])
    else
        error("not yet implemented")
    end
    reweightmodel
end

struct RangeMethodW{D, M<:OccMap, MR<:Real}
    occmap::M
    mr_M::MR
    function RangeMethodW{D}(map::M, mr_W::MR) where {D,M,MR}
        new{D,M,MR}(map, mr_W/map.scale_WM)
    end
    RangeMethodW(map, mr_W) = RangeMethodW{Int}(map, mr_W)
end

function (rm::RangeMethodW{D})(T_WP::Pose2D{T}, angle) where {D,T}
    T_MP = rm.occmap.T_MW ∘ T_WP
    x_M, y_M, theta_M = T_MP.statev
    x_G, y_G, theta_G = img2grid(x_M, y_M, theta_M)
    xhit_G, yhit_G = cast_heading(D, x_G, y_G, theta_G+angle, rm.mr_M, rm.occmap)
    T(rm.occmap.scale_WM * euclidean(x_G, y_G, xhit_G, yhit_G))::T
end

function setup_resampler(c)
    rs = c[:resampler]
    if rs === :naive
        return resample_naive!
    elseif rs === :lowvar
        error("fix me") #TODO
        return resample_lowvar!
    else
        error("Resampler \"$rs\" unknown!")
    end
end

function setup_pf(occmap::OccMap, c::Dict{Symbol, Any})
    predictmodel = setup_predict_model(c)
    reweightmodel = setup_reweight_model(occmap, c)
    resampler = setup_resampler(c)
    particles = BufferedWeightedParticleBelief{Pose2D{c[:dtype]}}(c[:max_particles])

    StatefulParticleFilter(particles, predictmodel, reweightmodel, resampler, c[:rng])
end

end # module
