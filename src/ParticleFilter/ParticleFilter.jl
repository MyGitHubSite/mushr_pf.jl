module ParticleFilter

include("particlefilter.jl")

using RayCast.Bresenham: cast_heading
using RoboLib.Util: rangebearing2point, binarize, mul!, img2grid, grid2img, binarize, euclidean
using MuSHRSLAM.SensorModels: LaserScanModel, DiscBeamPDF
using MuSHRSLAM.SensorModels
using MuSHRSLAM.MotionModels: AckerParams, AckerData, step!, BasicStochasticPredictModel, predict!, reset!
using MuSHRSLAM.MotionModels
using RoboLib.Geom: Pose2D
using FastClosures
using CoordinateTransformations, Rotations
using StaticArrays

struct OccMap{M<:AbstractMatrix{Bool}}
    map::M
    @inline OccMap(map, test) = OccMap(binarize(map, test))
    @inline OccMap(map::M) where M<:AbstractMatrix{Bool} = new{M}(map)
end
@inline (m::OccMap)(x::Integer, y::Integer)=m.map[x,y]

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

        #TODO(cxs) experiment w/ making these functors
        #predictmodel = @closure (p, c, rng, dt)->MotionModels.predict!(p, c, rng, dt, model)
        #resetfn = @closure (part, p, rng)->MotionModels.reset!(part, p, rng, model)
    else
        error("not yet implemented")
    end
    #return predictfn, resetfn
    return model
end

function setup_reweight_model(om::OccMap, T_MW, c::Dict{Symbol, Any})
    sc = c[:reweightconf]

    if sc[:type] == :beammodel
        raymethodW = RangeMethodW(om, sc[:max_range_meters], T_MW, sc[:scale_WM])
        #beammodel = DiscBeamPDF(sc[:scale_WM], sc[:max_range_meters]; a_short=sc[:a_short], a_max=sc[:a_max], a_rand=sc[:a_rand], a_hit=sc[:a_hit], sigma_hit=sc[:sigma_hit], lambda_short=sc[:lambda_short])
        beammodel = DiscBeamPDF(sc[:scale_WM], sc[:max_range_meters])
        reweightmodel = LaserScanModel(beammodel, raymethodW, sc[:method])
        #reweightfn = @closure (w, p, o, rng, dt)->SensorModels.reweight!(w, p, o, rng, dt, reweightmodel)
    else
        error("not yet implemented")
    end
    #return reweightfn
    reweightmodel
end


#function _rangecast(T_WP::Pose2D{T}, angle, max_range_M, T_MW, map, scale_WM) where T
    #T_MP = T_MW ∘ T_WP
    #x_M, y_M, theta_M = T_MP.statev
    #x_G, y_G, theta_G = img2grid(x_M, y_M, theta_M)
    #f(x,y)::Bool = mapoccfn(x,y)::Bool
    #f= let ma=mapoccfn
    #    function(x,y):Bool
    #        ma[x,y]::Bool
    #    end
    #end
    #(x,y)->@inbounds mapoccfn[x,y]
    #cast_heading(Int, 400, 2000, pi/3, 1000, map)
    #xhit_M, yhit_M = grid2img(xhit_G, yhit_G)
    #T(scale_WM * euclidean(x_M, y_M, xhit_M, yhit_M))::T
#end

#TODO specifiy restrictions on C
struct RangeMethodW{C, M<:OccMap, MR<:Real, T<:Union{Pose2D, AffineMap}, S<:Real}
    ctype::C
    occmapf::M
    mr_M::MR
    T_MW::T
    scale_WM::S
end
RangeMethodW(map, mr_W, T_MW, scale_WM) = RangeMethodW(Int, map, mr_W, T_MW, scale_WM)

function (r::RangeMethodW)(T_WP::Pose2D{T}, angle, hit::Symbol) where T
    T_MP = r.T_MW ∘ T_WP
    x_M, y_M, theta_M = T_MP.statev
    x_G, y_G, theta_G = img2grid(x_M, y_M, theta_M)
    xhit_G, yhit_G = cast_heading(r.ctype, x_G, y_G, theta_G+angle, r.mr_M, r.occmapf)
    xhit_M, yhit_M, theta_M, = grid2img(xhit_G, yhit_G, theta_G)
    T_MH = Pose2D(xhit_M, yhit_M, theta_M)
    range_W=T(r.scale_WM * euclidean(x_G, y_G, xhit_G, yhit_G))::T
    if hit===:M
        THIT = T_MH
    elseif hit===:W
        THIT = inv(r.T_MW) ∘ T_MH
    else
        error("$hit not a valid frame")
    end
    return range_W, THIT
end
function (r::RangeMethodW)(T_WP::Pose2D{T}, angle) where T
    T_MP = r.T_MW ∘ T_WP
    x_M, y_M, theta_M = T_MP.statev
    x_G, y_G, theta_G = img2grid(x_M, y_M, theta_M)
    #println((theta_G,angle))
    #println("XG ", (x_G, y_G, x_M, y_M))
    #println("WMG ANGLE: ", (T_WP.theta, theta_M, theta_G))
    xhit_G, yhit_G = cast_heading(r.ctype, x_G, y_G, theta_G+angle, r.mr_M, r.occmapf)
    xhit_M, yhit_M = grid2img(xhit_G, yhit_G)
    #T(scale_WM * euclidean(x_M, y_M, xhit_M, yhit_M))::T
    r=T(r.scale_WM * euclidean(x_G, y_G, xhit_G, yhit_G))::T
    #println("HIT MG: ", ((xhit_M, yhit_M), (xhit_G, yhit_G)))
    #println((x_M,y_M,xhit_M,yhit_M,r))
    return r
end

#todo proper T
function setup_pf(occmap::OccMap, T_MW, T, c::Dict{Symbol, Any})
    #predictfn, resetfn = setup_predict_model(c)
    #reweightfn = setup_reweight_model(occmap, T_MW, c)
    predictmodel = setup_predict_model(c)
    reweightmodel = setup_reweight_model(occmap, T_MW, c)
    particles = BufferedWeightedParticleBelief{T}(c[:max_particles])
    StatefulParticleFilter(particles, predictmodel, reweightmodel, c[:rng])
end

end # module