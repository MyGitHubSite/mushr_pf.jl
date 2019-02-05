using RayCast.Bresenham: cast_heading
using RoboLib.Util: euclidean, img2grid
using RoboLib.Geom: Pose2D
using LinearAlgebra

export LaserScanModel

@inline amcl_init() = 1
@inline amcl_combine(prob, p) = prob + p^3
@inline amcl_update(weight, prob, inv_squash) = weight *= prob

@noinline std_init() = 1
@noinline std_combine(prob, p) = prob * p
@noinline std_update(weight, prob, inv_squash) = prob ^ inv_squash

@inline log_init() = 0
@inline log_combine(prob, p) = prob + log(p)
@inline log_update(weight, prob, inv_squash) = prob * inv_squash

struct LaserScanModel{D, M, F, F1, F2, F3, S} <: AbstractSensorModel
    beammodel::M
    "rangemethodW(T_WP::Pose2D, theta_W)->(xhit_W, yhit_W)"
    rangemethod::F
    init::F1
    combine::F2
    update::F3
    inv_squash_factor::S
    nposes::Int
    function LaserScanModel{D}(beammodel::M, rangemethod::F, inv_squash_factor::S, nposes::Int, method=:std) where {D,M,F,S}
        if method === :std
            init = std_init
            combine = std_combine
            update = std_update
        elseif method === :amcl
            init = amcl_init
            combine = amcl_combine
            update = amcl_update
        elseif method === :log
            init = log_init
            combine = log_combine
            update = log_update
        else
            error("Type \"$method\" not know")
        end
        new{D,M,F,typeof(init),typeof(combine),typeof(update), S}(beammodel, rangemethod, init, combine, update, inv_squash_factor, nposes)
    end
end

function reweight!(weights_k::Vector{W}, particles_WP_k::Vector{<:Pose2D{T}}, obs_W_k, rng, dt, m::LaserScanModel) where {W,T}
    obsangles_k, obsranges_W_k = obs_W_k

    #totalw = W(0)
    @inbounds Threads.@threads for i in eachindex(particles_WP_k)
        prob = W(m.init())
        T_WP = particles_WP_k[i]
        @inbounds for (obsangle, obsrange_W) in zip(obsangles_k, obsranges_W_k)
            range_exp_W = m.rangemethod(T_WP, -obsangle) #TODO: negative sign?
            p = m.beammodel(obsrange_W, range_exp_W)
            prob = m.combine(prob, p)
        end
        @assert !iszero(prob) && !isnan(prob)
        w = m.update(weights_k[i], prob, m.inv_squash_factor)
        weights_k[i] = w
        #totalw += w
    end
    # Don't actually need to normalize, but keeping it here for now
    # weights_k ./= totalw
    weights_k
end