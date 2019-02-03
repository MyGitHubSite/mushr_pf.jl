using RayCast.Bresenham: cast_heading
using RoboLib.Util: euclidean, img2grid
using RoboLib.Geom: Pose2D
using LinearAlgebra

export LaserScanModel

@inline amcl_init() = 1
@inline amcl_combine(prob, p) = prob + p^3
@inline amcl_update(weight, prob) = weight *= prob
@inline amcl_final!(weights) = weights

@inline std_init() = 1
@inline std_combine(prob, p) = prob * p
@inline std_update(weight, prob) = prob
@inline std_final!(weights, inv_squash) = weights .^ inv_squash

@inline log_init() = 0
@inline log_combine(prob, p) = prob + log(p)
@inline log_update(weight, prob) = -prob
@inline log_final!(weights, inv_squash) = weights #* inv_squash

#TODO:(remove mutable, scratch)
mutable struct LaserScanModel{M, F, F1, F2, F3, F4} <: AbstractSensorModel
    beammodel::M
    "rangemethodW(T_WP::Pose2D, theta_W)->(xhit_W, yhit_W)"
    rangemethodW::F
    _init::F1
    _combine::F2
    _update::F3
    _final!::F4
    #scratchposes
    #scratchhitposes
    #scratchraw

    # TODO: generated? somehow inline variable update funcs?
    function LaserScanModel(beammodel::M, rangemethodW::F, type::Dict{Symbol, <:Any}=Dict(:name=>:std, :inv_squash=>0.3)) where {M,F}
        name = type[:name]
        if name === :std
            init = std_init
            combine = std_combine
            update = std_update
            final! = @closure (w)->std_final!(w, type[:inv_squash])
        elseif name === :amcl
            init = amcl_init
            combine = amcl_combine
            update = amcl_update
            final! = amcl_final!
        elseif name === :log
            init = log_init
            combine = log_combine
            update = log_update
            final! = @closure (w)->log_final!(w, type[:inv_squash])
        else
            error("Type \"$name\" not know")
        end
        new{M, F, typeof(init), typeof(combine), typeof(update), typeof(final!)}(beammodel, rangemethodW, init, combine, update, final!)
    end
end



function reweight!(weights_k, particles_WP_k::AbstractVector{<:Pose2D{T}}, obs_W_k, rng, dt, m::LaserScanModel) where T
    obsangles_k, obsranges_W_k = obs_W_k

    #@assert length(unique(weights_k)) == 1
    #@assert isapprox(weights_k[1], 1/length(weights_k)) "$(weights_k[1])"
    totalw = 0
    for i in eachindex(particles_WP_k)

        prob = m._init()
        #poses=[]
        T_WP = particles_WP_k[i]
        for (obsangle, obsrange_W) in zip(obsangles_k, obsranges_W_k)
            range_exp_W = m.rangemethodW(T_WP, obsangle)
            p = m.beammodel(obsrange_W, range_exp_W)
            #println((obsrange_W, range_exp_W),"\n")
            #x_W, y_W, theta_W = T_WP.statev
            #push!(poses, T_WHIT)
            #@assert p <= 0
            #wbar += p
            #prob += p^3
            prob = m._combine(prob, p)
            #println(prob)
        end
        #println("SQ ", m.inv_squash)
        #println("WBAR ", wbar/length(obsangles_k))
        weights_k[i] = m._update(weights_k[i], prob)
        #println((weights_k[i], prob))
        #println(prob)
        #println(weights_k[i])
        #if weights_k[i] > maxw
        #    maxw=weights_k[i]
        #    maxi=i
        #    maxposes=poses
        #end
    end
    #m._final!(weights_k)
    #println(weights_k[1:10])
    #weights_k ./= totalw
    #println(weights_k[1:10])
    #println(sum(weights_k))
    #println(minimum(weights_k), "--", maximum(weights_k))
    #for p in maxposes
    #    push!(m.scratchhitposes, p)
    #end
    #println(exp(prob))
end