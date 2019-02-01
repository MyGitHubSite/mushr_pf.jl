using RayCast.Bresenham: cast_heading
using RoboLib.Util: euclidean, img2grid
using RoboLib.Geom: Pose2D
using LinearAlgebra

export LaserScanModel

#TODO:(remove mutable, scratch)
mutable struct LaserScanModel{M, F, S} <: AbstractSensorModel
    beammodel::M
    "rangefn(T_WP::Pose2D, theta_W)->(xhit_W, yhit_W)"
    rangefn::F
    inv_squash::S
    #scratchposes
    #scratchhitposes
    #scratchraw
end

#function LaserScanModel(beammodel::M, rangefn::F, inv_squash::S) where {M, F, S}
#    LaserScanModel{M, F, S}(beammodel, rangefn, inv_squash)#, [], [], [])
#end

# assumes state_t, obs_t in SI units and that compose(m.T_MW, state_t.statev)
# is in map frame/units
#TODO(cxs): don't depend on beammodel being logprob
function reweight!(weights_k, particles_WP_k::AbstractVector{<:Pose2D{T}}, obs_W_k, rng, dt, m::LaserScanModel) where T
    #state_t_MP = m.T_MW ∘ state_t_WP
    #x, y, theta = state_t_MP.statev

    obsangles_k, obsranges_W_k = obs_W_k
    prob = 0

    #m.scratchposes=[]
    #m.scratchhitposes=[]
    #maxw = -1000000000
    #maxi = 0
    #maxposes = []
    @inbounds for i in eachindex(particles_WP_k)
        #poses=[]
        T_WP = particles_WP_k[i]
        for (obsangle, obsrange_W) in zip(obsangles_k, obsranges_W_k)
            range_exp_W = m.rangefn(T_WP, obsangle)
            p = m.beammodel(obsrange_W, range_exp_W)
            #x_W, y_W, theta_W = T_WP.statev
            #push!(poses, T_WHIT)
            #@assert p <= 0
            prob += p
        end
        weights_k[i] = prob * m.inv_squash
        #if weights_k[i] > maxw
        #    maxw=weights_k[i]
        #    maxi=i
        #    maxposes=poses
        #end

    end

    return weights_k
    #for p in maxposes
    #    push!(m.scratchhitposes, p)
    #end
    #println(exp(prob))
end