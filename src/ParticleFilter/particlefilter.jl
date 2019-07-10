using Random
using Distributions
using Statistics
using RoboLib.Geom: Pose2D
#TODO(cx) extend or pass fn?
import mushr_pf.MotionModels: reset!
import mushr_pf.MotionModels: predict!
import mushr_pf.SensorModels: reweight!
using LinearAlgebra
using StatsBase: Weights, sample!

# TODO(cxs): port what works out back to upstream
struct BufferedWeightedParticleBelief{T,W}
    particles::Vector{T}
    _particle_memory::Vector{T}

    weights::Vector{W}
    _weight_memory::Vector{W}

end

@inline resetweights!(b::BufferedWeightedParticleBelief{T,W}) where {T,W} = fill!(b.weights, 1/length(b.weights))

@inline function BufferedWeightedParticleBelief(particles::AbstractVector{T}, weights::AbstractVector{W}) where {T,W}
    BufferedWeightedParticleBelief{T,W}(particles, similar(particles), weights, similar(weights))
end
@inline function BufferedWeightedParticleBelief{T,W}(n::Integer) where {T,W}
    BufferedWeightedParticleBelief(Vector{T}(undef, n), Vector{W}(undef, n))
end
@inline BufferedWeightedParticleBelief{T}(n::Integer) where {T} = BufferedWeightedParticleBelief{T, Float64}(n)
@inline function BufferedWeightedParticleBelief{W}(particles::AbstractVector) where {W}
    weights = fill(one(W)/length(particles), length(particles))
    BufferedWeightedParticleBelief(particles, weights)
end
@inline BufferedWeightedParticleBelief(particles::AbstractVector) = BufferedWeightedParticleBelief{Float64}(particles)


Distributions.mean(b::BufferedWeightedParticleBelief) = sum((b.weights .* b.particles)) ./ sum(b.weights)
Distributions.mode(b::BufferedWeightedParticleBelief) = b.particles[argmax(b.weights)]
Distributions.mean(b::BufferedWeightedParticleBelief{<:Pose2D}) = mean(b.particles, b.weights)

struct StatefulParticleFilter{PC,P,R,S,RNG<:AbstractRNG}
    belief::PC
    predictmodel::P
    reweightmodel::R
    resample!::S
    rng::RNG
end

#@inline function StatefulParticleFilter(pc::BufferedWeightedParticleBelief, predictmodel, reweightmodel)
#    StatefulParticleFilter(pc, predictmodel, reweightmodel, Random.GLOBAL_RNG)
#end
#@inline function StatefulParticleFilter(n::Integer, predictmodel, reweightmodel)
#    StatefulParticleFilter(BufferedWeightedParticleBelief
#end

#todo: handle no reweight
function resample_lowvar!(b::BufferedWeightedParticleBelief, rng::AbstractRNG)
    error("fix me")
    ws = sum(b.weights)
    n = length(b.weights)
    pm = b._particle_memory
    wm = b._weight_memory

    dr = 1/n*ws
    r = rand(rng) * dr
    c = b.weights[1]
    U = r
    i = 1
    @inbounds for m in 1:n
        while U > c
            i += 1
            c += b.weights[i]
        end
        U += dr
        pm[m] = b.particles[i]
        wm[m] = b.weights[i]
    end
    copy!(b.particles, pm)
    #NOTE:(cxs): were weights not reset before?)
    #before: copy!(b.weights, wm)
    resetweights!(b)
end

function resample_naive!(b::BufferedWeightedParticleBelief, rng::AbstractRNG)
    weights = Weights(b.weights)
    @inbounds sample!(b.particles, weights, b.particles)
    resetweights!(b)
end

@inline function update_act!(up::StatefulParticleFilter, a, dt)
    predict!(up.belief.particles, a, up.rng, dt, up.predictmodel)
    up
end

@inline function update_obs!(up::StatefulParticleFilter, o, dt)
    b = up.belief
    reweight!(b.weights, b.particles, o, up.rng, dt, up.reweightmodel)
    up
end

@inline function reset!(pf::StatefulParticleFilter, state)
    reset!(pf.belief.particles, state, pf.rng, pf.predictmodel)
    resetweights!(pf.belief)
    pf
end

@inline resample!(up::StatefulParticleFilter) = up.resample!(up.belief, up.rng)

#function update!(up::StatefulParticleFilter, a, o, dt)
#    b = up.belief
#    pm = b._particle_memory
#    p = b.particles
#    wm = b._weight_memory
#    w = b.weights
#    resize!(pm, length(b.particles))
#    resize!(wm, length(b.particles))
#    predictmodel(p, up.predict_model, a, up.rng, dt)
#    reweightmodel(w, p, up.reweight_model, o, up.rng, dt)
#    # TODO(cxs): inplace resampler
#    resample!(b, up.rng)
#    return up
#end

