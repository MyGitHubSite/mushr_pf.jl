using Random
using Distributions
using Statistics
using RoboLib.Geom: Pose2D
using LinearAlgebra

# TODO(cxs): port what works out back to upstream
struct BufferedWeightedParticleBelief{T}
    particles::Vector{T}
    _particle_memory::Vector{T}

    weights::Vector{Float64}
    _weight_memory::Vector{Float64}

end

@inline function BufferedWeightedParticleBelief(particles::AbstractVector{T}, weights::AbstractVector{Float64}) where {T}
    BufferedWeightedParticleBelief{T}(particles, similar(particles), weights, similar(weights))
end

@inline function BufferedWeightedParticleBelief(particles::AbstractVector{T}) where {T}
    weights = fill(Float64(1)/length(particles), length(particles))
    BufferedWeightedParticleBelief(particles, weights)
end

Distributions.mean(b::BufferedWeightedParticleBelief) = sum((b.weights .* b.particles)) ./ sum(b.weights)
Distributions.mode(b::BufferedWeightedParticleBelief) = b.particles[argmax(b.weights)]
Distributions.mean(b::BufferedWeightedParticleBelief{Pose2D}) = mean(b.particles)

# TODO(cxs): immutable
mutable struct StatefulParticleFilter{PC,P,R,RE,RNG<:AbstractRNG}
    belief::PC
    predict!::P
    reweight!::R
    reset!::RE
    rng::RNG
end

function StatefulParticleFilter(pc::BufferedWeightedParticleBelief, predict!, reweight!, reset!)
    return StatefulParticleFilter(pc,
                                  predict!,
                                  reweight!,
                                  reset!,
                                  Random.GLOBAL_RNG
                                  )
end

function resample!(b::BufferedWeightedParticleBelief{S}, rng::AbstractRNG) where {S}
    #w = Weights(b.weights)
    #p = sample(b.particles, w, length(b.particles))
    #b.particles .= p
    #return b
    #b.weights[:] .= wm

    #println(b.weights)

    # convert logprob to prob
    w = b.weights
    m = maximum(w)
    w[:] .= exp.(w .- m)

    #println((minimum(b.weights), maximum(b.weights), mean(b.weights), std(b.weights)))
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
    copy!(b.weights, wm)
    return b
end

@inline function update_act!(up::StatefulParticleFilter, a, dt)
    up.predict!(up.belief.particles, a, up.rng, dt)
    up
end

@inline function update_obs!(up::StatefulParticleFilter, o, dt)
    b = up.belief
    up.reweight!(b.weights, b.particles, o, up.rng, dt)
    return up
end

@inline function reset!(pf::StatefulParticleFilter, state)
    pf.reset!(pf.belief.particles, state, pf.rng)
    pf.belief.weights .= 1/length(pf.belief.weights)
end

@inline resample!(up::StatefulParticleFilter) = resample!(up.belief, up.rng)

#function update!(up::StatefulParticleFilter, a, o, dt)
#    b = up.belief
#    pm = b._particle_memory
#    p = b.particles
#    wm = b._weight_memory
#    w = b.weights
#    resize!(pm, length(b.particles))
#    resize!(wm, length(b.particles))
#    predict!(p, up.predict_model, a, up.rng, dt)
#    reweight!(w, p, up.reweight_model, o, up.rng, dt)
#    # TODO(cxs): inplace resampler
#    resample!(b, up.rng)
#    return up
#end

