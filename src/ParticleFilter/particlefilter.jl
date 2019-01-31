using Random
using Distributions
using LinearAlgebra

# TODO(cxs): port what works out back to upstream
struct BufferedWeightedParticleBelief{T}
    particles::Vector{T}
    _particle_memory::Vector{T}

    weights::Vector{Float64}
    _weight_memory::Vector{Float64}

end
function BufferedWeightedParticleBelief(particles::AbstractVector{T}, weights::AbstractVector{Float64}) where {T}
    BufferedWeightedParticleBelief{T}(particles, similar(particles), weights, similar(weights))
end
function BufferedWeightedParticleBelief(particles::AbstractVector{T}) where {T}
    weights = fill(Float64(1)/length(particles), length(particles))
    BufferedWeightedParticleBelief{T}(particles, similar(particles), weights, similar(weights))
end

Distributions.mean(b::BufferedWeightedParticleBelief) = sum((b.weights .* b.particles)) ./ sum(b.weights)
Distributions.mode(b::BufferedWeightedParticleBelief) = b.particles[argmax(b.weights)]

# ---

# TODO(cxs): immutable
mutable struct StatefulParticleFilter{PC,PM,RM,RNG<:AbstractRNG}
    belief::PC
    predict_model::PM
    reweight_model::RM
    rng::RNG
end

function StatefulParticleFilter(pc::BufferedWeightedParticleBelief, pmodel, rmodel)
    return StatefulParticleFilter(pc,
                                  pmodel,
                                  rmodel,
                                  Random.GLOBAL_RNG
                                  )
end
# ----

function reweight!(weights, particles, reweight_model::Function, obs, rng, dt)
    for i in 1:length(particles)
        @inbounds x = particles[i]
        @inbounds weights[i] = reweight_model(x, obs, rng)
    end
end

function predict!(particles, predict_model::Function, ctrl, rng, dt)
    for i in 1:length(particles)
        x = particles[i]
        particles[i] = predict_model(x, ctrl, rng, dt)
    end
end

# ----
using Statistics
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
    b.particles[:] .= pm
    b.weights[:] .= wm
    return b
end

function update_action!(up::StatefulParticleFilter, a, dt)
    b = up.belief
    pm = b._particle_memory
    p = b.particles
    wm = b._weight_memory
    w = b.weights
    resize!(pm, length(b.particles))
    resize!(wm, length(b.particles))
    predict!(p, up.predict_model, a, up.rng, dt)
    return up
end

function update_obs!(up::StatefulParticleFilter, o, dt)
    b = up.belief
    pm = b._particle_memory
    p = b.particles
    wm = b._weight_memory
    w = b.weights
    resize!(pm, length(b.particles))
    resize!(wm, length(b.particles))
    reweight!(w, p, up.reweight_model, o, up.rng, dt)
    # TODO(cxs): inplace resampler
    resample!(b, up.rng)
    return up
end

function update!(up::StatefulParticleFilter, a, o, dt)
    b = up.belief
    pm = b._particle_memory
    p = b.particles
    wm = b._weight_memory
    w = b.weights
    resize!(pm, length(b.particles))
    resize!(wm, length(b.particles))
    predict!(p, up.predict_model, a, up.rng, dt)
    reweight!(w, p, up.reweight_model, o, up.rng, dt)
    # TODO(cxs): inplace resampler
    resample!(b, up.rng)
    return up
end

function reset!(pf::StatefulParticleFilter, newpose, init_posef)
    b = pf.belief
    w = 1/length(b.particles)
    for i in 1:length(b.particles)
        b.particles[i] = init_posef(newpose)
        b.weights[i] = w
    end
end
