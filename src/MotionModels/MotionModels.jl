module MotionModels

using Random
using StaticArrays
using Distributions

export MotionData, MotionParams, MotionModel, step!

abstract type MotionData end
abstract type MotionParams end
abstract type MotionModel <: Function end
abstract type StochasticMotionModel <: Function end

abstract type PredictModel <: Function end
abstract type StochasticPredictModel <: Function end

#(m::M)(state_t, ctrl_t) where M<:MotionModel = nothing
#(sm::SM)(state_t, ctrl_t, rng) where SM<:StochasticMotionModel = nothing

#struct BasicPredictModel{D, P} <: PredictModel
#    data::D
#    params::P
#end
#
#function predict!(state_t, ctrl_t, dt, m::BasicPredictModel)
#    m.data.pose = state_t
#    step!(m.data, m.params, dt)
#    return m.data.pose
#end

include("ackermann.jl")

struct BasicStochasticPredictModel{D, P, CD, PD, ID} <: StochasticPredictModel
    data::D
    params::P
    ctrl_dist::CD
    process_dist::PD
    init_dist::ID
end

function reset!(particles::AbstractVector{Pose2D{P}}, pose::Pose2D, rng, m::BasicStochasticPredictModel) where {P}
    statev = pose.statev
    @inbounds @simd for i in eachindex(particles)
        particles[i] = Pose2D{P}(statev + rand(rng, m.init_dist))
    end
end

function predict!(particles::AbstractVector{Pose2D{P}}, ctrl::SVector{N, C}, rng, dt, m::BasicStochasticPredictModel) where {P,N,C}
    @inbounds @simd for i in eachindex(particles)
        m.data.pose = particles[i]
        m.data.ctrl = ctrl + SVector{N, C}(rand(rng, m.ctrl_dist))
        step!(m.data, m.params, dt)
        particles[i] = Pose2D{P}(m.data.pose.statev + rand(rng, m.process_dist))
    end
end

end # module