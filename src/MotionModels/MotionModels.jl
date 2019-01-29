module MotionModels

using Random
using StaticArrays
using Distributions

export MotionData, MotionParams, MotionModel, step!

abstract type MotionData end
abstract type MotionParams end
abstract type MotionModel <: Function end
abstract type StochasticMotionModel <: MotionModel end

#(m::M)(state_t, ctrl_t) where M<:MotionModel = nothing
#(sm::SM)(state_t, ctrl_t, rng) where SM<:StochasticMotionModel = nothing


include("ackermann.jl")

end # module