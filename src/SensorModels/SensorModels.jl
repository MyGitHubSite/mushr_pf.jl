module SensorModels

export SensorData, SensorParams, SensorModel, BeamModel

abstract type AbstractSensorData end
abstract type AbstractSensorParams end
abstract type AbstractSensorModel <: Function end
abstract type AbstractBeamModel <: Function end

include("beammodel.jl")
include("laserscanmodel.jl")

end