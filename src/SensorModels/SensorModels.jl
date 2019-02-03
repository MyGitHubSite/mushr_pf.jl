module SensorModels

export SensorData, SensorParams, SensorModel, BeamPDF

abstract type AbstractSensorData end
abstract type AbstractSensorParams end
abstract type AbstractSensorModel <: Function end
abstract type AbstractBeamPDF <: Function end

include("beammodel.jl")
include("laserscanmodel.jl")

end