using CoordinateTransformations, Rotations, StaticArrays
using RoboLib.Geom: Pose2D
export AckerParams, AckerData

struct AckerParams{SCALAR<:AbstractFloat} <: MotionParams
    car_length::SCALAR
end

mutable struct AckerData{T} <: MotionData
    pose::Pose2D{T}
    ctrl::SVector{2, T}
    # TODO(cxs): special case
    @inline function AckerData{T}(pose, ctrl) where T
        pose = Pose2D{T}(pose)
        ctrl = SVector{2, T}(ctrl)
        new{T}(pose, ctrl)
    end
    @inline function AckerData{T}() where T
        pose = Pose2D{T}()
        ctrl = zeros(SVector{2, T})
        new{T}(pose, ctrl)
    end
end
@inline AckerData(pose, ctrl) = AckerData{Float64}(pose, ctrl)
@inline AckerData() = AckerData{Float64}()

@inline function _step(v, delta, thetac, car_length)
    beta = atan(tan(delta) / 2)
    s_thetac_beta, c_thetac_beta = sincos(thetac + beta)

    x_dot = v * c_thetac_beta
    y_dot = v * s_thetac_beta
    theta_dot = 2 * v / car_length * sin(beta)
    x_dot, y_dot, theta_dot
end

@inline function _step_nodelta(v, delta, thetac)
    s, c = sincos(thetac)
    x_dot = v*c
    y_dot = v*s
    theta_dot = 0
    x_dot, y_dot, theta_dot
end

function step!(data::AckerData{T}, model::AckerParams, dt) where T
    v, delta = data.ctrl
    xc, yc, thetac = data.pose.statev
    if abs(delta) < 0.001
        x_dot, y_dot, theta_dot = _step_nodelta(v, delta, thetac)
    else
        x_dot, y_dot, theta_dot = _step(v, delta, thetac, model.car_length)
    end
    x = xc + x_dot * dt
    y = yc + y_dot * dt
    theta = thetac + theta_dot * dt

    data.pose = Pose2D{T}(x, y, theta)
    data
end