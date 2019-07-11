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

@inline function _step(x, y, theta, dt, v, delta, car_length)
    beta = atan(tan(delta) / 2)
    theta_dot = (2 * v * sin(beta)) / car_length
    theta_n = theta + theta_dot * dt

    x_dot = (car_length / (2 * sin(beta))) * (sin(theta_n + beta) - sin(theta + beta))
    y_dot = (car_length / (2 * sin(beta))) * (-cos(theta_n + beta) + cos(theta + beta))

    x_n = x + x_dot
    y_n = y + y_dot

    x_n, y_n, theta_n
end

@inline function _step_nodelta(x, y, theta, dt, v, delta, car_length)
    beta = atan(tan(delta) / 2)
    s_theta_beta, c_theta_beta = sincos(theta + beta)

    x_dot = v * c_theta_beta
    y_dot = v * s_theta_beta
    theta_dot = (2 * v * sin(beta)) / car_length

    x_n = x + x_dot * dt
    y_n = y + y_dot * dt
    theta_n = theta + theta_dot * dt

    x_n, y_n, theta_n
end

function step!(data::AckerData{T}, model::AckerParams, dt) where T
    v, delta = data.ctrl
    xc, yc, thetac = data.pose.statev
    if abs(delta) < 0.001
        x, y, theta = _step_nodelta(xc, yc, thetac, dt, v, delta, model.car_length)
    else
        x, y, theta = _step(xc, yc, thetac, dt, v, delta, model.car_length)
    end

    data.pose = Pose2D{T}(x, y, theta)
    data
end
