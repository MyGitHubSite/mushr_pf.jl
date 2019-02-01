using CoordinateTransformations, Rotations, StaticArrays
using RoboLib.Geom: Pose2D
export AckerParams, AckerData



#NOTE(cxs): I think there's a bug preventing m from being qualified as subtype of abstract type

struct AckerParams{SCALAR<:AbstractFloat} <: MotionParams
    #speed2erpm_offset::SCALAR
    #speed2erpm_gain::SCALAR
    #steering2servo_offset::SCALAR
    #steering2servo_gain::SCALAR
    car_length::SCALAR
    #ctrl_noise!::Function

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

step!(data::AckerData, model::AckerParams, dt, rng) = step!(data, model, dt)


function step!(data::AckerData{T}, model::AckerParams, dt) where T
    v, delta = data.ctrl
    xc, yc, thetac = data.pose.statev

    if abs(delta) < 0.001
        s, c = sincos(thetac)
        x_dot = v*c
        y_dot = v*s
        theta_dot = 0
    else
        beta = atan(tan(delta) / 2)
        s_thetac_beta, c_thetac_beta = sincos(thetac + beta)

        x_dot = v * c_thetac_beta
        y_dot = v * s_thetac_beta
        theta_dot = 2 * v / model.car_length * sin(beta)
    end
    x = xc + x_dot * dt
    y = yc + y_dot * dt
    theta = thetac + theta_dot * dt

    data.pose = Pose2D{T}(x, y, theta)
    return data
end