using CoordinateTransformations, Rotations, StaticArrays
using RoboLib.Geom: Pose2D
export AckerParams, AckerData

struct AckerModel{D, P} <: MotionModel
    data::D
    params::P
end


#NOTE(cxs): I think there's a bug preventing m from being qualified as subtype of abstract type
function (m::AckerModel)(state_t, ctrl_t, dt)
    m.data.pose = state_t
    step!(m.data, m.params, dt)
    return m.data.pose
end
struct AckerParams{SCALAR} <: MotionParams
    speed2erpm_offset::SCALAR
    speed2erpm_gain::SCALAR
    steering2servo_offset::SCALAR
    steering2servo_gain::SCALAR
    car_length::SCALAR
    #ctrl_noise!::Function
    #process_noise!::Function

    function AckerParams(
        # convention: foo2bar_offset has units bar
        # convention: foo2bar_gain has units foo / bar
        speed2erpm_offset::S,
        speed2erpm_gain::S,
        steering2servo_offset::S,
        steering2servo_gain::S,
        car_length::S) where S

        new{S}(speed2erpm_offset,
            speed2erpm_gain,
            steering2servo_offset,
            steering2servo_gain,
            #(state, ctrl)->ctrl,
            #(state, ctrl)->state,
            car_length
        )
    end
end

mutable struct AckerData{P<:Pose2D, C<:SVector{2}} <: MotionData
    pose::P
    ctrl::C

    function AckerData(pose::P, ctrl::C) where P<:Pose2D where C<:SVector{2}
        new{P, C}(pose, ctrl)
    end
    function AckerData(T::Type)
        pose = Pose2D()
        ctrl = zeros(SVector{2, T})
        new{typeof(pose), typeof(ctrl)}(pose, ctrl)
    end
end
Base.eltype(d::AckerData{P, C}) where P where C = eltype(C)

step!(data::AckerData, model::AckerParams, dt, rng) = step!(data, model, dt)
# TODO(cxs): deal with non-zero rx ry z
function step!(data::AckerData, model::AckerParams, dt)
    v, delta = data.ctrl
    x, y, rz = data.pose.statev

    if abs(delta) < 0.001
        s, c = sincos(thetac)
        x_dot = v*c
        y_dot = v*s
        theta_dot = 0
    else
        v = (v - model.speed2erpm_offset) / model.speed2erpm_gain
        delta = (delta - model.steering2servo_offset) / model.steering2servo_gain

        beta = atan(tan(delta) / 2)
        s_thetac_beta, c_thetac_beta = sincos(thetac + beta)

        x_dot = v * c_thetac_beta
        y_dot = v * s_thetac_beta
        theta_dot = 2 * v / model.car_length * sin(beta)
    end
    x = xc + x_dot * dt
    y = yc + y_dot * dt
    theta = thetac + theta_dot * dt

    data.pose = Pose2D(x, y, rz)
    return data
end