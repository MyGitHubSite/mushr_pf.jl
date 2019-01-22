using CoordinateTransformations, Rotations, StaticArrays
using RoboLib.Geom: Pose2D
export AckerModel, AckerData

struct AckerModel{SCALAR}
    speed2erpm_offset::SCALAR
    speed2erpm_gain::SCALAR
    steering2servo_offset::SCALAR
    steering2servo_gain::SCALAR
    car_length::SCALAR
    dt::SCALAR
    #ctrl_noise!::Function
    #process_noise!::Function

    function AckerModel(
        # convention: foo2bar_offset has units bar
        # convention: foo2bar_gain has units foo / bar
        speed2erpm_offset::S,
        speed2erpm_gain::S,
        steering2servo_offset::S,
        steering2servo_gain::S,
        car_length::S,
        dt::S) where S

        new{S}(speed2erpm_offset,
            speed2erpm_gain,
            steering2servo_offset,
            steering2servo_gain,
            #(state, ctrl)->ctrl,
            #(state, ctrl)->state,
            car_length,
            dt,
        )
    end
end

mutable struct AckerData{P<:Pose2D, C<:SVector{2}}
    pose::P
    ctrl::C

    function AckerData(pose::P, ctrl::C) where P<:Pose2D where C<:SVector{2}
        new{P, C}(pose, ctrl)
    end
    function AckerData(T::Type)
        pose = Pose2D(T, zero(T), zero(T), zero(T))
        ctrl = zeros(SVector{2, T})
        new{typeof(pose), typeof(ctrl)}(pose, ctrl)
    end
end
Base.eltype(d::AckerData{P, C}) where P where C = eltype(C)

function step!(data::AckerData, model::AckerModel)
    v, delta = data.ctrl
    xc, yc, thetac = data.pose.x, data.pose.y, data.pose.theta

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
    x = xc + x_dot * model.dt
    y = yc + y_dot * model.dt
    theta = thetac + theta_dot * model.dt

    data.pose = Pose2D(x, y, theta)
end