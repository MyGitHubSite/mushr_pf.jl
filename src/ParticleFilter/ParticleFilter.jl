module ParticleFilter

using ParticleFilters
using ParticleFilters: LowVarianceResampler
using MuSHRSLAM.MotionModels: AckerModel, AckerData, step!
using RayCast.Bresenham: calc_hit_heading, BHamData
using MuSHRSLAM.SensorModels: BeamModel
using Random
using RoboLib.Geom: Pose2D
using StaticArrays
using RoboLib.Util: euclidean, from_grid, to_grid
using Distributions




struct MotionModel{M, D}
    model::M
    data::D
end
function (m::MotionModel)(state_t, ctrl_t, rng)
    x,y,th = state_t

    x += rand(Normal(0, 20))
    y += rand(Normal(0, 20))
    th +=rand(Normal(0, deg2rad(10))) * 0.1

    m.data.pose = Pose2D(x,y,th)
    m.data.ctrl = ctrl_t
    step!(m.data, m.model)
    return m.data.pose.statev
end

struct ParticleFilterAlg
    p
    pf
    function ParticleFilterAlg(initpose, motionmodel, sensormodel, nparticles)
        pfm = ParticleFilterModel{typeof(initpose)}((x, c, rng)->motionmodel(x, c, rng), (x, c, x1, y1)->sensormodel(x,c,x1,y1))
        p = ParticleCollection([initpose for _ in 1:nparticles])
        rng=Random.MersenneTwister(12)
        pf = BasicParticleFilter(pfm, LowVarianceResampler(nparticles), nparticles, rng)
        new(p, pf)
    end
end
function update(p::ParticleFilterAlg, u, y)
    newp= ParticleFilters.update(p.pf, p.p, u, y)
    p.p.particles = newp.particles
    p.p._probs = newp._probs
end


end # module