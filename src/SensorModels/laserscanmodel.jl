using RayCast.Bresenham: cast_heading
using RoboLib.Util: euclidean, to_grid
using RoboLib.Geom: Pose2D

export LaserScanModel

#TODO:(remove mutable, scratch)
mutable struct LaserScanModel{F, M, S, MAXR} <: AbstractSensorModel
    "castmethod(x, y, theta)->(xhit, yhit)"
    castmethod::F
    beammodel::M
    inv_squash_factor::S
    maxrange::MAXR
    scratchposes
    scratchhitposes
    scratchraw
end


function LaserScanModel(cast::F, beam::M, squash::S, maxrange::R) where {F,M,S,R}
    LaserScanModel{F, M, S, R}(cast, beam, squash, maxrange, [], [], [])
end

function (m::LaserScanModel)(state_t, obs_t, rng)
    x, y, theta = state_t.statev
    #println((x,y,theta))
    obsheadings, obsranges = obs_t
    prob = 0
    #TODO(cxs): don't depend on beammodel being logprob


    m.scratchposes=[]
    m.scratchhitposes=[]
    #m.scratchraw=[]
    for (i, (obsheading, obsrange)) in enumerate(zip(obsheadings, obsranges))
        xhit_exp, yhit_exp = m.castmethod(x, y, obsheading+theta)
        range_exp = euclidean(x, y, xhit_exp, yhit_exp)
        #TODO(CXS): remove hardcode
        p = m.beammodel(obsrange, range_exp*0.02)
        #println(p)

        push!(m.scratchhitposes, Pose2D(xhit_exp, yhit_exp, obsheading+theta))
        push!(m.scratchposes, Pose2D(x, y, theta))
        #push!(m.scratchraw, Pose2D())

        @assert p <= 0
        prob += p
    end
    #println(exp(prob))
    return prob * m.inv_squash_factor
end