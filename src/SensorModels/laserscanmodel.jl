using RayCast.Bresenham: cast_heading
using RoboLib.Util: euclidean, to_grid

export LaserScanModel

struct LaserScanModel{F, M, S, FOV, MAXR} <: AbstractSensorModel
    "castmethod(x, y, theta)->(xhit, yhit)"
    castmethod::F
    beammodel::M
    inv_squash_factor::S
    fov::FOV
    maxrange::MAXR
end

function (m::LaserScanModel)(state_t, obs_t, rng)
    x, y, theta = state_t
    #println((x,y,theta))
    obsheadings, obsranges = obs_t
    prob = 0
    #TODO(cxs): don't depend on beammodel being logprob

    #println("SCANALL")
    for (obsheading, obsrange) in zip(obsheadings, obsranges)
        xhit_exp, yhit_exp = m.castmethod(x, y, obsheading+theta)
        range_exp = euclidean(x, y, xhit_exp, yhit_exp)
        #println("scan")
        #println((x,y,theta))
        #println((xhit_exp, yhit_exp))
        #println((obsrange, range_exp))
        p = m.beammodel(obsrange, range_exp)
        #println(p)

        @assert p <= 0
        prob += p
    end
    #println(exp(prob))
    return prob * m.inv_squash_factor
end