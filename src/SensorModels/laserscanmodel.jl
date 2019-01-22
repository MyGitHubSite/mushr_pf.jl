using RayCast.Bresenham: calc_hit_heading
using RoboLib.Util: euclidean, to_grid

struct LaserScanModel{M, FOV, MAXR, MAXX, MAXY, F}
    model::M
    fov::FOV
    maxrange::MAXR
    maxx::MAXX
    maxy::MAXY
    occupied::F
end
function (s::LaserScanModel)(state_t, ctrl_t, state_t1, scan_t1)
    map = zeros(Bool, (600, 600))
    x, y, theta = to_grid(map, state_t...)
    min_theta, max_theta = theta - s.fov / 2, theta + s.fov / 2
    prob = 1
    for (heading, (xhit, yhit)) in zip(LinRange(min_theta, max_theta, length(scan_t1)), scan_t1)
        xhit_exp, yhit_exp = calc_hit_heading(x, y, heading, s.maxrange, s.maxx, s.maxy, s.occupied)
        range_exp = euclidean(x, y, xhit_exp, yhit_exp)
        range = euclidean(x, y, xhit, yhit)
        #println(range, ' ', range_exp)
        prob *= s.model(range, range_exp)
    end

    return prob
end