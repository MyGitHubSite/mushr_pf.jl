using Distributions, LinearAlgebra

#TODO after changeing raycast add map to sense functor args
#TODO(cxs): normalize beam model in disc no cont b/c discretely sampled pdf
struct BeamModel{T<:AbstractFloat} <: AbstractBeamModel
    a_hit::T
    a_short::T
    a_max::T
    a_rand::T
    sigma_hit::T
    lambda_short::T
    zmax::T
    logprob::Bool

    function BeamModel(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax; logprob=false)
        eta = a_short + a_max + a_rand + a_hit
        a_hit /= eta
        a_short /= eta
        a_max /= eta
        a_rand /= eta
        new{Float64}(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax, logprob)
    end
end

function (m::BeamModel)(z, zexp)
    if 0 <= z <= m.zmax
        d_hit = Normal(zexp, m.sigma_hit)
        #eta_hit = cdf(d_hit, m.zmax) - cdf(d_hit, 0) # integral_0_zmax Normal(zexp, m.sigma_hit)
        #p_hit = 1/eta_hit * pdf(d_hit, z)
        p_hit = pdf(d_hit, z)
    else
        p_hit = 0
    end
    if 0 <= z <= zexp
        d_short = Exponential(1/m.lambda_short)
        #eta_short = cdf(d_short, zexp) - cdf(d_short, 0)
        #TODO(cxs): deal with eta being 0
        p_short =  pdf(d_short, z)
        #p_short = 1/eta_short * pdf(d_short, z)
    else
        p_short = 0
    end
    if z ≈ m.zmax
        p_max = 1
    else
        p_max = 0
    end
    if 0 <= z <= m.zmax
        p_rand = 1/m.zmax
    else
        p_rand = 0
    end
    p = (m.a_hit * p_hit
       + m.a_short * p_short
       + m.a_max * p_max
       + m.a_rand * p_rand)
    if p > 0
        return m.logprob ? log(p) : p
    else
        return 0
    end
end


struct DiscBeamModel{P, S, M} <: AbstractBeamModel
    _p_z_zexp::P # _p_z_zexp[z, z_exp] = p(z | zexp)
    _scale::S # units= 1/units(zmax)
    _zmax::M

    function DiscBeamModel(resolution, zmax; a_hit=0.75, a_short=0.01, a_max=0.08, a_rand=0.1, sigma_hit=8.0,
        lambda_short=1/2.0, logprob=false)


        disc = round(Int, zmax / resolution)
        dists = collect(LinRange(0, zmax, disc))
        p_z_zexp = zeros(Float64, length(dists), length(dists))
        model = BeamModel(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax, logprob=logprob)
        for (i_zexp, zexp) in enumerate(dists)
            for (i_z, z) in enumerate(dists)
                p_z_zexp[i_z, i_zexp] = model(z, zexp)
            end
            normalize!(p_z_zexp[:, i_zexp])
        end
        scale = (length(dists) - 1)/ zmax
        new{typeof(p_z_zexp), typeof(scale), typeof(zmax)}(p_z_zexp, scale, zmax)
    end
end

function (p::DiscBeamModel)(z::Real, zexp::Real)
    # TODO(cxs): do we want to handle > zmax here or in caller?
    #if !(0 < z <= p._zmax) @warn("z not in range ", (z, p._zmax)) end
    #if !(0 < zexp <= p._zmax) @warn("zexp not in range ", (zexp, p._zmax)) end
    #println("ZZEXP ", (z, zexp))
    #println(p._zmax)
    z = min(z, p._zmax)
    zexp = min(zexp, p._zmax)
    #println("ZZEXP2 ", (z, zexp))
    z = round(Int, z * p._scale + 1)
    zexp = round(Int, zexp * p._scale + 1)
    #println("ZZEXP3 ", (z, zexp))
    #println(p._scale)
    return p._p_z_zexp[z, zexp]
end