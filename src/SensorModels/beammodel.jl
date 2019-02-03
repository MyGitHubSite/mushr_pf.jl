using Distributions, LinearAlgebra
using FastClosures

#TODO after changeing raycast add map to sense functor args
#TODO(cxs): normalize beam model in disc no cont b/c discretely sampled pdf

struct BeamPDF{T<:AbstractFloat} <: AbstractBeamPDF
    a_hit::T
    a_short::T
    a_max::T
    a_rand::T
    sigma_hit::T
    lambda_short::T
    zmax::T

    function BeamPDF(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax)
        eta = a_short + a_max + a_rand + a_hit
        a_hit /= eta
        a_short /= eta
        a_max /= eta
        a_rand /= eta
        new{Float64}(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax)
    end
end

# NOTE: unnormalized b/c there's no way to
# normalize before discretization
function (m::BeamPDF)(z, zexp_W)
    if zexp_W > m.zmax return 0 end
    # p_hit
    p = pdf(Normal(zexp_W, m.sigma_hit), z) * m.a_hit
    # p_short
    p += 0<=z<=zexp_W ? pdf(Exponential(1/m.lambda_short), z) * m.a_short : 0
    # p_max TODO: set tolerance
    p += z ≈ m.zmax ? m.a_max : 0
    # p_rand
    p += m.a_rand/m.zmax
    return p
end


struct DiscBeamPDF{P, S, M} <: AbstractBeamPDF
    _p_z_zexp_W::P # _p_z_zexp_W[z, z_exp] = p(z | zexp_W)
    _scale::S # units= 1/units(zmax)
    _zmax::M

    function DiscBeamPDF(scale_WM, zmax_W; a_hit=0.75, a_short=0.1, a_max=0.05, a_rand=0.1, sigma_hit=1.0,
        lambda_short=1/5)

        # +1 b/c z in inclusive range [0,zmax]
        dists_W = collect(LinRange(0, zmax_W, ceil(Int, zmax_W/scale_WM) + 1))
        p_z_zexp_W = zeros(Float64, length(dists_W), length(dists_W))
        model = BeamPDF(a_hit, a_short, a_max, a_rand, sigma_hit, lambda_short, zmax_W)
        for (i_zexp_W, zexp_W) in enumerate(dists_W)
            norm = 0.0
            for (i_z_W, z_W) in enumerate(dists_W)
                p = model(z_W, zexp_W)
                p_z_zexp_W[i_z_W, i_zexp_W] = p
                norm += p
            end
            p_z_zexp_W[:, i_zexp_W] ./= norm
        end
        scale = length(dists_W) / zmax_W
        new{typeof(p_z_zexp_W), typeof(scale), typeof(zmax_W)}(p_z_zexp_W, scale, zmax_W)
    end
end

# In world units
function (bm::DiscBeamPDF)(z_W::Real, zexp_W::Real)
    z_W = clamp(z_W, 0, bm._zmax)
    zexp_w = clamp(zexp_W, 0, bm._zmax)
    z_W = round(Int, z_W * bm._scale + 1)
    zexp_W = round(Int, zexp_W * bm._scale + 1)
    @assert 1 <= z_W <= length(bm._p_z_zexp_W)
    @assert 1 <= zexp_W <= length(bm._p_z_zexp_W)
    return @inbounds bm._p_z_zexp_W[z_W, zexp_W]
end