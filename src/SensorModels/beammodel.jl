using Distributions: Normal, Exponential, pdf
using LinearAlgebra

struct BeamPDF{T<:AbstractFloat} <: AbstractBeamPDF
    a_hit::T
    a_short_lin::T
    a_short_exp::T
    a_max::T
    a_rand::T
    sigma_hit::T
    lambda_short::T
    zmax::T

    function BeamPDF(a_hit, a_short_lin, a_short_exp, a_max, a_rand, sigma_hit, lambda_short, zmax)
        eta = a_short_lin + a_short_exp + a_max + a_rand + a_hit
        a_hit /= eta
        a_short_lin /= eta
        a_short_exp /= eta
        a_max /= eta
        a_rand /= eta
        new{Float64}(a_hit, a_short_lin, a_short_exp, a_max, a_rand, sigma_hit, lambda_short, zmax)
    end
end

function (m::BeamPDF)(z_W, zexp_W)
    # Not a valid measurement, but we don't want to collapse the distribution
    if z_W > m.zmax || z_W < 0 return m.a_rand/m.zmax end
    # p_hit
    p = pdf(Normal(zexp_W, m.sigma_hit), z_W) * m.a_hit
    # p_short_exp
    p += 0<=z_W<=zexp_W ? pdf(Exponential(1/m.lambda_short), z_W) * m.a_short_exp : 0
    # p_short_lin
    p += 0<=z_W<zexp_W ? 2 * m.a_short_lin * (zexp_W-z_W) / zexp_W : 0
    # p_max
    p += z_W == m.zmax ? m.a_max : 0
    # p_rand
    p += m.a_rand/m.zmax
    @assert !iszero(p) && !isnan(p)
    p
end

struct DiscBeamPDF{P, S, M} <: AbstractBeamPDF
    p_z_zexp_W::P # p_z_zexp_W[z, z_exp] = p(z | zexp_W)
    scale::S
    zmax::M
    arrsize::Int

    function DiscBeamPDF(scale_WM, zmax_W; a_hit=0.75, a_short_lin=0.1, a_short_exp=0.1, a_max=0.05, a_rand=0.1, sigma_hit=1.0,
        lambda_short=1/5)
        # +1 b/c z in inclusive range [0,zmax]
        dists_W = collect(LinRange(0, zmax_W, ceil(Int, zmax_W/scale_WM) + 1))
        p_z_zexp_W = zeros(Float64, length(dists_W), length(dists_W))
        model = BeamPDF(a_hit, a_short_lin, a_short_exp, a_max, a_rand, sigma_hit, lambda_short, zmax_W)
        for (i_zexp_W, zexp_W) in enumerate(dists_W)
            norm = 0.0
            for (i_z_W, z_W) in enumerate(dists_W)
                p = model(z_W, zexp_W)
                p_z_zexp_W[i_z_W, i_zexp_W] = p
                norm += p
            end
            p_z_zexp_W[:, i_zexp_W] ./= norm
        end
        @assert !any(isnan, p_z_zexp_W)
        @assert !any(iszero, p_z_zexp_W)
        new{typeof(p_z_zexp_W), typeof(scale_WM), typeof(zmax_W)}(p_z_zexp_W, 1/scale_WM, zmax_W, size(p_z_zexp_W,1))
    end
end

# In world units
function (bm::DiscBeamPDF)(z_W::Real, zexp_W::Real)
    z_W = round(Int, z_W * bm.scale + 1)
    zexp_W = round(Int, zexp_W * bm.scale + 1)
    z_W = clamp(z_W, 1, bm.arrsize)
    zexp_W = clamp(zexp_W, 1, bm.arrsize)
    @assert 1 <= z_W <= bm.arrsize
    @assert 1 <= zexp_W <= bm.arrsize
    @inbounds bm.p_z_zexp_W[z_W, zexp_W]
end
