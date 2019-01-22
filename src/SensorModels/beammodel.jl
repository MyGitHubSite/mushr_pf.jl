using Distributions, LinearAlgebra

#TODO after changeing raycast add map to sense functor args
struct BeamModel
    squash_factor

    _p_z_zexp # _p_z_zexp[zexp_zexp, z_zexp] = p(z | zexp) b/c column order in build step below
    _scale # units= 1/units(max_range)

    function BeamModel(α_short, α_max, α_rand, α_hit, sigma_hit,
        lambda_short, squash_factor, disc, max_range)

        @assert 0 < disc
        @assert 0 < max_range

        dists = collect(range(0, step=disc, stop=max_range))
        p_z_zexp = zeros(Float64, length(dists), length(dists))
        for (idx, zexp) in enumerate(dists)
            # p_hit
            p_z_zexp[idx, :] .+= normalize!(pdf.(Normal(zexp, sigma_hit), dists)) * α_hit
            # p_short
            # TODO: verify that there is no cutoff? CSE490R pdf fig suggests this dist == 0 >= z_exp
            p_z_zexp[idx, :] .+= normalize!(pdf.(Exponential(lambda_short), dists)) * α_short
            # p_rand
            p_z_zexp[idx, :] .+= normalize!(pdf.(Uniform(0, max_range), dists)) * α_rand
            # p_max
            p_z_zexp[idx, end] += α_max

            #normalize!(view(p_z_zexp, idx, :))
        end
        scale = (length(dists) - 1)/ max_range
        new(squash_factor, p_z_zexp, scale)
    end
end
function (bm::BeamModel)(z::Real, zexp::Real)
    z = round(Int, z * bm._scale + 1)
    zexp = round(Int, zexp * bm._scale + 1)
    @assert 0 < z
    @assert 0 < zexp
    return bm._p_z_zexp[zexp, z]
end