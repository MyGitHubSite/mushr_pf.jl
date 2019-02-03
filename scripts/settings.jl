# Whether to use logprob, prob, or AMCL version of BeamModel
const stdconf=Dict(:name=>:std, :inv_squash=>0.3)
const amclconf=Dict(:name=>:amcl)
const logconf=Dict(:name=>:log, :inv_squash=>0.3)

const reweightconf = Dict{Symbol,Any}(
:type=>:beammodel,
:method=>stdconf,
:raymethod=>:bresenham,
:max_range_meters=>20,
:squash_factor=>1/8,
:nrays=>10,
:a_short=>0.1,
:a_max=>0.2,
:a_rand=>:0.1,
:a_hit=>0.75,
:sigma_hit=>8.0,
:lambda_short=>1/5.0)

const predictconf = Dict{Symbol,Any}(
:type=>:ackermann,
:sigma_x=>0.1,
:sigma_y=>0.1,
:sigma_theta=>0.02,
#:sigma_x=>0.1,
#:sigma_y=>0.1,
#:sigma_theta=>0.025,
:init_sigma_x=>0.3,
:init_sigma_y=>0.3,
:init_sigma_theta=>0.05,
:vel_sigma=>0.05,
:delta_sigma=>0.05,
:car_length=>0.33,
:stochastic=>true)

const pfconf=Dict{Symbol,Any}(:max_particles=>1000, :predictconf=>predictconf, :reweightconf=>reweightconf)

const rosconf = Dict{Symbol,Any}(
:max_viz_particles=>50,
:static_map_service=>"/static_map",
:ackermann_topic=>"/vesc/low_level/ackermann_cmd_mux/output",
:pose_topic=>"/pf/inferred_pose",
:particle_topic=>"/pf/viz/particles")

const conf = Dict{Symbol,Any}(:rosconf=>rosconf, :pfconf=>pfconf)