const reweightconf = Dict{Symbol,Any}(
:type=>:beammodel,
:method=>:std,
:raymethod=>:bresenham,
#:max_range_meters=>20,
:max_range_meters=>5.6,
:exclude_max_range=>true,
:inv_squash_factor=>2,
:a_short_exp=>0.4,
:a_short_lin=>0.2,
:a_max=>2,
:a_rand=>:0.7,
:a_hit=>0.75,
:sigma_hit=>0.07,
:lambda_short=>1/0.6,
:nrays=>50)

const predictconf = Dict{Symbol,Any}(
:type=>:ackermann,
:sigma_x=>0.01,
:sigma_y=>0.01,
:sigma_theta=>0.05,
:init_sigma_x=>0.3,
:init_sigma_y=>0.3,
:init_sigma_theta=>0.05,
:vel_sigma=>0.000001,
:delta_sigma=>0.000001,
:car_length=>0.33,
:stochastic=>true)

const pfconf=Dict{Symbol,Any}(:max_particles=>3000, :dtype=>Float64, :rng_seed=>1234, :resampler=>:naive, :predictconf=>predictconf, :reweightconf=>reweightconf)

const vescconf = Dict{Symbol, Any}(
:speed2erpm_offset=>0.0,
:speed2erpm_gain=>4614.0,
:steering2servo_offset=>0.5304,
:steering2servo_gain=>-1.2135)

const rosconf = Dict{Symbol,Any}(
:max_viz_particles=>50,
:static_map_service=>"/static_map",
:ackermann_topic=>"/vesc/low_level/ackermann_cmd_mux/output",
:servo_pos_topic=>"/vesc/sensors/servo_position_command",
:vesc_topic=>"/vesc/sensors/core",
:pose_topic=>"/pf/inferred_pose",
:particle_topic=>"/pf/viz/particles",
:scan_topic=>"/scan",
:initialpose_topic=>"/initialpose",
:odom_topic=>"/odom",
:vescconf=>vescconf,
:laserpose_topic=>"/pf/viz/laserpose")

const conf = Dict{Symbol,Any}(:rosconf=>rosconf, :pfconf=>pfconf)