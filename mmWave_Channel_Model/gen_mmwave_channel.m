function [ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
    gen_mmwave_channel( dev_config, ch_prop, simu_time )
%GEN_MMWAVE_CH generates one sample of the mmWave channel in P2P scenario
%   Input parameter ========
%   dev_config: the configuration of the tx/rx devices
%   ch_prop: the characterizations of the mmWave channle
%   simu_time: simulation time point
%   Output parameter =======
%   mmwave_ch_sample: the samples of the mmWave channel
%   los_vec_at: the samples of the LOS AOD spatial signature
%   los_vec_ar: the samples of the LOS AOA spatial signature
%   nlos_vec_at: the samples of the NLOS AOD spatial signature
%   nlos_vec_ar: the samples of the NLOS AOA spatial signature
%   TODO: LOS prob, Doppler shift

if nargin <= 2
    simu_time = 0;
end
j = sqrt(-1);

%% parameter initialization
% device config
carrier_freq = dev_config.carrier_freq;
veloc_tx = dev_config.veloc_tx;
veloc_rx = dev_config.veloc_rx;
pos_tx = dev_config.pos_tx + veloc_tx * simu_time;
pos_rx = dev_config.pos_rx + veloc_rx * simu_time;
num_ant_tx = dev_config.num_ant_tx;
num_ant_rx = dev_config.num_ant_rx;
spacing_ant_tx = dev_config.spacing_ant_tx;
spacing_ant_rx = dev_config.spacing_ant_rx;
type_array_tx = dev_config.type_array_tx;
type_array_rx = dev_config.type_array_rx;
% channel scattering properties
scenario = ch_prop.scenario;
num_cluster = max(1, poissrnd(ch_prop.lambda_cluster));
num_ray = ch_prop.num_ray;
% cluster power fraction
power_frac_tau = ch_prop.power_frac_tau;
power_frac_sigma = ch_prop.power_frac_sigma;
% angle of departure (AOD)
azmth_range_tx = ch_prop.azmth_range_tx * pi/180;
azmth_spread_tx = ch_prop.azmth_spread_tx * pi/180;
elvt_range_tx = ch_prop.elvt_range_tx * pi/180;
elvt_spread_tx = ch_prop.elvt_spread_tx * pi/180;
% angle of arrival (AOA)
azmth_range_rx = ch_prop.azmth_range_rx * pi/180;
azmth_spread_rx = ch_prop.azmth_spread_rx * pi/180;
elvt_range_rx = ch_prop.elvt_range_rx * pi/180;
elvt_spread_rx = ch_prop.elvt_spread_rx * pi/180;


% generate the mmWave channel sample based on the angles of each ray
nlos_vec_at = zeros(num_ant_tx, num_ray, num_cluster);
nlos_vec_ar = zeros(num_ant_rx, num_ray, num_cluster);

% TX/RX position
delta_x = pos_tx(1) - pos_rx(1);
delta_y = pos_tx(2) - pos_rx(2);
delta_z = pos_tx(3) - pos_rx(3);
dis_tx_rx = sqrt(delta_x^2 + delta_y^2 + delta_z^2);
assert(dis_tx_rx > 1, 'Distance between TX and RX must be larger than 1m');


%% Generate LOS path
azmth_los = atan(delta_y/dis_tx_rx);
elvt_los = atan(delta_z/dis_tx_rx);
% LOS large-scale fading
los_path_gain = 10^(-0.1*Evaluation_Path_loss(dis_tx_rx, carrier_freq, scenario, 1));
% LOS small-scale fading
los_rand_phase = exp(j*2*pi*rand());
los_azmth_t = azmth_los;
los_azmth_r = -azmth_los;
los_elvt_t = elvt_los;
los_elvt_r = -elvt_los;
los_vec_at = array_response( los_azmth_t, los_elvt_t, num_ant_tx, spacing_ant_tx, type_array_tx );
los_vec_ar = array_response( los_azmth_r, los_elvt_r, num_ant_rx, spacing_ant_rx, type_array_rx );
% LOS Doppler shift
los_angle_at = [sin(los_elvt_t)*cos(los_azmth_t), sin(los_elvt_t)*sin(los_azmth_t), cos(los_elvt_t)];
los_angle_ar = [sin(los_elvt_r)*cos(los_azmth_r), sin(los_elvt_r)*sin(los_azmth_r), cos(los_elvt_r)];
los_doppler_freq = carrier_freq/3e8*(dot(veloc_rx, los_angle_ar) + dot(veloc_tx, los_angle_at));
los_doppler_phase = exp(j*2*pi*los_doppler_freq*simu_time);
los_link = sqrt(num_ant_tx*num_ant_rx) * los_rand_phase * sqrt(los_path_gain) * ...
    los_doppler_phase * los_vec_ar * los_vec_at';
% LOS probability
los_prob = los_probability(dis_tx_rx, scenario);
mmwave_ch_sample.los_prob = los_prob;
mmwave_ch_sample.los_link = los_link;

%% Generate NLOS paths
% cluster mean angles
nlos_azmth_cluster_tx = (azmth_range_tx(2) - azmth_range_tx(1)) * ...
    rand(num_cluster, 1) + azmth_range_tx(1);
nlos_elvt_cluster_tx = (elvt_range_tx(2) - elvt_range_tx(1)) * ...
    rand(num_cluster, 1) + elvt_range_tx(1);
nlos_azmth_cluster_rx = (azmth_range_rx(2) - azmth_range_rx(1)) * ...
    rand(num_cluster, 1) + azmth_range_rx(1);
nlos_elvt_cluster_rx = (elvt_range_rx(2) - elvt_range_rx(1)) * ...
    rand(num_cluster, 1) + elvt_range_rx(1);
% cluster scatter-TX distances
dis_scatter_tx = (7/4*dis_tx_rx - 1) * rand(num_cluster, 1) + 1;
nlos_path_len = zeros(num_ray, num_cluster);

% cluster power fraction
power_frac_z = power_frac_sigma * randn(num_cluster, 1);
power_frac_u = rand(num_cluster, 1);
power_frac_gamma = power_frac_u.^(power_frac_tau - 1) .* 10.^(-0.1 * power_frac_z);
power_frac_gamma = power_frac_gamma / sum(power_frac_gamma);
% cluster-power-fraction based path gain
nlos_alpha = 1/sqrt(2) * (randn(num_ray, num_cluster) + ...
    j*randn(num_ray, num_cluster));
nlos_alpha = nlos_alpha .* sqrt(power_frac_gamma.');

nlos_link = zeros(num_ant_rx, num_ant_tx);
for index_cls = 1:num_cluster
    % nlos path angles
    nlos_azmth_ray_tx = rand_laplace(num_ray, 1, nlos_azmth_cluster_tx(index_cls), azmth_spread_tx, azmth_range_tx);
    nlos_elvt_ray_tx = rand_laplace(num_ray, 1, nlos_elvt_cluster_tx(index_cls), elvt_spread_tx, elvt_range_tx);
    nlos_azmth_ray_rx = rand_laplace(num_ray, 1, nlos_azmth_cluster_rx(index_cls), azmth_spread_rx,azmth_range_rx);
    nlos_elvt_ray_rx = rand_laplace(num_ray, 1, nlos_elvt_cluster_rx(index_cls), elvt_spread_rx, elvt_range_rx);
    % nlos path lengths
    dis_sct = dis_scatter_tx(index_cls);
    nlos_path_len(:, index_cls) = dis_sct + ...
            sqrt((pos_tx(3) - pos_rx(3) + dis_sct*sin(nlos_elvt_ray_tx)).^2 + ...
            (dis_tx_rx - dis_sct*cos(nlos_elvt_ray_tx).*cos(nlos_azmth_ray_tx)).^2);
    % generate the mmWave channel sample based on the angles of each ray
    for index_ray = 1:num_ray
        nlos_path_gain = 10^(-0.1*Evaluation_Path_loss(nlos_path_len(index_ray, index_cls), carrier_freq, scenario, 0));
        nlos_vec_at(:, index_ray, index_cls) = ...
            array_response(nlos_azmth_ray_tx(index_ray), nlos_elvt_ray_tx(index_ray), num_ant_tx, spacing_ant_tx, type_array_tx);
        nlos_vec_ar(:, index_ray, index_cls) = ...
            array_response(nlos_azmth_ray_rx(index_ray), nlos_elvt_ray_rx(index_ray), num_ant_rx, spacing_ant_rx, type_array_rx);
        % NLOS Doppler shift
        nlos_angle_at = [sin(nlos_elvt_ray_tx(index_ray))*cos(nlos_azmth_ray_tx(index_ray)), ...
            sin(nlos_elvt_ray_tx(index_ray))*sin(nlos_azmth_ray_tx(index_ray)), cos(nlos_elvt_ray_tx(index_ray))];
        nlos_angle_ar = [sin(nlos_elvt_ray_rx(index_ray))*cos(nlos_azmth_ray_rx(index_ray)), ...
            sin(nlos_elvt_ray_rx(index_ray))*sin(nlos_azmth_ray_rx(index_ray)), cos(nlos_elvt_ray_rx(index_ray))];
        nlos_doppler_freq = carrier_freq/3e8*(dot(veloc_rx, nlos_angle_ar) + dot(veloc_tx, nlos_angle_at));
        nlos_doppler_phase = exp(j*2*pi*nlos_doppler_freq*simu_time);
        
        nlos_link = nlos_link + sqrt(nlos_path_gain) * ...
            nlos_alpha(index_ray, index_cls) * nlos_doppler_phase * ...
            nlos_vec_ar(:, index_ray, index_cls) * nlos_vec_at(:, index_ray, index_cls)';
    end
end
nlos_link = sqrt(num_ant_tx*num_ant_rx / (num_ray)) * nlos_link;
mmwave_ch_sample.nlos_link = nlos_link;
        
end

