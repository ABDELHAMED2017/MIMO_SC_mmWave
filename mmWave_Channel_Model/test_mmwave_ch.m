dev_config.carrier_freq = 73e9;
dev_config.pos_tx = [0, 0, 10];
dev_config.pos_rx = [10, 1, 0];
dev_config.num_ant_tx = 64;
dev_config.num_ant_rx = 16;
dev_config.spacing_ant_tx = 0.5; 
dev_config.spacing_ant_rx = 0.5;
dev_config.type_array_tx = 'UPA';
dev_config.type_array_rx = 'UPA';


%%  channel scattering properties
ch_prop.scenario = 2;
ch_prop.lambda_cluster = 1.9;
ch_prop.num_ray = 20;
% cluster power fraction
ch_prop.power_frac_tau = 1.1;
ch_prop.power_frac_sigma = 1;
% angle of departure (AOD)
ch_prop.azmth_range_tx = [-90, 90];
ch_prop.azmth_spread_tx = 5;
ch_prop.elvt_range_tx = [-90, 90];
ch_prop.elvt_spread_tx = 5;
% angle of arrival (AOA)
ch_prop.azmth_range_rx = [-180, 180];
ch_prop.azmth_spread_rx = 5;
ch_prop.elvt_range_rx = [-90, 90];
ch_prop.elvt_spread_rx = 5;

[ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
    gen_mmwave_channel( dev_config, ch_prop );
