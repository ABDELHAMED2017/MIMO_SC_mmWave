dev_config.carrier_freq = 73e9;
dev_config.pos_tx = [0, 0, 20];
dev_config.pos_rx = [0, 0, 0];
dev_config.veloc_tx = [0, 0, 0];
dev_config.veloc_rx = [20, 15, 0];   % m/s
dev_config.num_ant_tx = 128;
dev_config.num_ant_rx = 8;
dev_config.spacing_ant_tx = 0.5; 
dev_config.spacing_ant_rx = 0.5;
dev_config.type_array_tx = 'UPA';
dev_config.type_array_rx = 'UPA';


%%  channel scattering properties
ch_prop.scenario = 2;
ch_prop.lambda_cluster = 1.9;
ch_prop.num_ray = 20;
% cluster power fraction
ch_prop.power_frac_tau = 3;
ch_prop.power_frac_sigma = 2;
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


simu_cnt = 3000;
%% test distance-los_nlos_power
dev_config.carrier_freq = 73e9;
dev_config.pos_rx = [0, 0, 0];
simu_time_range = 0:1e-2:100;

los_nlos_pdb_res = zeros(1, length(simu_time_range));
for simu_time_idx = 1:length(simu_time_range)
    simu_time = simu_time_range(simu_time_idx);
    [ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
        gen_mmwave_channel( dev_config, ch_prop, simu_time );
    los_nlos_pdb = 10*log10(norm(mmwave_ch_sample.los_link, 'fro')^2/norm(mmwave_ch_sample.nlos_link, 'fro')^2);
    los_nlos_pdb_res(simu_time_idx) = los_nlos_pdb;
    fprintf('  time = %.4f ms, pdb_res = %.4f\n', ...
        simu_time*1000, los_nlos_pdb_res(simu_time_idx));
end

% save and plot results
%save('mmwave_ch_pos.mat', 'los_nlos_pdb_res');
figure(1);
grid on;
plot(simu_time_range, los_nlos_pdb_res);
xlabel('Time')
ylabel('LOS/NLOS PL (dB) Ratio');

