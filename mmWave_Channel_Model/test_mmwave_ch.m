dev_config.carrier_freq = 73e9;
dev_config.pos_tx = [0, 0, 0];
dev_config.pos_rx = [10, 0, 0];
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


simu_cnt = 3000;
%% test distance-los_nlos_power
dev_config.carrier_freq = 73e9;
dev_config.pos_rx = [10, 0, 0];

rx_pos_range = 5:10:200;
los_nlos_pdb_res = zeros(simu_cnt, length(rx_pos_range));
for pos_idx = 1:length(rx_pos_range)
    dev_config.pos_rx(1) = rx_pos_range(pos_idx);
    for sc = 1:simu_cnt
        [ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
            gen_mmwave_channel( dev_config, ch_prop );
        los_nlos_pdb = 10*log10(norm(mmwave_ch_sample.los_link, 'fro')^2/norm(mmwave_ch_sample.nlos_link, 'fro')^2);
        los_nlos_pdb_res(sc, pos_idx) = los_nlos_pdb;
        fprintf('  d = %.4f, sc = %d, pdb_res = %.4f\n', ...
            norm(dev_config.pos_rx - dev_config.pos_tx, 2), sc, los_nlos_pdb_res(sc, pos_idx));
    end
end

% save and plot results
save('mmwave_ch_pos.mat', 'los_nlos_pdb_res');
figure(1);
grid on;
plot_rx_pos_range = [1, 5, 10, 15, 19];
dist_legend = cell(length(plot_rx_pos_range), 1);
for pos_idx = 1:length(plot_rx_pos_range)
    dev_config.pos_rx(1) = rx_pos_range(plot_rx_pos_range(pos_idx));
    cdfplot(los_nlos_pdb_res(:, pos_idx));
    dist_legend{pos_idx} = sprintf('d = %.4f', norm(dev_config.pos_rx - dev_config.pos_tx, 2));
    hold on;
end
xlabel('LOS/NLOS PL (dB) Ratio');
ylabel('Probability')
legend(dist_legend);


%% test freq-los_nlos_power
dev_config.carrier_freq = 73e9;
dev_config.pos_rx = [50, 0, 0];

freq_range = 5:10:80;
los_nlos_pdb_res = zeros(simu_cnt, length(freq_range));
for freq_idx = 1:length(freq_range)
    dev_config.carrier_freq = freq_range(freq_idx) * 1e9;
    for sc = 1:simu_cnt
        [ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
            gen_mmwave_channel( dev_config, ch_prop );
        los_nlos_pdb = 10*log10(norm(mmwave_ch_sample.los_link, 'fro')^2/norm(mmwave_ch_sample.nlos_link, 'fro')^2);
        los_nlos_pdb_res(sc, freq_idx) = los_nlos_pdb;
        fprintf('  freq = %d GHz, sc = %d, pdb_res = %.4f\n', ...
            dev_config.carrier_freq/1e9, sc, los_nlos_pdb_res(sc, freq_idx));
    end
end

% save and plot results
save('mmwave_ch_freq.mat', 'los_nlos_pdb_res');
figure(2);
grid on;
plot_freq_range = [1, 3, 5, 7, 8];
dist_legend = cell(length(plot_freq_range), 1);
for freq_idx = 1:length(plot_freq_range)
    dev_config.carrier_freq = freq_range(plot_freq_range(freq_idx));
    cdfplot(los_nlos_pdb_res(:, freq_idx));
    dist_legend{freq_idx} = sprintf('freq = %d GHz', dev_config.carrier_freq/1e9);
    hold on;
end
xlabel('LOS/NLOS PL (dB) Ratio');
ylabel('Probability')
legend(dist_legend);
