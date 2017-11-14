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
dev_config.pos_rx = [10, 0, 0];

rx_pos_range = 5:10:200;
ch_power_res = zeros(simu_cnt, length(rx_pos_range));
for pos_idx = 1:length(rx_pos_range)
    dev_config.pos_rx(1) = rx_pos_range(pos_idx);
    for sc = 1:simu_cnt
        [ mmwave_ch_sample, los_vec_at, los_vec_ar, nlos_vec_at, nlos_vec_ar ] = ...
            gen_mmwave_channel( dev_config, ch_prop );
        mmwave_ch_mat = (mmwave_ch_sample.link_state == 1) * mmwave_ch_sample.los_link ...
            + (mmwave_ch_sample.link_state ~= 0) * mmwave_ch_sample.nlos_link;
        ch_power_res(sc, pos_idx) = 10*log10(norm(mmwave_ch_mat, 'fro')^2 / prod(size(mmwave_ch_mat)));
        fprintf('  d = %.4f, sc = %d, pdb_res = %.4f dB, link state %d\n', ...
            norm(dev_config.pos_rx - dev_config.pos_tx, 2), sc, ch_power_res(sc, pos_idx), mmwave_ch_sample.link_state);
    end
end

% save and plot results
save('mmwave_ch_pos.mat', 'ch_power_res');
figure(1);
grid on;
plot_rx_pos_range = [1, 5, 10, 15, 19];
dist_legend = cell(length(plot_rx_pos_range), 1);
for pos_idx = 1:length(plot_rx_pos_range)
    dev_config.pos_rx(1) = rx_pos_range(plot_rx_pos_range(pos_idx));
    cdfplot(-ch_power_res(:, pos_idx));
    dist_legend{pos_idx} = sprintf('d = %.4f', norm(dev_config.pos_rx - dev_config.pos_tx, 2));
    hold on;
end
xlabel('TX-RX PL (dB)');
ylabel('Probability')
legend(dist_legend);

