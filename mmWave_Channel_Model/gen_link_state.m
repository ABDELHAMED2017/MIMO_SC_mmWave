function [ link_state, outage_prob, los_prob, nlos_prob ] = gen_link_state(d)
%GEN_LINK_STATE
% d: distance between tx and rx
% link_state: 0, 1, 2
%   0-outage_prob: link outage
%   1-los_prob: LOS link
%   2-nlos_prob: NLOS link

%% outage probability 
a_out = 1/30;
b_out = 5.2;
outage_prob = max(0, 1-exp(-a_out*d + b_out));

%% LOS probability (d1/d2 model)
d1 = 20;
d2 = 66;
los_prob = min(d1/d, 1)*(1-exp(-d/d2)) + exp(-d/d2);
los_prob = (1 - outage_prob) * los_prob;
    
%% NLOS probability
nlos_prob = 1 - outage_prob - los_prob;

state_num = rand();
if state_num < outage_prob
    link_state = 0;
elseif state_num < outage_prob + los_prob
    link_state = 1;
else
    link_state = 2;
end

end

