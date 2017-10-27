function [ los_prob ] = los_probability(d, scenario)
%LOS_PROBABILITY 
% d: distance between tx and rx
% scenario: variable that contains information about the use-case scenario,
% it assumes the values:
% - scenario==1  ==> 'Open square'
% - scenario==2  ==> 'Street Canyon'
% - scenario==3  ==> 'Indoor Office'
% - scenario==4  ==> 'Shopping mall'
% return the probability if los exists

if (scenario==1 || scenario==2)
    los_prob = min(20/d,1)*(1-exp(-d/39))+exp(-d/39);
    
elseif (scenario==3 || scenario==4)
    if d<1.2
        los_prob = 1;
    elseif d>2 && d<6.5
        los_prob = exp(-(d-1.2)/4.7);
    elseif d>=6.5
        los_prob = 0.32*exp(-(d-6.5)/32.6);
    end
else
    error('ERROR: INVALID SCENARIO');
end
end

