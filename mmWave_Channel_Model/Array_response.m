function [ vec_a ] = array_response( azmth, elvt, num_ant, spacing_ant, type_ant_array )
%SPATIAL_SIG calculates a spatial signature vector based on the inputs
%   Input parameter =========
%   azmth: azimuth angle
%   elvt: elevation angle
%   num_ant: number of the antenna
%   spacing_ant: normalized antenna spacing
%   type_ant_array: the type of the antenna array, e.g. UPA, ULA
%   Output parameter ========
%   vec_a: the spatial signature

% normalized antenna spacing multiplied by 2*pi
kd = 2*pi*spacing_ant;
j = sqrt(-1);

% generate the spatial signature
switch type_ant_array
    case 'ULA'
        vec_a = exp(j*kd*sin(azmth)*(0:1:(num_ant-1))') / sqrt(num_ant);
    case 'UPA'
        % generate the planar array: num_ant = ant_wd * ant_ht
        num_wd = round(sqrt(num_ant));
        num_wd = 2^round(log2(num_wd));
        num_ht = round(num_ant/num_wd);
        if num_ant ~= num_wd*num_ht
            error('Inappropriate antenna number for the UPA.');
        end
        
        vec_a = exp(j*kd*sin(azmth)*sin(elvt)*(0:1:(num_wd-1))') * ...
            exp(j*kd*cos(elvt)*(0:1:(num_ht-1))) / sqrt(num_ant);
        vec_a = reshape(vec_a, num_ant, 1);
    otherwise
        error('Unacceptable array type.');
end

end