function [ y ] = rand_laplace( m, n, mu, sigma, bound )
%RAND_LAPLACE generate i.i.d. laplacian random number drawn from laplacian distribution
%   Detailed explanation goes here
%   with mean mu and standard deviation sigma. 
%   mu      : mean
%   sigma   : standard deviation
%   [m, n]  : the dimension of y.
%   bound: the lower bound and upper bound of the samples
%   Default mu = 0, sigma = 1. 
%   For more information, refer to
%   http://en.wikipedia.org./wiki/Laplace_distribution

%%  Check inputs
switch nargin
    case {0, 1}
        error('At least two inputs are required');
    case 2
        mu = 0; sigma = 1; bound = [-inf, inf];
    case 3
        sigma = 1; bound = [-inf, inf];
    case 4
        bound = [-inf, inf];
    otherwise
end
lb = bound(1);
ub = bound(2);

%%  Generate Laplacian noise
b = sigma/sqrt(2);
y = zeros(m, n);
for row = 1:m
    for col = 1:n
        u = rand() - 0.5;
        y(row, col) = mu - b*sign(u).*log(1-2*abs(u));
        while y(row, col) > ub || y(row, col) < lb
            u = rand() - 0.5;
            y(row, col) = mu - b*sign(u).*log(1-2*abs(u));
        end
    end
end

end