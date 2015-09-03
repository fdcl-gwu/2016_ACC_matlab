%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Direction cosine matrix to simple rotation
%
%   Purpose:
%       - Convert direction cosine matrix to euler axis and angle
%
%   [lambda theta] = dcm2srt(C)
%
%   Inputs:
%       - C - 3x3 direction cosine matrix in row vector format b = a
%       *dcm_a2b
%
%   Outputs:
%       - lambda - euler axis of rotation 1x3 unit vector
%       - theta - euler angle of rotation in radians
%
%   Dependencies:
%       - none
%
%   Author:
%       - Shankar Kulumani 19 Jan 2013
%
%   References
%       - P. Hughes. Spacecraft attitude dynamics. Dover Publications, 2004.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [lambda theta] = dcm2srt(C)

C = C';

sigma = trace(C);

theta = acos(1/2*(sigma-1));

sin_theta = sin(theta);
lambda = zeros(1,3);

switch sigma
    case 3
        disp('Error: undefined')
    case -1
        disp('error')
    otherwise
        lambda(1) = 1/2*(C(2,3)-C(3,2))/sin_theta;
        lambda(2) = 1/2*(C(3,1)-C(1,3))/sin_theta;
        lambda(3) = 1/2*(C(1,2)-C(2,1))/sin_theta;
end