%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Simple rotation theorem 
%
%   Purpose: 
%       - Converts a vector in frame a to the corresponding vector in frame
%       b given a euler axis and angle of rotation
%
%   b = srt_rotvec(lambda,theta,a)
%
%   Inputs: 
%       - lambda - euler axis of rotation 3x1 unit vector
%       - theta - euler angle of rotation in radians
%       - a - original vector in frame a
%
%   Outputs: 
%       - b - a vector rotated into frame b
%
%   Dependencies: 
%       - skew_matrix.m - creates skew symmetric matrix 
%
%   Author: 
%       - Shankar Kulumani 24 Jan 2013
%           - list revisions
%
%   References
%       - AAE590 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function b = srt_rotvec(lambda,theta,a)

cos_theta = cos(theta);
sin_theta = sin(theta);
a_skew = skew_matrix(a);

b = a*cos_theta - cross(a,lambda)*sin_theta + dot(a,lambda)*lambda*(1-cos_theta);