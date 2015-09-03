%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Axis/Angle to DCM
%
%   Purpose:
%       - Converts a given rotation vector and rotation angle into the
%       corresponding direction cosine matrix
%
%   dcm = srt2dcm(lambda,theta)
%
%   Inputs:
%       - lambda - 3 element unit vector of principle rotation axis
%       - theta - scalar rotation angle in radians
%
%   Outputs:
%       - dcm - direction cosine matrix to convert from inertial to fixed
%       frames assuming row vector notation (v_b = v_a * dcm_a2b)
%
%   Dependencies:
%       - skew_matrix.m - create skew symmetric matrix for cross product
%       operations
%
%   Author:
%       - Shankar Kulumani 19 Jan 2013
%
%   References
%       - AAE590 Lesson 5
%       - P. Hughes. Spacecraft attitude dynamics. Dover Publications, 2004.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dcm = srt2dcm(lambda,theta)

lambda = reshape(lambda,3,1);
cos_theta = cos(theta);
sin_theta = sin(theta);
lambda_skew = skew_matrix(lambda);

dcm = eye(3,3)*cos(theta) - sin_theta*lambda_skew + (1-cos_theta)*(lambda*lambda');

dcm = dcm';