%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quaternion to Euler Axis/Angle
%
%   Purpose: 
%       - Converts a quaternion to the equivalent euler axis and angle
%       describing the rotation
%
%   [lambda theta] = quat2srt(quat)
%
%   Inputs: 
%       - quat - 4 element quaternion where the last element is the scalar
%       parameter ( [e1 e2 e3 n] ) describing the rotation from frame a to
%       frame b
%
%   Outputs: 
%       - lambda - euler axis of rotation 3x1 unit vector describing
%       rotation from frame a to frame b
%       - theta - euler angle of rotation in radians
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 24 Jan 2013
%           - list revisions
%
%   References
%       - AAE590 Lesson 7  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [lambda theta] = quat2srt(quat)

e = quat(1:3);
n = quat(4);

lambda = zeros(3,1);

lambda = e/norm(e);
theta = 2*acos(n);