%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Axis/Angle to Quaternion
%
%   Purpose:
%       - Converts a given rotation vector and rotation angle into the
%       corresponding quaternion
%
%   dcm = srt2quat(lambda,theta)
%
%   Inputs:
%       - lambda - 3 element unit vector of principle rotation axis
%       - theta - scalar rotation angle in radians
%
%   Outputs:
%       - quat - quaternion representation of rotation (a to b )
%
%   Dependencies:
%       - none
%
%   Author:
%       - Shankar Kulumani 19 Jan 2013
%
%   References
%       - AAE590 Lesson 7
%       - P. Hughes. Spacecraft attitude dynamics. Dover Publications, 2004.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function quat = srt2quat(lambda,theta)


e = lambda*sin(theta/2);

n = cos(theta/2);

quat = [e n];