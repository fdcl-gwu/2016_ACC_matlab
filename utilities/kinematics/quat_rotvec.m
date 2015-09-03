%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1 line code description
%
%   Purpose: 
%       - Converts a vector in frame a to the corresponding vector in frame
%       b given a quaternion
%
%    b = quat_rotvec(quat,a)
%
%   Inputs: 
%       - quat - 4 element quaternion where the last element is the scalar
%       parameter ( [e1 e2 e3 n] ) describing the rotation from frame a to
%       frame b
%
%   Outputs: 
%       - b - a vector rotated into frame b
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


function b = quat_rotvec(quat,a)

e = quat(1:3);
n = quat(4);

a_e_cross = cross(a,e);
b = a - 2*n*a_e_cross + 2*cross(e,-a_e_cross);
