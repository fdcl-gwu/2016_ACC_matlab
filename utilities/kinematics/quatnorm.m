%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quaternion Normalize
%
%   Purpose: 
%       - Normalize quaternions to avoid problems - vectorized
%
%   quat_out = quatnorm(quat_in)
%
%   Inputs: 
%       - quat_in - Nx4 element quaternion where the last element is the scalar
%       parameter ( [e1 e2 e3 n] ) describing the rotation from frame a to
%       frame b
%
%   Outputs: 
%       - quat_out - Nx4 element normalized quaternion where the last element is the scalar
%       parameter ( [e1 e2 e3 n] ) describing the rotation from frame a to
%       frame b 
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 12 Feb 2013
%       - Shankar Kulumani 2 Mar 2013
%           - modified repmat command
%
%   References
%       - AAE590 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function quat_out = quatnorm(quat_in)

N = size(quat_in,1);

% Find the magnitude of each quaternion
quat_mag=sqrt(sum(quat_in.^2,2));

% resize quat mag
quat_mag = repmat(quat_mag,1,4);

% Divide each element of q by appropriate qmag
quat_out=quat_in./quat_mag;