%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Space 2-3-1 Differential Equation
%
%   Purpose: 
%       - Finds the rate of change of the Euler Space 2-3-1 angles 
%
%   theta_d = space231dot(theta,w)
%
%   Inputs: 
%       - theta - 3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] in radians
%       - w - 1x3 angular velocity vector in teh body frame components in
%       rad/sec
%
%   Outputs: 
%       - theta_d - 3 element vector with the 3 rotation angle derivatives. Same order
%       as m-file filename. theta = [first second third] rad/sec
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 4 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Omega Angle Rates pdf 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function theta_d = space231dot(theta,w)

ct1 = cos(theta(1));
st1 = sin(theta(1));
st2 = sin(theta(2));
ct2 = cos(theta(2));

theta_d = zeros(1,3);

theta_d(1) = (w(1)*ct1+w(3)*st1)*st2/ct2 + w(2);
theta_d(2) = -w(1)*st1 + w(3)*ct1;
theta_d(3) = (w(1)*ct1 + w(3)*st1)/ct2;