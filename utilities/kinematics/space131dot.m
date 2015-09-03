%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Space 1-3-1 Differential Equation
%
%   Purpose: 
%       - Finds the rate of change of the Euler Space 1-3-1 angles 
%
%   theta_d = space131dot(theta,w)
%
%   Inputs: 
%       - theta - 3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] rad
%       - w - 1x3 angular velocity vector in teh body frame components.
%
%   Outputs: 
%       - theta_d - 3 element vector with the 3 rotation angle derivatives. Same order
%       as m-file filename. theta = [first second third] rad/sec
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 2 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Omega Angle Rates pdf 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function theta_d = space131dot(theta,w)

c2 = cos(theta(2));
c1 = cos(theta(1));
s2 = sin(theta(2));
s1 = sin(theta(1));

theta_d(1) = w(1) + (w(2)*c1 -w(3)*s1)*c2/s2;
theta_d(2) = w(2)*s1+w(3)*c1;
theta_d(3) = (-w(2)*c1 +w(3)*s1)/s2;