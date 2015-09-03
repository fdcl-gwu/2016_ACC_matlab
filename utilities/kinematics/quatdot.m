%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quaternion differential equation
%
%   Purpose: 
%       - Calculates the quaternion differential equation in terms of
%       angular velocity in the body frame
%
%   [q_d] = quatdot(q, w)
%
%   Inputs: 
%       - q - 1x4 quaternion representing the orientation of the body wrt
%       to inertial frame. the last element is the scalar component
%       - w - 1x3 angular velocity vector in teh body frame components.
%
%   Outputs: 
%       - q_d - 1x4 quaternion derivative with components in either the
%       inertial or fixed frame
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 2 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Lesson 11  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [q_d] = quatdot(q, w)

E = [q(4) -q(3) q(2) q(1);...
    q(3) q(4) -q(1) q(2);...
    -q(2) q(1) q(4) q(3);...
    -q(1) -q(2) -q(3) q(4)];

w = [w 0];

q_d = 1/2*w*E';
