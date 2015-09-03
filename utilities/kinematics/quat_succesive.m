%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Successive Quaternion Rotations
%
%   Purpose: 
%       - Combines 2 successive quaternion represented rotations into a
%       single quaternion rotation
%
%   quat_A2B= quat_succesive(quat_A2B1,quat_B12B)
%
%   Inputs: 
%       - quat_A2B1 - nx4 quaternion representing first rotation. Frame A
%       to B1
%       - quat_B12B - nx4 quaternion representing second rotation. Frame B1
%       to B
%
%   Outputs: 
%       - quat_A2B - nx4 final quaternion rotation from frame A to B
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 26 Jan 2013
%       - Shankar Kulumani 5 Mar 2013
%           - went back to vector equations
%       - Shankar Kulumani 21 Mar 2013
%           - vectorized for many quaternions
%
%   References
%       - AAE590 Lesson 8 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function quat_A2B= quat_succesive(quat_A2B1,quat_B12B)

% e_A2B = zeros(1,3);
% n_A2B = 0;

e_A2B1 = quat_A2B1(:,1:3);
n_A2B1 = quat_A2B1(:,4);

e_B12B = quat_B12B(:,1:3);
n_B12B = quat_B12B(:,4);

e_A2B = e_A2B1.*repmat(n_B12B,1,3) + e_B12B.*repmat(n_A2B1,1,3) + cross(e_B12B,e_A2B1);
n_A2B = n_A2B1.*n_B12B - dot(e_A2B1,e_B12B,2);

quat_A2B = [e_A2B n_A2B];

% A = [n_A2B1 -e_A2B1(3) e_A2B1(2) e_A2B1(1);...
%     e_A2B1(3) n_A2B1 -e_A2B1(1) e_A2B1(2);...
%     -e_A2B1(2) e_A2B1(1) n_A2B1 e_A2B1(3);...
%     -e_A2B1(1) -e_A2B1(2) -e_A2B1(3) n_A2B1];
% 
% quat_A2B = A*quat_B12B';
% quat_A2B = quat_A2B';