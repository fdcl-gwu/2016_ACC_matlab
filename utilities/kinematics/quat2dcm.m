%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quaternion to direction cosine matrix
%
%   Purpose: 
%       - Converts a 4 element quaternion into the corresponding direction
%       cosine matrix
%
%   [dcm] = quat2dcm(quat)
%
%   Inputs: 
%       - quat - Nx4 element quaternion where the last element is the scalar
%       parameter ( [e1 e2 e3 n] ) describing the rotation from frame a to
%       frame b
%
%   Outputs: 
%       - dcm - 3x3xN rotation matrix describing rotation from frame a to
%       frame b. Assumes row vector format b = a * dcm_a2b
%
%   Dependencies: 
%       - quatnorm.m - normalize a quaternion - vectorized
%
%   Author: 
%       - Shankar Kulumani 24 Jan 2013
%       - Shankar Kulumani 12 Feb 2013
%           - vectorized and quaternion norm check is implemented
%       - Shankar Kulumani 24 Feb 2013
%           - modified size check for vectorization
%
%   References
%       - AAE590 Lesson 7  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function [dcm] = quat2dcm(quat)

N  = size(quat,1);
% perform normalization check on inpute quaternion
quat = quatnorm(quat);

% quaternion is 4 element parameter where the 4th element is the scalar
% element
e1 = quat(:,1);
e2 = quat(:,2);
e3 = quat(:,3);
e4 = quat(:,4);

% form the quaternion products ahead of time
e1e1=e1.*e1;
e1e2=e1.*e2;
e1e3=e1.*e3;
e1e4=e1.*e4;

e2e2=e2.*e2;
e2e3=e2.*e3;
e2e4=e2.*e4;

e3e3=e3.*e3;
e3e4=e3.*e4;
  
e4e4=e4.*e4;

% dcm is for row element vectors b = a*dcm_a2b (b - 1x3 a-1x3)
dcm = zeros(3,3,N);

dcm(1,1,:) = 1-2*e2e2-2*e3e3;
dcm(1,2,:) = 2*(e1e2 - e3e4);
dcm(1,3,:) = 2*(e1e3 + e2e4);
dcm(2,1,:) = 2*(e1e2+e3e4);
dcm(2,2,:) = 1-2*e3e3-2*e1e1;
dcm(2,3,:) = 2*(e2e3-e1e4);
dcm(3,1,:) = 2*(e1e3 - e2e4);
dcm(3,2,:) = 2*(e2e3+e1e4);
dcm(3,3,:) = 1-2*e1e1-2*e2e2;