%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Body 1-3-1 Differential Equation
%
%   Purpose: 
%       - Finds the rate of change of the Euler Body 1-3-1 angles 
%
%   theta_d = body131dot(theta,w)
%
%   Inputs: 
%       - theta - Nx3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] rad
%       - w - Nx3 angular velocity vector in teh body frame components in
%       rad/sec
%
%   Outputs: 
%       - theta_d - Nx3 element vector with the 3 rotation angle derivatives. Same order
%       as m-file filename. theta = [first second third] rad/sec
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 2 Feb 2013
%       - Shankar Kulumani 17 Feb 2013
%           - vecotrized 
%
%   References
%       - AAE590 Omega Angle Rates pdf 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function theta_d = body131dot(theta,w)

c2 = cos(theta(:,2));
c3 = cos(theta(:,3));
s2 = sin(theta(:,2));
s3 = sin(theta(:,3));

w1 = w(:,1);
w2 = w(:,2);
w3 = w(:,3);

theta_d(:,1) = (-w2.*c3 + w3.*s3)./s2;
theta_d(:,2) = w2.*s3+w3.*c3;
theta_d(:,3) = w1 +(w2.*c3-w3.*s3).*c2./s2;