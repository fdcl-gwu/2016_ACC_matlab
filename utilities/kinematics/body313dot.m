%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Euler Body 3-1-3 Differential Equation
%
%   Purpose: 
%       - Finds the rate of change of the Euler Body 3-1-3 angles 
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
%       - Shankar Kulumani 12 Feb 2013
%           - vectorized code
%
%   References
%       - AAE590 Omega Angle Rates pdf 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function theta_d = body313dot(theta,w)

c2 = cos(theta(:,2));
c3 = cos(theta(:,3));
s2 = sin(theta(:,2));
s3 = sin(theta(:,3));

w1 = w(:,1);
w2 = w(:,2);
w3 = w(:,3);

theta_d(:,1) = (w1.*s3 + w2.*c3)./s2;
theta_d(:,2) = w1.*c3-w2.*s3;
theta_d(:,3) = w3 -(w1.*s3+w2.*c3).*c2./s2;