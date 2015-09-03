%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Direction cosine matrix to Euler Body 2-1-2 Angles
%
%   Purpose: 
%       - Converts the row vector format direction cosine matrix
%       representing a rotation into the equivalent Euler Body 2-1-2 Angles 
%       about body fixed axes 
%
%   theta = dcm2body212(dcm)
%
%   Inputs: 
%       - dcm - 3x3xN rotation matrix assuming row vector format b = a*dcm
%
%   Outputs: 
%       - theta - Nx3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] in radians
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 26 Jan 2013
%       - Shankar Kulumani 20 Mar 2013
%           - vectorized the code
%
%   References
%       - AAE590 Lesson 9 
%       - H. Schaub and J. Junkins. Matlab toolbox for rigid body kinematics. Spaceflight mechanics 1999, pages 549?560, 1999.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta = dcm2body212(dcm)

theta(:,1) = atan2(dcm(1,2,:),dcm(3,2,:));
theta(:,2) = acos(dcm(2,2,:));
theta(:,3)= atan2(dcm(2,1,:),-dcm(2,3,:));

