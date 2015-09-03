%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Direction cosine matrix to Body Euler 1-3-1 Angles
%
%   Purpose: 
%       - Converts the row vector format direction cosine matrix
%       representing a rotation into the equivalent Body Euler 1-3-1Angles. 
%       Rotation about body fixed axes
%
%   theta = dcm2body131(dcm)
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
%       - Shankar Kulumani 17 Feb 2013
%           - vectorized the code
%
%   References
%       - AAE590 Lesson 9 
%       - H. Schaub and J. Junkins. Matlab toolbox for rigid body kinematics. Spaceflight mechanics 1999, pages 549?560, 1999.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function theta = dcm2body131(dcm)

theta(:,1) = atan2(dcm(3,1,:),dcm(2,1,:));
theta(:,2) = acos(dcm(1,1,:));
theta(:,3)= atan2(dcm(1,3,:),-dcm(1,2,:));