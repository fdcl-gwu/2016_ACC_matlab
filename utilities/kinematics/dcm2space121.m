%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Direction cosine matrix to Space Euler 1-2-1 Angles
%
%   Purpose: 
%       - Converts the row vector format direction cosine matrix
%       representing a rotation into the equivalent Space Euler 1-2-1 Angles. 
%       Rotation about space fixed axes
%
%   theta = dcm2space121(dcm)
%
%   Inputs: 
%       - dcm - 3x3 rotation matrix assuming row vector format b = a*dcm
%
%   Outputs: 
%       - theta - 3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] rad
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 2 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Lesson 9 
%       - H. Schaub and J. Junkins. Matlab toolbox for rigid body kinematics. Spaceflight mechanics 1999, pages 549?560, 1999.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function theta = dcm2space121(dcm)

theta(1) = atan2(dcm(1,2),dcm(1,3));
theta(2) = acos(dcm(1,1));
theta(3) = atan2(dcm(2,1),-dcm(3,1));