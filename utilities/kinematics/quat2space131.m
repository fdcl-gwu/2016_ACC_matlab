%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Quaternion to Euler Space 1-3-1
%
%   Purpose: 
%       - Convert a quaternion to the equivalaent Euler Space 1-3-1 sequence
%       about the body fixed axes.
%
%   function theta = quat2space131(quat)
%
%   Inputs: 
%       - quat - 1x4 quaternion describing the orientation of the body
%       frame wrt inertial frame. The 4th element is the scalar value.
%
%   Outputs: 
%       - theta - 3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] radians
%
%   Dependencies: 
%       - quat2dcm.m - convert quaternion to dcm
%       - dcm2space131.m - convert dcm to euler angles
%
%   Author: 
%       - Shankar Kulumani 2 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Lesson 9 
%       - H. Schaub and J. Junkins. Matlab toolbox for rigid body kinematics. Spaceflight mechanics 1999, pages 549?560, 1999.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function theta = quat2space131(quat)

dcm = quat2dcm(quat);

theta = dcm2space131(dcm);