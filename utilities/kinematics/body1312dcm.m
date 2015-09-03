%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Body Euler 1-3-1 Angles to direction cosine matrix
%
%   Purpose: 
%       - Converts the Body Euler 1-3-1 Angles representing a rotation into the equivalent  
%        row vector format direction cosine matrix
%
%   dcm = body1312dcm(theta)
%
%   Inputs: 
%       - theta - 3 element vector with the 3 rotation angles. Same order
%       as m-file filename. theta = [first second third] in radians
%
%   Outputs: 
%       - dcm - 3x3 rotation matrix assuming row vector format b = a*dcm
%
%   Dependencies: 
%       - none
%
%   Author: 
%       - Shankar Kulumani 4 Feb 2013
%           - list revisions
%
%   References
%       - AAE590 Lesson 9 
%       - H. Schaub and J. Junkins. Matlab toolbox for rigid body kinematics. Spaceflight mechanics 1999, pages 549?560, 1999.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dcm = body1312dcm(theta)

st1 = sin(theta(1));
st2 = sin(theta(2));
st3 = sin(theta(3));
ct1 = cos(theta(1));
ct2 = cos(theta(2));
ct3 = cos(theta(3));

dcm = zeros(3,3);

dcm(1,1) = ct2;
dcm(1,2) = -st2*ct3;
dcm(1,3) = st2*st3;
dcm(2,1) = ct1*st2;
dcm(2,2) = ct1*ct2*ct3-st3*st1;
dcm(2,3) = -ct1*ct2*st3-ct3*st1;
dcm(3,1) = st1*st2;
dcm(3,2) = st1*ct2*ct3+st3*ct1;
dcm(3,3) = -st1*ct2*st3+ct3*ct1;