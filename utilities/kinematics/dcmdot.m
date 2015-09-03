%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Poisson Equation 
%
%   Purpose: 
%       - Calculates the kinematic differential equation for the direction
%       cosine matrix ( Poisson Equation)
%
%   dcm_dot = dcmdot(dcm,omega)
%
%   Inputs: 
%       - dcm - 3x3 direction cosine matrix relating the rotation between
%       two frames. assumes row vector format b = a*dcm_a2b
%       - omega - 1x3 angular velocity between the two frames. 
%
%   Outputs: 
%       - dcm_dot - 3x3 derivative of direction cosine matrix 
%
%   Dependencies: 
%       - skew_matrix.m - calculates skew symmetric matrix
%
%   Author: 
%       - Shankar Kulumani 7 April 2013
%           - created for AAE590 PS10
%
%   References
%       - AAE590 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dcm_dot = dcmdot(dcm,omega)

dcm_dot = dcm*skew_matrix(omega);