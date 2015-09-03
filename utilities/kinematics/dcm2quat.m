%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Direction cosine matrix to quaternion
%
%   Purpose:
%       - Converts a direction cosine matrix to equivalent quaternion
%
%   quat = dcm2quat(C)
%
%   Inputs:
%       - C - 3x3 direction cosine matrix in row vector format b =
%       a*dcm_a2b
%
%   Outputs:
%       - quat - quaternion with [e n] where e - 1x3 vector and n is the
%       scalar magnitude assumes row vector format
%
%   Dependencies:
%       - none
%
%   Author:
%       - Shankar Kulumani 19 Jan 2013
%       - Shankar Kulumani 24 Jan 2013
%           - modified for row vector format
%
%   References
%       - P. Hughes. Spacecraft attitude dynamics. Dover Publications, 2004.
%       - AAE590 Lesson 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function quat = dcm2quat(C)

tr = C(1,1)+C(2,2)+C(3,3)+1;

e = zeros(1,3);

n = sqrt(tr)*1/2;

e(1) = (C(3,2)- C(2,3))/4/n;
e(2) = (C(1,3)-C(3,1))/4/n;
e(3) = (C(2,1)-C(1,2))/4/n;

quat = [e n];

%
% [v,i] = max(b2);
% switch i
% 	case 1
% 		b(1) = sqrt(b2(1));
% 		b(2) = (C(2,3)-C(3,2))/4/b(1);
% 		b(3) = (C(3,1)-C(1,3))/4/b(1);
% 		b(4) = (C(1,2)-C(2,1))/4/b(1);
% 	case 2
% 		b(2) = sqrt(b2(2));
% 		b(1) = (C(2,3)-C(3,2))/4/b(2);
% 		if (b(1)<0)
% 			b(2) = -b(2);
% 			b(1) = -b(1);
% 		end
% 		b(3) = (C(1,2)+C(2,1))/4/b(2);
% 		b(4) = (C(3,1)+C(1,3))/4/b(2);
% 	case 3
% 		b(3) = sqrt(b2(3));
% 		b(1) = (C(3,1)-C(1,3))/4/b(3);
% 		if (b(1)<0)
% 			b(3) = -b(3);
% 			b(1) = -b(1);
% 		end
% 		b(2) = (C(1,2)+C(2,1))/4/b(3);
% 		b(4) = (C(2,3)+C(3,2))/4/b(3);
% 	case 4
% 		b(4) = sqrt(b2(4));
% 		b(1) = (C(1,2)-C(2,1))/4/b(4);
% 		if (b(1)<0)
% 			b(4) = -b(4);
% 			b(1) = -b(1);
% 		end
% 		b(2) = (C(3,1)+C(1,3))/4/b(4);
% 		b(3) = (C(2,3)+C(3,2))/4/b(4);
% end
% b = b';