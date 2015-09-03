function [Rd Wd Wd_dot]=tracking_command(t)
% Tracking Command Generation

phi=20*pi/180*sin(pi*t);
phidot=20*pi/180*pi*cos(pi*t);
phiddot=20*pi/180*pi*pi*-sin(pi*t);

theta=20*pi/180*cos(pi*t);
thetadot=20*pi/180*pi*-sin(pi*t);
thetaddot=20*pi/180*pi*pi*-cos(pi*t);

psi=0;
psidot=0;
psiddot=0;

Rd=[cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
Wd=[1 0 -sin(theta);
    0 cos(phi) sin(phi)*cos(theta);
    0 -sin(phi) cos(phi)*cos(theta)]*[phidot; thetadot; psidot];
Wd_dot=[ -psidot*thetadot*cos(theta);
    - phidot*(thetadot*sin(phi) - psidot*cos(phi)*cos(theta)) - psidot*thetadot*sin(phi)*sin(theta);
    - phidot*(thetadot*cos(phi) + psidot*cos(theta)*sin(phi)) - psidot*thetadot*cos(phi)*sin(theta)]+...
    [1 0 -sin(theta);
    0 cos(phi) sin(phi)*cos(theta);
    0 -sin(phi) cos(phi)*cos(theta)]*[phiddot; thetaddot; psiddot];

end