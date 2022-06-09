function quat_body2inertial = quaternion_rpy(euler)

phi = euler(1)
theta = euler(2)
psi = euler(3)

R_z = [cos(psi), sin(psi), 0;
       -sin(psi), cos(psi), 0;
       0, 0, 1]
R_y = [cos(theta), 0, -sin(theta); 
       0, 1, 0;
       sin(theta), 0, cos(theta)]
R_x = [1, 0, 0;
       0, cos(phi), sin(phi);
       0, -sin(phi), cos(phi)]
    
inertial2body = R_x*R_y*R_z;

quat_body2inertial = rotm2quat(inertial2body)';

% This is rotation XYZ
% qw = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
% qx = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
% qy = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
% qz = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);


% quat_body2inertial = [qw; qx; qy; qz]


end