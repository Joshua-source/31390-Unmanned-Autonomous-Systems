function omega = quaternion_omega(euler)

phi = euler(1);
theta = euler(2);
psi = euler(3);
phi_d = euler(4);
theta_d = euler(5);
psi_d = euler(6);

R_z = [cos(psi), sin(psi), 0;
       -sin(psi), cos(psi), 0;
       0, 0, 1];
R_y = [cos(theta), 0, -sin(theta); 
       0, 1, 0;
       sin(theta), 0, cos(theta)];
R_x = [1, 0, 0;
       0, cos(phi), sin(phi);
       0, -sin(phi), cos(phi)];
    
R_xq = rotm2quat(R_x);
R_yq = rotm2quat(R_y);
R_zq = rotm2quat(R_z);

omega = quat2rotm(R_xq .* R_yq .* R_zq) * [0; 0; psi_d] + quat2rotm(R_xq .* R_yq) * [0; theta_d; 0] + ...
    quat2rotm(R_xq) * [phi_d; 0; 0];


end