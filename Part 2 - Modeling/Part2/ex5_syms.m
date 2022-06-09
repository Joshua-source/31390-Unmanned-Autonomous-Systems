clear
clc
close all
%%

syms Rz Ry Rx psi theta phi dpsi dtheta dphi

Rz = [cos(psi), -sin(psi), 0;
       sin(psi), cos(psi), 0;
       0, 0, 1].';
Ry = [cos(theta), 0, sin(theta); 
       0, 1, 0;
       -sin(theta), 0, cos(theta)].';
Rx = [1, 0, 0;
       0, cos(phi), -sin(phi);
       0, sin(phi), cos(phi)].';
   
Rx*Ry*Rz*[0;0;dpsi] + Rx*Ry*[0; dtheta; 0] + Rx*[dphi; 0; 0]

Omega = [dphi; dtheta; dpsi];
A = [1, 0, -sin(theta); 
    0, cos(phi), sin(phi)*cos(theta);
    0, -sin(phi), cos(theta)*cos(phi)];

A*Omega


%%
% 
% syms Rz Ry Rx psi theta phi
% 
% assume(psi,'real')
% assume(theta,'real')
% assume(psi,'real')
% 
% Rz = [cos(psi), -sin(psi), 0;
%        sin(psi), cos(psi), 0;
%        0, 0, 1];
% Ry = [cos(theta), 0, sin(theta); 
%        0, 1, 0;
%        -sin(theta), 0, cos(theta)];
% Rx = [1, 0, 0;
%        0, cos(phi), -sin(phi);
%        0, sin(phi), cos(phi)];
% 
% xC = Rz*[1;0;0];
% zB = Rz*Ry*Rx*[0;0;1];
% 
% yB = cross(zB,xC)/(norm(cross(zB,xC)))
% 
% dotprod = dot(yB,xC)
% 
% %%
% syms k_f arm_length moment_constant motor_constant
% 
% allocation_matrix = [1 1 1 1
%     0 k_f*arm_length 0 -k_f*arm_length
%     -k_f*arm_length 0 k_f*arm_length 0
%     -moment_constant moment_constant -moment_constant moment_constant];
% 
% mix_matrix = motor_constant * allocation_matrix;