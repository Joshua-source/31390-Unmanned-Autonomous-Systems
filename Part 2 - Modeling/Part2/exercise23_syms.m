clear
clc
close all
%% setup
syms phi theta psi p q r Ixx Iyy Izz tau_1 tau_2 tau_3 m g u1 kd dx dy dz ex ey

R = [cos(psi) * sin(theta), cos(psi)*sin(theta)*sin(phi)-...
    cos(phi)*sin(psi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);...
    cos(theta)*sin(psi), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),...
    cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);...
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

w = [p;q;r];

I = [Ixx,0,0; 0,Iyy,0; 0,0,Izz];

tau_B = [tau_1; tau_2; tau_3];

F_g = [0;0;-m*g];
F_B = [0;0;u1];
F_D = -kd*[dx;dy;dz];

%%

F_g+R*F_B+F_D

I \ (tau_B - cross(w, I*w))


%% Exercise 3.3
[0 -1 0; 1 0 0]*R*[ex ey 0].'