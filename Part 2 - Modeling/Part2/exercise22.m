clear
clc
close all
%% DEFINE
r2d = 180/pi;
d2r = pi/180;

addpath("./lib");

%% CONSTANTS
k_f = 0.01; % [?]
b = 0.001; % [?]
m = 0.5; % [kg]
g = 9.81; % [m/s²]
L = 0.225; % [m]

D = diag([0.01, 0.01, 0.01]); % drag of UAV in air [Ns/m]
I = diag([3E-6, 3E-6, 1E-5]); % inertia [Nms²/rad]

T = 0; % elapsed time [s]

%% INITIAL CONDITIONS
p = [0,0,0]'; % position
dp = [0,0,0]'; % velocity
Th = [0,0,0]'; % orientation Euler
dTh = [0,0,0]'; % angular velocity

%% SET SIM VALUES

quat = false;

Omega = [0,10000,0,10000]'; % angular speed of four propellers

dt = 0.1; % time increment [s]
start_time = 0;
end_time = 1;
time = start_time:dt:end_time;

p_vec = zeros(3,length(time));
Th_vec = zeros(3,length(time));

% filename for storing pictures
if quat
    filename = sprintf('QUAT-%d-%d-%d-%d', Omega(1), Omega(2), Omega(3), Omega(4));
else
    filename = sprintf('%d-%d-%d-%d', Omega(1), Omega(2), Omega(3), Omega(4));
end

path = '~/Desktop/';

%% Simulation
i = 0;
for t = time
    i = i + 1;
    %% Current state
    % rotation of body fixed frame w.r.t. intertial frame
    b2i = rot_rpy(Th);
    R = b2i';
    quat_rot = quaternion_rpy(Th)';     
    
    
    % Forces
    F_B = k_f*[0,0,sum(Omega.^2)]';
    F_D = -D*dp;
    F_g = -[0,0,m*g]';
    
    % linear acceleration
    if quat
        ddp = 1/m*(F_g + rotate_vec_quat(quat_rot, F_B) + F_D);
    else
        ddp = 1/m*(F_g + R*F_B + F_D);
    end
   
    % rotational velocity of body fixed frame
%     omega = EulerParam(Th(1),Th(2))*dTh;
    omega = quaternion_omega([Th(1), Th(2), Th(3), dTh(1), dTh(2), dTh(3)]);
    % angular acceleration
    domega = I \ (getTauB(L, k_f, b, Omega) - cross(omega,I*omega));
    
    
    %% Update variables
    % position
    dp = dp + dt * ddp;
    p = p + dt * dp;
    
    % rotation
    omega = omega + dt * domega;
    dTh = EulerParam(Th(1),Th(2)) \ omega;
    Th = Th + dt * dTh;
%     Th = normalizeAng(Th);
    
    %% Append to vectors
    p_vec(:,i) = p;
    Th_vec(:,i) = Th;
end

%% PLOT
figure(1)
plot(time, p_vec(1,:), 'LineWidth', 2)
hold on
plot(time, p_vec(2,:), 'LineWidth', 2)
plot(time, p_vec(3,:), 'LineWidth', 2)
grid on
xlabel('Time [s]')
ylabel('Position [m]')
legend('x position', 'y position', 'z position', 'Location', 'northwest')

saveas(gcf,strcat(path, 'position_', filename), 'epsc');

figure(2)
plot(time, Th_vec(1,:), 'LineWidth', 2)
hold on
plot(time, Th_vec(2,:), 'LineWidth', 2)
plot(time, Th_vec(3,:), 'LineWidth', 2)
grid on
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('\phi', '\theta', '\psi', 'Location', 'northwest')

saveas(gcf,strcat(path, 'angle_', filename), 'epsc');

%% FUNCTIONS
function dTh2omega = EulerParam(phi, theta)
    dTh2omega = [1, 0, -sin(theta);
                 0, cos(phi), cos(theta)*sin(phi);
                 0, -sin(phi), cos(theta)*cos(phi)];
end

function tau_B = getTauB(L,k,b,Omega)
    tau_B = [L*k*(Omega(1)^2 - Omega(3)^2);
             L*k*(Omega(2)^2 - Omega(4)^2);
             b*(Omega(1)^2-Omega(2)^2+Omega(3)^2-Omega(4)^2)];
end

function ang = normalizeAng(ang)
    ang = atan2(sin(ang), cos(ang));
end