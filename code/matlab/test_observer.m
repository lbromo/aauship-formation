%% 3 DOF model
clear all;
close all;
clc;
% states:
% [N E psi u v r]'
% u:
% [Fx Fy torque]'

% Rigid body mass matrix, page 53 fossen
% The boat is approx 13 kg. M(2,3), M(3,2) M(3,3) are calculated by Nick in openloopplots
M = [13 0 0; 0 13 -0.39; 0 -0.39 1.1068];
D = [2.86 0 0; 0 32.50 0.0926; 0 0.09750 0.2628]; % Calculated by Nick in openloopplots

% Fossen page 175, eq. 7.219
A = [zeros(3) eye(3); zeros(3) -inv(M) * D];
B = [zeros(3); inv(M)];

% We care bout [N E psi]'
C = [eye(3) zeros(3,3)];

% Continous to discrete
ts= 0.2; % sample time
sys = ss(A,B,C,0);
sysd = c2d(sys,ts,'zoh');

Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;

% Step discrete model
x0 = [0 0 0 0 0 0]';
t = 0:ts:400;
% u = zeros(3,length(t));
% u = [ones(1,length(t)); zeros(1,length(t)); zeros(1,length(t))];
u = [ones(1,length(t)); 0.5*ones(1,length(t)); zeros(1,length(t))];

% Simulation running
[y,t,x] = lsim(sysd,u,t,x0);

% % Representation
% figure(1);
% subplot(3,1,1)
% plot(t,y(:,1));
% title('Postitions w/o controller: Surge position (North)')
% subplot(3,1,2);
% plot(t,y(:,2));
% title('Postitions w/o controller: Sway position (East)')
% subplot(3,1,3);
% plot(t,y(:,3));
% title('Postitions w/o controller: Yaw Angle')
% axis([0 max(t) -pi pi]);

%% Adding Feedback
% poles = [-1.5,-0.25,-2.5,-0.5,-1.7,-1];
% K = place(A,B,poles);
% K = [eye(3) zeros(3,3)];

% LQR
Q = eye(6);
Q(1,1) = 1/40^2; %meters
Q(2,2) = 1/40^2; %meters
Q(3,3) = 1/(2*pi)^2; %radians?
Q(4,4) = 1/2^2; %m/s
Q(5,5) = 1/2^2; %m/s
Q(6,6) = 1/(pi/2)^2; %rads/s?

R = eye(3);
R(1,1) = 1/200^2;
R(2,2) = 1/200^2;
R(3,3) = 1/400^2; %?

K = lqr(A,B,Q,R);

ts= 0.2;
sys_cl = ss(A-B*K,B,C,0);
sysd_cl = c2d(sys_cl,ts,'zoh');

% Simulation running
[y_cl,t,x_cl] = lsim(sysd_cl,u,t,x0);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Kd = lqr(Ad,Bd,Q,R);
save('state_feedback', 'Ad', 'Bd', 'Cd', 'Kd', 'A', 'B', 'C', 'K')

% 
% % Representation
% figure(1);
% subplot(3,1,1)
% plot(t,y(:,1));
% title('Postitions w/o controller: Surge position (North)')
% subplot(3,1,2);
% plot(t,y(:,2));
% title('Postitions w/o controller: Sway position (East)')
% subplot(3,1,3);
% plot(t,y(:,3));
% title('Postitions w/o controller: Yaw Angle')
% axis([0 max(t) -pi pi]);
% 
% figure(2);
% plot(t,x(:,4), t,x(:,5), t,x(:,6));
% title('Velocities w/o controller')
% legend('surge vel', 'sway vel', 'yaw vel')
% 
% figure(3);
% scatter(x(:,2),x(:,1));
% title('Velocities w/o controller')

x = [0 0 0 0 0 0]';
ref = [1 1 0]';
x(1)
for i = 1:10000
  x(:,i+1) = A*x(:,i) + B*(ref - (K*x(:,i)));
end
