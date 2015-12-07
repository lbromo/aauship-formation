%% 3 DOF model

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
C = eye(6);

ts= 0.2; % sample time
sys = ss(A,B,C,0);
sysd = c2d(sys,ts,'zoh');

Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;
    
% Step discrete model
x0 = [0 0 0 1 1 0]';
figure(1)
t = 0:ts:400;
u = zeros(3,length(t));

[y,t,x] = lsim(sysd,u,t,x0);
subplot(2,1,1)
plot(t,x(:,1), t,x(:,2), t,x(:,3));
title('Postitions w/o controller')
legend('surge pos', 'sway pos', 'yaw pos')

subplot(2,1,2)
plot(t,x(:,4), t,x(:,5), t,x(:,6));
title('Velocities w/o controller')
legend('surge vel', 'sway vel', 'yaw vel')
%% LQR
Q = eye(6);
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 100;
Q(4,4) = 0.25;
Q(5,5) = 0.25;
Q(6,6) = 1;

R = eye(3);
R(1,1) = 1/500;
R(2,2) = 1/500;
R(3,3) = 1;

LQR = lqr(A,B,Q,R);
%Poles = eig(A-B*K);

%K = place(A,B,Poles);

ts= 0.1;
sys_cl = ss(A-B*LQR,B,C,0);
sysd_cl = c2d(sys_cl,ts,'zoh');

% Step cloased loop system
x0 = [0 0 0 1 1 0]';
figure(2)
t = 0:ts:400;
u = zeros(3,length(t));

[y,t,x] = lsim(sysd_cl,u,t,x0);
subplot(2,1,1)
plot(t,x(:,1), t,x(:,2), t,x(:,3));
title('Postitions w controller')
legend('surge pos', 'sway pos', 'yaw pos')
subplot(2,1,2)
plot(t,x(:,4), t,x(:,5), t,x(:,6));
title('Velocities w controller')
legend('surge vel', 'sway vel', 'yaw vel')

%% Save stuff
save('3_dof', 'Ad', 'Bd', 'Cd', 'LQR')
