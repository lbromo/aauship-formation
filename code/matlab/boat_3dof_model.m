%% 3 DOF model

M = [13 0 0; 0 13 -0.39; 0 -0.39 1.1068];
D = [2.86 0 0; 0 32.50 0.0926; 0 0.09750 0.2628];

A = [zeros(3) eye(3); zeros(3) -inv(M) * D];
B = [zeros(3); inv(M)];

C = eye(6);


ts= 0.1; % sample time

sys = ss(A,B,C,0);
sysd = c2d(sys,ts,'zoh');

% states:
% [N E psi u v r]

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
Q(4,4) = 0.5;
Q(5,5) = 0.5;
Q(6,6) = 2;

R = eye(3);
R(1,1) = 1/500;
R(2,2) = 1/500;
R(3,3) = 100;

K = lqr(A,B,Q,R,0);
P = eig(A-B*K);

K = place(A,B,P);

ts= 0.1; % sample time
sys_cl = ss(A-B*K,B,C,0);
sysd_cl = c2d(sys_cl,ts,'zoh');

% states:
% [N E psi u v r]

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