%% 3 DOF model

M = [13 0 0; 0 13 -0.39; 0 -0.39 1.1068];
D = [2.86 0 0; 0 32.50 0.0926; 0 0.09750 0.2628];

A = [zeros(3) eye(3); zeros(3) -inv(M) * D];
B = [zeros(3); inv(M)];

C = eye(6);


ts= 0.1; % sample time

sys = ss(A,B,C,0);
sysd = c2d(sys,ts,'zoh');

x0 = [0 0 0 1 1 0];
figure(2)
t = 0:ts:400;
u = zeros(3,length(t));

[y,t,x] = lsim(sysd,u,t,x0);
subplot(2,1,1)
plot(t,x(:,1), t,x(:,2), t,x(:,3));
legend('surge pos', 'sway pos', 'yaw pos')

subplot(2,1,2)
plot(t,x(:,4), t,x(:,5), t,x(:,6));
legend('surge vel', 'sway vel', 'yaw vel')
title('lsim')