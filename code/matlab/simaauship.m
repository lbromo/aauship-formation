%% Simulation of aauship
% TODO nonlinear stuff for simulation model

clear all; clf;

%% Pre allocation of variables
N = 4000;
x = zeros(N,10);
x(1,:) = [0 0 0 0 0 2 0 0 0 0]';
xdot = zeros(N,10);
NED = zeros(N,2);

taus = [1 0 0 0 0.005]';
tau = repmat(taus',N,1);
% tau(:,1) = (1:N)/600;
taus = [12 0 0 0 0]';
tau(ceil(N/2)+1:N,:)  = repmat(taus',N/2,1);

%% Thrust allocation 
% lx1 = 0.41; lx2 = 0.18; lx3 = 0.48; lx4 = 0.48; ly3 = 0.05; ly4 = 0.05;
% lz3 = 0.05; lz4 = 0.05;
% a = atan(ly3/lx3);
% az = atan(lz3/lz3);
% T = [ 0 0 1 1;...
%       1 1 -sin(a) sin(a);...
%       -1 -1 0 0;...
%       0 0 sin(az)*lz3 sin(az*lz4);...
%       lx1 -lx2 -sin(a)*lx3 sin(a)*lx4];
% K = eye(4,4);
% K(3,3) = 0.2657/2;
% K(4,4) = 0.2657/2;
% uf = [0 0 15 15]'; % Thruster force vector [N]
% ta = T*K*(uf+[0 0 24.8350/2 24.8350/2]');

%% Waypoints
start = [100, 1000];
stop = [-1000,1000];
track = [20,0; 30,-40; 40,0; 40,30]; %1000,1000; 1000,2000; 0,2010];
track = [x(1:2,1)';track];
n = 1;
error = zeros(1,N);
integral = zeros(1,N);
derivative = zeros(1,N);
Kp = 0.05;
Ki = 0.5;
Kd = 10.4;
thrustdiff = zeros(1,N);
heading = zeros(N,1);
headingdesired = zeros(N,1);
%% Simulation
figure(1)
clf
hold on

for k = 1:N
%     x(k+1,:) = aauship(x(k,:)', ta); % Used fo thrust allocaiton testing

    % GNC
%     NED(k,:)
    [headingdesired(k), wp_reached] = wp_gen(track(n,:),track(n+1,:),NED(k,:)); % WP Gen
    if (wp_reached == 1)
        n = n+1;
        if n >= length(track)
            es = k;
            break
        end
    end

    tau(k,:)=[6 0 0 0 thrustdiff(k)];
    x(k+1,:) = aauship(x(k,:)', tau(k,:)');
    psi=x(k,5);
    Rz = [cos(psi) -sin(psi);
          sin(psi)  cos(psi)];
    if k ~=  1
    NED(k+1,:) = Rz*x(k,6:7)'*0.1 + NED(k-1,:)';
    heading(k) = x(k,10)*0.1 + heading(k-1);
    end
    
    
    error(k) = headingdesired(k) -heading(k);
    integral(k) = integral(k) + error(k);
    if k~=1
    derivative(k) = error(k) - error(k-1);
    end
    thrustdiff(k+1) = Kp*error(k) + Ki*integral(k) + Kd*derivative(k);
end

%% Plot the results
t = 0:0.1:N/10;
tt = 0.01:0.1:N/10;


% figure(1)
% clf
% subplot(2,1,1)

for k = 1:20:N
    ship(NED(k,2),NED(k,1),-x(k,5)+pi/2,'y')
end
% for k = 1:20:N
%     ship(NED(k,2),NED(k,1),pi/2-headingdesired(k),'y')
% end
hold on
plot(track(:,2),track(:,1),'b-o', NED(1:es,2),NED(1:es,1),'-r')
xlabel('Easting [m]');
ylabel('Northing [m]');
grid on
axis equal
hold off

% subplot(2,1,2)
% plot(tt,heading,tt,headingdesired)

% 
% figure(2)
% subplot(3,1,1)
% plot(t,x(:,6))
% ylabel('Surge speed [m/s]')
% subplot(3,1,2)
% plot(t,x(:,7))
% ylabel('Sway speed [m/s]')
% subplot(3,1,3)
% plot(t,x(:,10))
% ylabel('Yaw speed [rad/s]')
% xlabel('Time [s]')
% 
% figure(3)
% subplot(3,1,1)
% plot(t,x(:,3))
% ylabel('Rool angle [rad]')
% subplot(3,1,2)
% plot(t,x(:,4))
% ylabel('Pitch angle [rad]')
% subplot(3,1,3)
% plot(t,x(:,5))
% ylabel('Yaw angle [rad]')
% xlabel('Time [s]')

