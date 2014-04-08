clear all
for fighandle = findobj('Type','figure')', clf(fighandle), end

%% Data files
logpath = '/afs/ies.auc.dk/group/14gr1034/public_html/tests/';
testname = 'crashtest';
% fid = fopen('busroute/mbus5/gpsdata141212.txt'); % reduced GPS
% fidr = fopen('busroute/gpsdata141212.txt'); % all GPS
% adata = load('busroute/mbus5/accdata141212.csv'); % Accelerometer outputs

%% Data files
gps1file = fopen([logpath,testname,'/gps1.log']);
imudata = load([logpath,testname,'/imu.log']);
echofile = fopen([logpath,testname,'/echo.log']);
starttime = imudata(1,13); % Earliest timestamp


%% Reading GPS data
line = textscan(gps1file,'%f,%c,%f,%c,%f,%c,%f,%f');
time = line{1};
nmealat = line{3};
latsign = line{4};
nmealon = line{5};
lonsign = line{6};
nmeaspeed = line{7};
walltime = line{8};

%% Converts the latitude an longitide to decimal coordinates
[pos] = nmea2decimal({nmealat,latsign,nmealon,lonsign});
lat = pos(1,:);
lon = pos(2,:);

figure(1)
plot(lon,lat,'.r')
plot_google_map('maptype','satellite')
title('WGS84')


%%
latrad = lat*pi/180;
lonrad = lon*pi/180;
% hei = gpsdata(:,3);
N = length(lat);
hei=zeros(N,1);
x=zeros(N,1);
y=zeros(N,1);
z=zeros(N,1);
for kk = 1:N
    %[x(kk) y(kk) z(kk)] = wgs842ecef(latrad(kk),lonrad(kk),0);
    [x(kk) y(kk) z(kk)] = geodetic2ecef(latrad(kk),lonrad(kk),hei(kk),referenceEllipsoid('wgs84'));
end

%% Transform
%index = 4;
%meanlat = latrad(1);
%meanlon = lonrad(1);
 meanlat = 57.015179789287792*pi/180;
 meanlon = 9.985062449450744*pi/180;
meanhei = hei(1);
% [a b c]=wgs842ecef(meanlat,meanlon,meanhei);
[a b c]=geodetic2ecef(meanlat,meanlon,meanhei,referenceEllipsoid('wgs84'));
% plot3(a,b,c,'r*')
R_e2t = [-sin(meanlat)*cos(meanlon) -sin(meanlat)*sin(meanlon) cos(meanlat);...
    -sin(meanlon) cos(meanlon) 0;...
    -cos(meanlat)*cos(meanlon) -cos(meanlat)*sin(meanlon) -sin(meanlat)];

T = zeros(3,N);
for kk = 1:N
    T(:,kk) = R_e2t*([x(kk);y(kk);z(kk)]-[a;b;c]);
end
T = T';

figure(1)
% T(300:360,:) = 0
plot(T(:,2),T(:,1))
title('Raw GPS log (localframe)')



%%%%

%%
latradr = lat*pi/180;
lonradr = lon*pi/180;
% hei = gpsdata(:,3);
N = length(lat);
heir=zeros(N,1);
xr=zeros(N,1);
yr=zeros(N,1);
zr=zeros(N,1);
for kk = 1:N
    %[x(kk) y(kk) z(kk)] = wgs842ecef(latrad(kk),lonrad(kk),0);
    [xr(kk) yr(kk) zr(kk)] = geodetic2ecef(latradr(kk),lonradr(kk),heir(kk),referenceEllipsoid('wgs84'));
end

%% Transform
Tr = zeros(3,N);
for kk = 1:N
    Tr(:,kk) = R_e2t*([xr(kk);yr(kk);zr(kk)]-[a;b;c]);
end
Tr = Tr';


%% ADIS16405 Inertial Measurement Unit
supply = imudata(:,1)*0.002418; % Scale 2.418 mV
gyro = imudata(:,2:4)*0.05; % Scale 0.05 degrees/sec
accl = (imudata(:,5:7)*0.00333)*9.82;   %/333)*9.82; % Scale 3.33 mg (g is gravity, that is g-force)
magn = imudata(:,8:10)*0.0005; % 0.5 mgauss
temp = imudata(:,11)*0.14; % 0.14 degrees celcius 
aux_adc = imudata(:,12)*0.806; % 0.806 mV
imutime = imudata(:,13)-starttime; % Seconds since epoch on HLI

figure(2)
subplot(4,1,1)
plot(imutime, gyro)
title('Gyrometer')
ylabel('[degrees/sec]')
legend('X','Y','Z')

subplot(4,1,2)
plot(imutime, accl)
title('Accelerometer')
ylabel('[m/s^2]')

subplot(4,1,3)
plot(imutime, magn)
title('Magnetometer')
ylabel('[gauss]')

subplot(4,1,4)
plot(imutime, imudata(:,[1 11:12]))
title('Supply, temperature and ADC of the ADIS16405 IMU')
ylabel('[V, degC, V]')
xlabel('Time [s]')
legend('Supply','Temp','ADC')

%% Heading
% X_H = X*cos(pitch) + Y*sin(roll)*sin(pitch) - Z*cos(roll)*sin(pitch)
% Y_H = Y*cos(roll) + Z*sin(roll)
% Azimuth = atan2 (Y_H / X_H )

heading = atan2(-magn(:,2),magn(:,1))*180/pi;
figure(3)
% subplot(2,1,1)
plot(imutime,heading)
% subplot(2,1,1)
% plot(imutime,Azimuth)

%% Echosounder data
echo.depth.value=NaN;
echo.depth.timestamp=NaN;
echo.temperature.value=NaN;
echo.temperature.timestamp=NaN;

ii=1;

filenotdone = 1;
while(filenotdone > 0)
    % scan single line into matrix
    [str, count] = fscanf(echofile, '%[^\n]', 1);
    % skip over \n
    [strnl, count] = fscanf(echofile, '%[\n]', 1);
    % make sure we stop the while when EOF
    if(count == 0)
        filenotdone = 0;
    end
    
    delim = findstr(',',str);
    
    % detph data
    if isempty(findstr(str,'$SDDPT,'))~=1
        ii=ii+1;
        s = sscanf(str(delim(1)+1:delim(2)-1), '%f');
        ss = sscanf(str(delim(3)+1:length(str)), '%f');
        if(s)
            echo.depth.value(ii) = s;
            echo.depth.timestamp(ii) = ss;
        else
            echo.depth.value(ii) = NaN;
            echo.depth.timestamp(ii) = NaN;
        end
    end
    
    % temperature data
    if isempty(findstr(str,'$SDMTW,'))~=1
        s = sscanf(str(delim(1)+1:delim(2)-1), '%f');
        ss = sscanf(str(delim(3)+1:length(str)), '%f');
        if(s)
            echo.temperature.value(ii) = s;
            echo.temperature.timestamp(ii) = ss;
        else
            echo.temperature.value(ii) = NaN;
            echo.temperature.timestamp(ii) = NaN;
        end
    end
    
end
figure(4)
plot(echo.depth.timestamp,echo.depth.value,echo.temperature.timestamp',echo.temperature.value)
xlabel('Timestamp [s]')
ylabel('Depth [m] / Temperature [degree C]')
legend('Depth','Temperature')