bagselect = rosbag("LocationB.bag");
rosbag info LocationB.bag;
bSel = select(bagselect,'Topic','/vectornav');
topics=bSel.AvailableTopics;

msgStructs = readMessages(bSel,'DataFormat','struct');
%msgStructs{1};
%writetable(struct2table(msgStructs{1}), 'LocationB.csv');

for i = 1:length(msgStructs)
    values = split(msgStructs{i}.Data, ',');
    if length(values) == 13
        % Convert strings to numbers
        angular_velocity_x = str2double(values{11});
        angular_velocity_y = str2double(values{12});
        cali_gyroz = split(values{13}, '*');
        angular_velocity_z = str2double(cali_gyroz{1});
        angular_velocity(i,:) = [angular_velocity_x, angular_velocity_y, angular_velocity_z];
    end
     if  length(values)<=13
         values = zeros(13,1);
     end
end

%One Plot of  gyro x, y, z vs. Time
figure('Name',"Angular Velocity Time Series", 'NumberTitle', 'off')
plot(linspace(1,length(msgStructs)/40,length(msgStructs)), angular_velocity(:,1),'-', 'color','r')
hold on
plot(linspace(1,length(msgStructs)/40,length(msgStructs)), angular_velocity(:,2),'-', 'color','g')
hold on
plot(linspace(1,length(msgStructs)/40,length(msgStructs)), angular_velocity(:,3),'-', 'color','b')
grid on

xlabel('Time (in secs)')
ylabel('Angular Velocity (rad/s)')
%legend ('GyroX','GyroY','GyroZ')
title('Angular Velocity Time Series')
hold off
saveas(gcf, 'OnePlot_LocationB.png')



% Allan variance - Angular Velocity - x
Fs = 40;
t0 = 1/Fs;

theta = cumsum(angular_velocity(:,1), 1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';

m = ceil(m); 
m = unique(m); 
tau = m*t0;
[avarFromFunc, tauFromFunc] = allanvar(angular_velocity(:,1), m, Fs);
adevFromFunc = sqrt(avarFromFunc);

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N_angularVelocity_X = 10^logN

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K_angularVelocity_X = 10^logK

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tauFromFunc); %tau = tauFromFunc
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B_angularVelocity_X = 10^logB
% Plot
tauN = 1;
lineN = N_angularVelocity_X ./ sqrt(tau);
% Plot
tauK = 3;
lineK = K_angularVelocity_X .* sqrt(tau/3);
% Plot
tauB = tau(i);
lineB = B_angularVelocity_X * scfB * ones(size(tau));
% Plot results
figure('Name',"Allan Deviation of Gyro X (With Noise Parameters)", 'NumberTitle', 'off')
loglog(tauFromFunc, adevFromFunc, tauFromFunc, lineN, '--', tauN, N_angularVelocity_X, 'o', tauFromFunc, lineK, '--', tauK, K_angularVelocity_X, 'o', tau, lineB, '--', tauB, scfB*B_angularVelocity_X, 'o');
legend('\sigma (rad/s)', '\sigma_N ((rad/s)sqrt{Hz})', '','\sigma_K ((rad/s)sqrt{Hz})','','\sigma_B (rad/s)')
title('Allan Deviation with Noise Parameters of Gyro X')
xlabel('\tau')
ylabel('\sigma(\tau)')
text(tauN, N_angularVelocity_X, 'N')
text(tauK, K_angularVelocity_X, 'K')
text(tauB, scfB*B_angularVelocity_X, '0.664B')
grid on
axis equal
saveas(gcf, 'LocationB_angularX.png')





% Allan variance - gyro y
Fs = 40; 
t0 = 1/Fs;
theta = cumsum(angular_velocity(:,2), 1)*t0;
maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); 
m = unique(m); 
tau = m*t0;
[avarFromFunc, tauFromFunc] = allanvar(angular_velocity(:,2), m, Fs);
adevFromFunc = sqrt(avarFromFunc);

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.

b = logadev(i) - slope*logtau(i);
% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N_angularVelocity_Y = 10^logN

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K_angularVelocity_Y = 10^logK

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tauFromFunc); %tau = tauFromFunc
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B_angularVelocity_Y = 10^logB

% Plot
tauN = 1;
lineN = N_angularVelocity_Y ./ sqrt(tau);
% Plot 
tauK = 3;
lineK = K_angularVelocity_Y .* sqrt(tau/3);
% Plot 
tauB = tau(i);
lineB = B_angularVelocity_Y * scfB * ones(size(tau));
% Plot results
figure('Name',"Allan Deviation of Gyro Y (with Noise Parameters)", 'NumberTitle', 'off')
loglog(tauFromFunc, adevFromFunc, tauFromFunc, lineN, '--', tauN, N_angularVelocity_Y, 'o', tauFromFunc, lineK, '--', tauK, K_angularVelocity_Y, 'o', tau, lineB, '--', tauB, scfB*B_angularVelocity_Y, 'o');
legend('\sigma (rad/s)', '\sigma_N ((rad/s)sqrt{Hz})', '','\sigma_K ((rad/s)sqrt{Hz})','','\sigma_B (rad/s)')
title('Allan Deviation with Noise Parameters of Gyro Y')
xlabel('\tau')
ylabel('\sigma(\tau)')
text(tauN, N_angularVelocity_Y, 'N')
text(tauK, K_angularVelocity_Y, 'K')
text(tauB, scfB*B_angularVelocity_Y, '0.664B')
grid on
axis equal
saveas(gcf, 'LocationB_angularY.png')


% Allan variance - gyro z
Fs = 40;
t0 = 1/Fs;
theta = cumsum(angular_velocity(:,3), 1)*t0;

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); 
m = unique(m); 
tau = m*t0;
angular_velocity(402372,3) = double(0.0);
angular_velocity(402371,3) = double(0.0);

[avarFromFunc, tauFromFunc] = allanvar(angular_velocity(:,3), m, Fs);
adevFromFunc = sqrt(avarFromFunc);
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N_angularVelocity_Z = 10^logN

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tauFromFunc);
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K_angularVelocity_Z = 10^logK

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tauFromFunc); %tau = tauFromFunc
logadev = log10(adevFromFunc);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);
% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B_angularVelocity_Z = 10^logB

% Plot
tauN = 1;
lineN = N_angularVelocity_Z ./ sqrt(tau);
% Plot
tauK = 3;
lineK = K_angularVelocity_Z .* sqrt(tau/3);
% Plot
tauB = tau(i);
lineB = B_angularVelocity_Z * scfB * ones(size(tau));
% Plot results
figure('Name',"Allan Deviation of Gyro Z (with Noise Parameters)", 'NumberTitle', 'off')
loglog(tauFromFunc, adevFromFunc, tauFromFunc, lineN, '--', tauN, N_angularVelocity_Z, 'o', tauFromFunc, lineK, '--', tauK, K_angularVelocity_Z, 'o', tau, lineB, '--', tauB, scfB*B_angularVelocity_Z, 'o');
legend('\sigma (rad/s)', '\sigma_N ((rad/s)sqrt{Hz})', '','\sigma_K ((rad/s)sqrt{Hz})','','\sigma_B (rad/s)')
title('Allan Deviation with Noise Parameters of Gyro Z')
xlabel('\tau')
ylabel('\sigma(\tau)')
text(tauN, N_angularVelocity_Z, 'N')
text(tauK, K_angularVelocity_Z, 'K')
text(tauB, scfB*B_angularVelocity_Z, '0.664B')
grid on
axis equal
saveas(gcf, 'LocationB_angularZ.png')

