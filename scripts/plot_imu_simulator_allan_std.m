clearvars;
close all;
clc;

%% Data import
% Read IMU data from csv file
run("read_imu_simulator_csv_data.m");

%% Setup
% IMU sample frequency and sample time
Fs          = 125;
dT          = 1 / Fs;
data.rate   = Fs;

% Generate cluster time (tau)
maxNumM = 50;
L       = size(accX, 1);
maxM    = 2.^floor(log2(L/2));
m       = logspace(0.1, log10(maxM), maxNumM).';
m       = ceil(m); % m must be an integer.
m       = unique(m); % Remove duplicates.
tau     = m*dT;

%% Compute Allan standard deviations (ASD) for Accelerometer XYZ
data.freq   = accX;
asd         = allan(data, tau);
tauX        = asd.tau1;
asdRealX    = asd.sig2;

data.freq   = accY;
asd         = allan(data, tau);
tauY        = asd.tau1;
asdRealY    = asd.osig;

data.freq   = accZ;
asd         = allan(data, tau);
tauZ        = asd.tau1;
asdRealZ    = asd.osig;

%% ASD Models Plot
figure;
loglog(gca, tauX, asdRealX, 'LineWidth', 1); grid on; hold on;
loglog(gca, tauY, asdRealY, 'LineWidth', 1);
loglog(gca, tauZ, asdRealZ, 'LineWidth', 1);
xlim([10^(-1), 10^4]);
ylim([1e-4, 1e-2]);

ylabel('ASD \sigma(\tau) / (m/s^2)')
xlabel('Cluster Time \tau / s');
title('Allan Standard Deviation - STIM300 Accelerometer');

set(gcf,'color','w');

%% Extract Allan Parameters
[Nx, Bx, Kx, Tbx] = extract_allan_parameters(tau, asdRealX', 'Acc-X', 'Acc');
[Ny, By, Ky, Tby] = extract_allan_parameters(tau, asdRealY', 'Acc-Y', 'Acc');
[Nz, Bz, Kz, Tbz] = extract_allan_parameters(tau, asdRealZ', 'Acc-Z', 'Acc');

%% Compute Allan standard deviations (ASD) for Gyroscope XYZ
data.freq   = gyroX;
asd         = allan(data, tau);
tauX        = asd.tau1;
asdRealX    = asd.sig2;

data.freq   = gyroY;
asd         = allan(data, tau);
tauY        = asd.tau1;
asdRealY    = asd.osig;

data.freq   = gyroZ;
asd         = allan(data, tau);
tauZ        = asd.tau1;
asdRealZ    = asd.osig;

%% ASD Models Plot
figure;
loglog(gca, tauX, asdRealX, 'LineWidth', 1); grid on; hold on;
loglog(gca, tauY, asdRealY, 'LineWidth', 1);
loglog(gca, tauZ, asdRealZ, 'LineWidth', 1);
xlim([10^(-1), 10^4]);
% ylim([1e-4, 1e-2]);

ylabel('ASD \sigma(\tau) / (rad/sqrt(Hz))')
xlabel('Cluster Time \tau / s');
title('Allan Standard Deviation - STIM300 Gyroscope');

set(gcf,'color','w');

%% Extract Allan Gyroscope Parameters
extract_allan_parameters(tau, asdRealX', 'Gyro-X', 'Gyro');
extract_allan_parameters(tau, asdRealY', 'Gyro-Y', 'Gyro');
extract_allan_parameters(tau, asdRealZ', 'Gyro-Z', 'Gyro');