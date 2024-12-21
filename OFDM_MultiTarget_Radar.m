% OFDM-based Multi-Target Radar System Implementation
% This script implements an OFDM-based radar system for multiple target detection

%% System Configuration
% Basic radar parameters
radar.fc = 24e9;          % Carrier frequency (24 GHz)
radar.B = 100e6;          % Bandwidth (100 MHz)
radar.fs = radar.B;       % Sampling frequency equals bandwidth

% OFDM parameters
ofdm.Nsc = 1024;         % Number of subcarriers
ofdm.df = radar.B/ofdm.Nsc;  % Subcarrier spacing
ofdm.Tsym = 1/ofdm.df;   % OFDM symbol duration
ofdm.Ncp = 16;           % Cyclic prefix length
ofdm.Tcp = ofdm.Ncp/radar.fs;  % CP duration
ofdm.Tofdm = ofdm.Tsym + ofdm.Tcp;  % Total OFDM symbol duration
ofdm.Nofdm = ofdm.Nsc + ofdm.Ncp;   % Samples per OFDM symbol
ofdm.M = 128;            % Number of OFDM symbols to transmit

% Modulation parameters
mod.bps = 4;             % Bits per symbol (16-QAM)
mod.K = 2^mod.bps;       % Modulation order

%% Signal Generation
% Generate random binary data
dataTx = randi([0,1], [ofdm.Nsc*mod.bps ofdm.M]);

% QAM modulation
qamTx = qammod(dataTx, mod.K, 'InputType', 'bit', 'UnitAveragePower', true);

% OFDM modulation
ofdmTx = ofdmmod(qamTx, ofdm.Nsc, ofdm.Ncp);

% Reshape into matrix (each column is one OFDM symbol)
xofdm = reshape(ofdmTx, ofdm.Nofdm, ofdm.M);

%% Signal Reception and Processing for Multiple Targets
% Receive signal after multiple target reflections
yofdmr = receiveSignal_multiTarget(xofdm, radar.fs, radar.fc, ofdm.Tofdm);

% Reshape and demodulate OFDM
yofdmr1 = reshape(yofdmr, ofdm.Nofdm*ofdm.M, 1);
Yofdmr = ofdmdemod(yofdmr1, ofdm.Nsc, ofdm.Ncp);

% QAM Demodulation for signal decoding
dataRx = qamdemod(Yofdmr, mod.K, 'OutputType', 'bit', 'UnitAveragePower', true);

% Calculate bit error rate
[numErrors, ber] = biterr(dataTx(:), dataRx(:));
fprintf('Bit Error Rate: %e\nNumber of Errors: %d\n', ber, numErrors);

% Calculate channel response
Zofdmr = Yofdmr./qamTx;

%% Range-Doppler Processing using MUSIC Algorithm
% MUSIC algorithm parameters
music.nsignals = 3;  % Number of expected targets
music.nrange_bins = 128;  % Increased for better resolution

% Reshape channel response for MUSIC processing
Z = reshape(Zofdmr, ofdm.Nsc, ofdm.M);

% Compute correlation matrix and its eigendecomposition
R = (Z * Z') / ofdm.M;
[V, D] = eig(R, 'vector');
[~, idx] = sort(abs(D), 'descend');
V = V(:, idx);

% Separate signal and noise subspaces
Vn = V(:, music.nsignals+1:end);  % Noise subspace
VnVn = Vn * Vn';  % Pre-compute noise projection matrix

% Define speed of light and wavelength
c = 3e8;  % Speed of light in m/s
lambda = c/radar.fc;  % Wavelength

% Calculate range parameters
range_res = c/(2*radar.B);  % Range resolution
max_range = c/(2*ofdm.df);  % Maximum unambiguous range based on subcarrier spacing

% Generate range grid with finer resolution
rng_grid = linspace(0, max_range, music.nrange_bins);

% Define frequency grid for steering vectors
freq_grid = (0:ofdm.Nsc-1)*ofdm.df;

% Pre-compute range steering vectors for all range bins
range_steering = zeros(music.nrange_bins, ofdm.Nsc);
for i = 1:music.nrange_bins
    % Phase shift for each subcarrier based on range
    tau = 2*rng_grid(i)/c;  % Round-trip delay
    range_steering(i,:) = exp(-1j*2*pi*freq_grid*tau);
end

% Normalize steering vectors
range_steering = range_steering ./ sqrt(sum(abs(range_steering).^2, 2));

% Initialize and compute MUSIC spectrum
P_music = zeros(music.nrange_bins, 1);
for i = 1:music.nrange_bins
    a = range_steering(i,:).';
    P_music(i) = 1/(real(a' * VnVn * a) + eps);
end

% Normalize MUSIC spectrum
P_music = abs(P_music);
P_music = P_music/max(P_music(:));

% Find peaks in MUSIC spectrum
[peaks, locs] = findpeaks(P_music, 'MinPeakHeight', 0.5, 'SortStr', 'descend');
detected_ranges = rng_grid(locs);

% Keep only the top 3 peaks
if length(peaks) > music.nsignals
    detected_ranges = detected_ranges(1:music.nsignals);
end

% Define true target ranges (from receiveSignal_multiTarget.m)
true_ranges = [
    sqrt(50^2 + 50^2);     % Target 1: ~70.71m
    sqrt(20^2 + (-20)^2);  % Target 2: ~28.28m
    sqrt(100^2 + 80^2)     % Target 3: ~128.06m
];

% Print results
fprintf('\nSystem Parameters:\n');
fprintf('Range Resolution: %.2f m\n', range_res);
fprintf('Maximum Range: %.2f m\n', max_range);
fprintf('Subcarrier spacing: %.2f Hz\n', ofdm.df);

fprintf('\nMUSIC Algorithm Detection Results:\n');
fprintf('True Target Ranges:\n');
for i = 1:length(true_ranges)
    fprintf('Target %d: Range = %.2f m\n', i, true_ranges(i));
end

fprintf('\nDetected Ranges:\n');
for i = 1:length(detected_ranges)
    fprintf('Target %d: Range = %.2f m\n', i, detected_ranges(i));
    
    % Find closest true target and calculate error
    [min_error, closest_idx] = min(abs(true_ranges - detected_ranges(i)));
    fprintf('  Error from closest true target: %.2f m\n', min_error);
end

%% Visualization
% Plot MUSIC spectrum
figure('Name', 'MUSIC Range Spectrum');
plot(rng_grid, 10*log10(P_music), 'b-', 'LineWidth', 1.5);
hold on;

% Plot true target ranges
for i = 1:length(true_ranges)
    xline(true_ranges(i), 'r--', sprintf('True Target %d', i));
end

% Plot detected ranges
for i = 1:length(detected_ranges)
    xline(detected_ranges(i), 'g-.', sprintf('Detected %d', i));
end

xlabel('Range (m)');
ylabel('Power (dB)');
title('MUSIC Range Spectrum');
grid on;
legend('MUSIC Spectrum', 'True Targets', 'Detected Targets');

%% Helper function for 2D peak finding
function [peaks, locs] = findpeaks2D(matrix, varargin)
    % Find local maxima in 2D matrix
    p = inputParser;
    addParameter(p, 'MinPeakHeight', 0);
    parse(p, varargin{:});
    
    minHeight = p.Results.MinPeakHeight;
    [rows, cols] = size(matrix);
    peaks = [];
    locs = [];
    
    for i = 2:rows-1
        for j = 2:cols-1
            window = matrix(i-1:i+1, j-1:j+1);
            if matrix(i,j) == max(window(:)) && matrix(i,j) > minHeight
                peaks = [peaks; matrix(i,j)];
                locs = [locs; i j];
            end
        end
    end
end
