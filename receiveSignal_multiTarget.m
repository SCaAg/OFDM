%This file create the reflected signal by a single taget

% xofdm: the input OFDM symbol of the size (OFDM number)x(Subcarrier No)
% fc: carrier frequency
% fs: sampling rate
% Tofdm: ofdm duration
% yofdmr: the received signal after reflecting from the target
function yofdmr=receiveSignal_multiTarget(xofdm,fs,fc,Tofdm)

Pt = 0.1;                              % Peak power (W)
Gtx = 20;                               % Tx antenna gain (dB)

Grx = 20;                               % Radar Rx antenna gain (dB)
NF = 2.9;                               % Noise figure (dB)
Tref = 290;                             % Reference temperature (K)

% Setup the transmitter and the radiator
transmitter = phased.Transmitter('Gain', Gtx, 'PeakPower', Pt);

% Assume the JRC is using an isotropic antenna
ant = phased.IsotropicAntennaElement;
radiator = phased.Radiator('Sensor', ant, 'OperatingFrequency', fc);

% Setup the collector and the receiver
collector = phased.Collector('Sensor', ant, 'OperatingFrequency', fc);
receiver = phased.ReceiverPreamp('SampleRate', fs, 'Gain', Grx, 'NoiseFigure', NF, 'ReferenceTemperature', Tref);
% Setup a free space channel to model the JRC signal propagation from the
% transmitter to the targets and back to the radar receiver
radarChannel = phased.FreeSpace('SampleRate', fs, 'TwoWayPropagation', true, 'OperatingFrequency', fc);

% OFDM waveform is not a constant modulus waveform. The generated OFDM
% samples have power much less than one. To fully utilize the available
% transmit power, normalize the waveform such that the sample with the
% largest power had power of one.
xofdm = xofdm/max(sqrt(abs(xofdm).^2), [], 'all');
[~,Mofdm]=size(xofdm); %get the number of subcarriers and ofdm symbos
% Preallocate space for the signal received by the radar receiver
yofdmr = zeros(size(xofdm));
BSLoc=[0;0;0];
tgt1_pos = [50; 50; 2];
tgt1_vel = [10; 10; 0];
tgt1_rcs = 4.7;
tgt1_motion = phased.Platform('InitialPosition', tgt1_pos, 'Velocity', tgt1_vel);
target1 = phased.RadarTarget('Model', 'Swerling1', 'MeanRCS', tgt1_rcs, 'OperatingFrequency', fc);

% Target 2 properties (new target)
tgt2_pos = [20; -20; 5];
tgt2_vel = [-5; 15; 0];
tgt2_rcs = 3.2;
tgt2_motion = phased.Platform('InitialPosition', tgt2_pos, 'Velocity', tgt2_vel);
target2 = phased.RadarTarget('Model', 'Swerling1', 'MeanRCS', tgt2_rcs, 'OperatingFrequency', fc);

% Target 3 properties (new target)
tgt3_pos = [100; 80; 7];
tgt3_vel = [-30; 15; 5];
tgt3_rcs = 4.1;
tgt3_motion = phased.Platform('InitialPosition', tgt3_pos, 'Velocity', tgt3_vel);
target3 = phased.RadarTarget('Model', 'Swerling1', 'MeanRCS', tgt3_rcs, 'OperatingFrequency', fc);
% Simulation loop for each OFDM symbol
for m = 1:Mofdm
    % Update target 1 position and velocity
    [tgt1_pos, tgt1_vel] = tgt1_motion(Tofdm);

    % Update target 2 position and velocity
    [tgt2_pos, tgt2_vel] = tgt2_motion(Tofdm);

     % Update target 3 position and velocity
    [tgt3_pos, tgt3_vel] = tgt3_motion(Tofdm);

    % Calculate the angles to both targets
    [~, tgt1_ang] = rangeangle(tgt1_pos, BSLoc);
    [~, tgt2_ang] = rangeangle(tgt2_pos, BSLoc);
    [~, tgt3_ang] = rangeangle(tgt3_pos, BSLoc);

    % Transmit signal
    txsig = transmitter(xofdm(:, m));

    % Radiate signal towards targets
    radtxsig1 = radiator(txsig, tgt1_ang);
    radtxsig2 = radiator(txsig, tgt2_ang);
    radtxsig3 = radiator(txsig, tgt3_ang);

    % Propagate signal to and from targets
    chansig1 = radarChannel(radtxsig1, BSLoc, tgt1_pos, zeros(3, 1), tgt1_vel);
    chansig2 = radarChannel(radtxsig2, BSLoc, tgt2_pos, zeros(3, 1), tgt2_vel);
    chansig3 = radarChannel(radtxsig3, BSLoc, tgt3_pos, zeros(3, 1), tgt3_vel);

    % Reflect signals off targets
    tgtsig1 = target1(chansig1, false);
    tgtsig2 = target2(chansig2, false);
    tgtsig3 = target3(chansig3, false);

    % Receive combined target returns at the receiver
    rxsig1 = collector(tgtsig1, tgt1_ang); 
    rxsig2 = collector(tgtsig2, tgt2_ang); 
    rxsig3 = collector(tgtsig3, tgt3_ang); 
    rxsig = rxsig1 + rxsig2 + rxsig3;
    % Add thermal noise at the receiver
    yofdmr(:, m) = receiver(rxsig);
end
end

