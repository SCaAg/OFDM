%This file create the reflected signal by a single taget

% xofdm: the input OFDM symbol of the size (OFDM number)x(Subcarrier No)
% fc: carrier frequency
% fs: sampling rate
% Tofdm: ofdm duration
% yofdmr: the received signal after reflecting from the target
function yofdmr=receiveSignal_singleTarget(xofdm,fs,fc,Tofdm)

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
tgtpos=[50; 50; 2]; % the initial position of the target
tgtvel=[10; 10; 0]; % the target velocity;
tgtrcs=4.7; % Target RCS
tgtmotion = phased.Platform('InitialPosition', tgtpos, 'Velocity', tgtvel);
target = phased.RadarTarget('Model', 'Swerling1', 'MeanRCS', tgtrcs, 'OperatingFrequency', fc);

for m=1:Mofdm
    [tgtpos,tgtvel]=tgtmotion(Tofdm); % update the target position and relative velocity

    % Calculate the target angles as seen from the transmit array
    [~,tgtang]=rangeangle(tgtpos,BSLoc);

    % Transmit signal
    txsig = transmitter(xofdm(:, m));

    % Radiate signal towards the targets
    radtxsig = radiator(txsig, tgtang);

    % Apply free space channel propagation effects
    chansig = radarChannel(radtxsig, BSLoc, tgtpos, zeros(3,1), tgtvel); 

    % Reflect signal off the targets
    tgtsig = target(chansig, false);
    
    % Receive target returns at the receive array
    rxsig = collector(tgtsig, tgtang);
 
    % Add thermal noise at the receiver
    yofdmr(:, m) = receiver(rxsig);
end

