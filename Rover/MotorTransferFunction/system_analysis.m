%
% 
% Encoder Signal Analysis for Step Response
% This script processes raw step response data from an oscilloscope and 
% analyzes the frequency of encoder pulses over time.

% Clear workspace, close all figures, and clear command window
clearvars;
close all;
clc;

s = tf('s');

% Specify the CSV file containing the step response data
filename = 'step_response_0_60.csv';

% Set up import options for the CSV file
opts = delimitedTextImportOptions("NumVariables", 5);
opts.VariableNames = ["k", "Encoder", "Trig", "StartingTime", "SamplingTime"];
opts.VariableTypes = ["double", "double", "double", "double", "double"];
opts = setvaropts(opts, 1:5, "EmptyFieldRule", "auto");
opts.DataLines = [2, Inf]; % Ignore the header

% Import data from the CSV file into a table
tbl = readtable(filename, opts);

% Retrieve sampling and starting time from the table
sampling_time = tbl.SamplingTime(1); % Assume sampling time is in seconds
starting_time = tbl.StartingTime(1);

% Remove the first row of the table
tbl(1, :) = [];

% Remove the unnecessary columns (fourth and fifth)
tbl(:, 4:5) = [];

% Extract the data
time = (tbl.k * sampling_time) + starting_time; % Calculate the time
ch1 = tbl.Encoder; % Encoder signal
ch2 = tbl.Trig; % Trigger signal

% Plot raw data
figure;
plot(time, ch1, 'b', 'DisplayName','Raw Encoder Signal');
hold on;
plot(time, ch2, 'r', 'DisplayName','Raw Trigger Signal');
hold off;

% Set graphic details
title('Raw Data of Encoder and Trigger Signals');
xlabel('Time (s)');
ylabel('Voltage (V)');
legend;
grid on;

% Thresholds for normalization
threshold_encoder = 2.5; % Half of the encoder's operating voltage (5V)
threshold_trigger = 1.65; % Half of the trigger signal's voltage (3.3V)

% Normalize the data based on the thresholds
ch1_norm = ch1 > threshold_encoder;
ch1_norm = ch1_norm * 5; % Scale to the maximum encoder voltage

ch2_norm = ch2 > threshold_trigger;
ch2_norm = ch2_norm * 3.3; % Scale to the maximum trigger signal voltage

% Find the index of the first rising edge in the normalized trigger signal
index_first_rising_edge = find(ch2_norm > 0, 1, 'first');

% Create the step signal
step_signal = zeros(length(time), 1); % Initialize the step signal with zeros
step_signal(index_first_rising_edge:end) = 60; % Set the signal to 60 after the first rising edge


% Plot the normalized signals
figure;
plot(time, ch1_norm, 'b', 'DisplayName', 'Normalized Encoder Output');
hold on;
plot(time, ch2_norm, 'r', 'DisplayName', 'Normalized Trigger Signal');
hold off;

% Set the y-axis limits
ylim([-1 8]); % Set y-axis limits from -1 to 8V for clarity

% Set the graph details
title('Normalized Encoder and Trigger Signals Comparison');
xlabel('Time (s)');
ylabel('Voltage (V)');
legend;
grid on;

% Find rising edges of the normalized encoder signal
rising_edges = [false; diff(ch1_norm) == 5];

% Calculate time intervals between rising edges
rising_times = time(rising_edges);
intervals = diff(rising_times);

% Compute instantaneous pulse frequency
instantaneous_frequency = 1 ./ intervals;

% Interpolate to create a continuous representation over time
% Interpolazione con metodo pchip per una curva che preserva la forma dei dati
interpolated_frequency = interp1(rising_times(1:end-1), instantaneous_frequency, time, 'pchip', 'extrap');

figure;
yyaxis left; % Imposta l'asse sinistro per la frequenza interpolata
plot(time, interpolated_frequency, 'b', 'DisplayName', 'Interpolated Pulse Frequency');
ylabel('Frequency (Hz)');

yyaxis right; % Imposta l'asse destro per il trigger normalizzato
plot(time, ch2_norm, 'r--', 'DisplayName', 'Normalized Trigger Signal');
ylabel('Voltage (V)');

title('Interpolated Pulse Frequency and Normalized Trigger Signal Over Time');
xlabel('Time (s)');
legend;
grid on;


% Calcola la velocità in RPM
PPR = 12; % Impulsi per giro dell'encoder
GEAR_RATIO = 51; % Rapporto di trasmissione

% Calcolo della velocità in RPM
rpm = (interpolated_frequency * 60) / (PPR * GEAR_RATIO);

figure;
plot(time, rpm, 'b', 'DisplayName', 'Velocity in RPM');
ylabel('RPM');

yyaxis right; % Imposta l'asse destro per il trigger normalizzato
plot(time, ch2_norm, 'r--', 'DisplayName', 'Normalized Trigger Signal');
ylabel('Voltage (V)');

title('Motor Velocity in RPM and Normalized Trigger Signal Over Time');
xlabel('Time (s)');
legend;
grid on;
