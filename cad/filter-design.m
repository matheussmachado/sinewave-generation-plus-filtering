clc; clear; close all
pkg load signal
pkg load control

% FILTER DESIGN
% =====================================================================
% Digital Filter
T = 1e-3;   % sampling period
fs = 1/T;
fc = 50;    % cut off frequency
[b, a] = butter(2, fc/(fs/2), "low");    % filter calculation
save filter.mat b a

% Analog Filter
[z, p, k] = butter(2, 2*pi*fc, "low", "s");
[n, d] = zp2tf(z, p, k);
wn2 = d(3);

% Analog Filter Adjusted
C1 = 200e-9;
C2 = C1/2;
R = sqrt(1/(wn2*C1*C2));

% R1 and R2 based on R value
R1 = 20e3;
R2 = 26e3;

s = tf("s");
wn2_adj = 1/( R1*R2*C1*C2 );
two_csi = ( (1/R1) + (1/R2) ) / C1;
Fs =  wn2_adj / ( s^2 +  two_csi*s + wn2_adj);  % analog filter transfer function


% FILTER COEFFICIENTS TO ARM CMSIS DSP IRR FORMAT
% =====================================================================
[sos, g] = tf2sos(b, a);    % second-order section, gain
coeff = sos([1 2 3 5 6]);
coeff(1:3) = g*coeff(1:3);
coeff(4:5) = -coeff(4:5);

disp("CMSIS IIR FILTER COEFFICIENTS: ")
for i=1:5
    if i != 5
        printf("%f, ", coeff(i));
    else
        printf("%f\n", coeff(i));
    endif
end


% FILTER PROCESSING
% =====================================================================
f = 50;  % input signal frequency
t = 0:T:T*(0.1/T-1);
x = 0.9*(sin(2*pi*f*t) + 1.1)*4095/2;  % input signal
yfz = filter(b, a, x);
[yfs, _, ~] = lsim(Fs, x, t);


% PLOT
% =====================================================================
figure(1)
Analog = Fs;
Digital = tf(b, a, T); 
bode(Analog, Digital);

figure(2)
plot(t, x, 'r');
hold on
plot(t, yfs, 'm');
hold on
plot(t, yfz, 'b--');
title("Filter perfomance (Fc = 50 Hz)")
xlabel("Time [sec]"); ylabel("Amplitude [ADC 12 bits]")
legend("Input Signal", "Output Analog Filter", "Ouput Digital Filter")
xlim([0 t(end)])
