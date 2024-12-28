clear; clc; close all;
pkg load signal
load filter.mat

data = csvread("log");

t = data(3:end,1) - data(1,1);
in_adc = data(3:end,2);
in_filter = data(3:end,3);
out_filter = data(3:end,4);

yf = filter(b, a, in_adc);
plot(t, in_adc, 'b-o')
hold on
plot(t, out_filter, 'g-o')
hold on
plot(t, yf, 'mx')
legend("ADC Input", "Real Filter", "Ideal Filter")