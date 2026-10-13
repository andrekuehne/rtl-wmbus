clear all;

% C1 fast path (-c): same pass band and method as lp_fir_butter_800kHz_100kHz_160kHz.m
% at half the sample rate. buttord gives n = 5, i.e. 6 taps:
% -0.00963706533, 0.0493523251, 0.4602847402, 0.4602847402, 0.0493523251, -0.00963706533
% Those taps come from c1_fast_path_constants.py, which reproduces the existing
% 800 kHz filters to 1e-10 with the fir1/buttord behaviour that generated them
% (Wc matched to the pass band, fir2 ramp of one grid step ending at Wc,
% unity DC gain).
% Newer Octave signal releases normalize fir1 differently and may print slightly
% different values.

samplerate = 400e3;
nyqistrate = samplerate/2;

Wp1 = 100e3/nyqistrate;
Ws1 = 160e3/nyqistrate;
Rp = 1;
Rs = 40;

[n, Wc] = buttord(Wp1, Ws1, Rp, Rs);
[b] = fir1(n, Wc);

print_fir_filter_coef(b);
