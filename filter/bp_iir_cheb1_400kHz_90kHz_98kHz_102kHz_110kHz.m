clear all;

% Fallback for the C1 fast path (-c) clock filter, the conventional design of
% bp_iir_cheb1_800kHz_90kHz_98Hz_102kHz_110kHz.m at 400 kHz. It is not used:
% the fs/4 resonator (bp_resonator_400kHz_100kHz.m) passed the lock, jitter and
% sensitivity tests.

samplerate = 400e3;
nyqistrate = samplerate/2;

Ws1 = 90e3/nyqistrate;
Wp1 = 98e3/nyqistrate;
Wp2 = 102e3/nyqistrate;
Ws2 = 110e3/nyqistrate;
Rp = 1;
Rs = 40;

[n, Wc] = cheb1ord([Wp1, Wp2], [Ws1, Ws2], Rp, Rs);
[b, a] = cheby1(n, Rp, Wc);

print_iir_filter_coef(b ,a);
