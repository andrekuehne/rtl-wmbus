clear all;

% C1 fast path (-c) time-2 clock filter. At 400 kHz the 100 kHz symbol clock is
% fs/4, so a second-order resonator with poles at +-j*r and zeros at DC and fs/2
% has a zero middle coefficient:
%   y[n] = x[n] - x[n-2] - r^2 * y[n-2]
% Its gain at fs/4 is 2/(1 - r^2) with zero phase, like the 800 kHz Chebyshev
% band pass (about -1 degree at 100 kHz). r = 0.99 gives a 1.28 kHz 3 dB band
% and a 1/(1 - r) = 100 sample (25 bit) time constant; r = 0.969, the width of
% the 98..102 kHz pass band, lost frames in noise (see c1_fast_path_constants.py).

samplerate = 400e3;
r = 0.99;

b = [1, 0, -1];
a = [1, 0, r^2];

[h, w] = freqz(b, a, 2^16, samplerate);
peak = max(abs(h));
band = w(abs(h) >= peak/sqrt(2));

fprintf(stdout, "#define C1_FAST_CLOCK_POLE_RADIUS_SQUARED %.10gf // r = %g\n", r^2, r);
fprintf(stdout, "%% peak %.4g at %.6g Hz, 3 dB band %.6g..%.6g Hz, time constant %.4g samples\n", ...
        peak, w(abs(h) == peak), min(band), max(band), 1/(1 - r));
