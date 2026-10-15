#!/usr/bin/env python3
"""Derive the C1 fast path (-c R) constants of rtl_wmbus.c without Octave.

The FIR taps use a port of the fir1/buttord behaviour that produced the
existing 800 kHz filters; it is checked against those filters first.
"""

import cmath
import math


def interp1(xs, ys, x):
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            if xs[i + 1] == xs[i]:
                return ys[i]
            return ys[i] + (x - xs[i]) / (xs[i + 1] - xs[i]) * (ys[i + 1] - ys[i])
    raise ValueError(x)


def legacy_fir1_lowpass(n, wc, grid_n=512, ramp_n=2):
    """fir1(n, wc): fir2 frequency sampling on a 512-point grid with a linear
    ramp over ramp_n / 2 grid steps ending at wc, Hamming window, normalized
    to unity DC gain."""
    f = [0.0, wc - ramp_n / 2 / grid_n, wc, 1.0]
    m = [1.0, 1.0, 0.0, 0.0]
    grid = [interp1(f, m, k / grid_n) for k in range(grid_n + 1)]
    if n % 2 == 0:
        spectrum = grid + grid[grid_n - 1:0:-1]
        indexes = list(range(len(spectrum) - n // 2, len(spectrum))) + list(range(n // 2 + 1))
        scale = 1
    else:
        spectrum = grid + [0.0] * (2 * grid_n) + grid[grid_n - 1:0:-1]
        indexes = list(range(len(spectrum) - n, len(spectrum), 2)) + list(range(1, n + 1, 2))
        scale = 2
    size = len(spectrum)
    taps = [scale * sum(x * cmath.exp(2j * math.pi * j * t / size) for j, x in enumerate(spectrum)).real / size
            for t in indexes]
    window = [0.54 - 0.46 * math.cos(2 * math.pi * k / n) for k in range(n + 1)]
    taps = [b * w for b, w in zip(taps, window)]
    total = sum(taps)
    return [b / total for b in taps]


def buttord_lowpass(wp, ws, rp, rs):
    """Order and -3 dB frequency matched to the pass band, bilinear prewarping."""
    wpw, wsw = math.tan(math.pi * wp / 2), math.tan(math.pi * ws / 2)
    n = math.ceil(math.log10((10 ** (rs / 10) - 1) / (10 ** (rp / 10) - 1)) / (2 * math.log10(wsw / wpw)))
    return n, 2 / math.pi * math.atan(wpw / (10 ** (rp / 10) - 1) ** (1 / (2 * n)))


def response_db(taps, frequency, rate):
    h = sum(b * cmath.exp(-2j * math.pi * frequency / rate * k) for k, b in enumerate(taps))
    return 20 * math.log10(max(abs(h), 1e-12))


def design_fir(rate, passband=100e3, stopband=160e3):
    n, wc = buttord_lowpass(passband / (rate / 2), stopband / (rate / 2), 1, 40)
    return legacy_fir1_lowpass(n, wc)


def check_legacy_designs():
    existing = {
        (800e3, 100e3, 160e3): [-0.00456638213, -0.002571450348, 0.02689425925, 0.1141330398, 0.2264456422,
                                0.2793297826],
        (800e3, 32e3, 36e3): [-0.000649081282, -0.0009491938209, -0.001361601657, -0.001910785234,
                              -0.002570133495, -0.003251218426, -0.003801634695, -0.004012672882,
                              -0.003636803575, -0.002413585945, -0.0001013597693, 0.003488892085,
                              0.008461671287, 0.01481127545, 0.02240598045, 0.03098477999, 0.0401679839,
                              0.04948137286, 0.05839197924, 0.06635211627, 0.07284719662, 0.07744230649,
                              0.07982251613],
    }
    for (rate, passband, stopband), folded in existing.items():
        taps = design_fir(rate, passband, stopband)
        error = max(abs(a - b) for a, b in zip(taps, folded))
        assert len(taps) == 2 * len(folded) - (len(taps) % 2) and error < 1e-9, error
        print("legacy method reproduces %g/%g kHz at %g kHz: %d taps, max error %.1e" % (
            passband / 1e3, stopband / 1e3, rate / 1e3, len(taps), error))


def cic_db(frequency, rate, length, stages=3):
    x = math.pi * frequency / rate
    return 20 * stages * math.log10(abs(math.sin(length * x) / (length * math.sin(x))))


def main():
    check_legacy_designs()

    taps = design_fir(400e3)
    print("\n400 kHz FIR, %d taps: %s" % (len(taps), ", ".join("%.10g" % b for b in taps)))
    print("folded: {%s, }" % ", ".join("%.10g" % b for b in taps[:(len(taps) + 1) // 2]))
    for frequency in (50e3, 100e3, 160e3):
        print("  %3.0f kHz: %6.2f dB (800 kHz design %6.2f dB)" % (
            frequency / 1e3, response_db(taps, frequency, 400e3),
            response_db(design_fir(800e3), frequency, 800e3)))

    print("\nDecimators: droop at 100 kHz; aliases into the 100 kHz band edge from 300 and 500 kHz")
    print("  boxcar-8 at 1.6 MS/s, decimate by 2: %.2f dB; from 700 kHz %.2f dB" % (
        cic_db(100e3, 1.6e6, 8, 1), cic_db(700e3, 1.6e6, 8, 1)))
    for rate in range(3, 9):
        fs = 400e3 * rate
        figures = [cic_db(f, fs, rate) for f in (100e3, 300e3, 500e3) if f < fs / 2]
        print("  CIC R=%d (N=3, gain %3d, %.1f MS/s): %s dB" % (
            rate, rate ** 3, fs / 1e6, " / ".join("%.2f" % x for x in figures)))

    r = 0.99
    print("\nClock resonator y[n] = x[n] - x[n-2] - r^2 y[n-2], r = %g: r^2 = %.6g, peak gain %.1f," % (
        r, r * r, 2 / (1 - r * r)))
    print("  3 dB bandwidth about (1 - r) fs / pi = %.0f Hz, time constant %.0f samples (%.0f bits)" % (
        (1 - r) * 400e3 / math.pi, 1 / (1 - r), 1 / (1 - r) / 4))

    print("\nDC blocker: pole 0.999 per 800 kHz sample -> 0.999^2 = %.6f at 400 kHz" % 0.999 ** 2)
    print("RSSI smoother: same time constant would be 1 - (1 - 0.6789)^2 = %.8f;" % (1 - (1 - 0.6789) ** 2))
    print("  0.5 chosen (3.6 us): fewer weak frames aborted by PACKET_CAPTURE_THRESHOLD")
    print("Time-2 edge: sample at the rising edge if the interpolated crossing lies in the first half")
    print("  of the preceding interval (old + new >= 0), else one sample later")
    print("Run length: bit length 4 * 256, minimum run 3, majority of 3 raw bits, PI gains 64/32")


if __name__ == "__main__":
    main()
