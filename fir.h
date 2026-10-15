#ifndef FIR_H
#define FIR_H

/*-
 * Copyright (c) 2017 <xael.south@yandex.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Floating and fixed point implementations of Finite Response Filter.
*/

#include <stdint.h>
#include <stddef.h>
#include <fixedptc/fixedptc.h>

typedef struct
{
    const size_t length;
    float *const b;

    size_t i;
    float *hist;
} FIRF_FILTER;

float firf(float sample, FIRF_FILTER *filter);

float firf(float sample, FIRF_FILTER *filter)
{
    const float *b = filter->b;
    float *hist = &filter->hist[filter->i++];
    size_t i;

    *hist = sample;

    sample = 0; // will be "y"

    for (i = filter->i; i--;)
    {
        sample += *b++ * *hist--;
    }

    hist = &filter->hist[filter->length-1];
    for (i = filter->length; i-- > filter->i;)
    {
        sample += *b++ * *hist--;
    }

    if (filter->i >= filter->length) filter->i = 0;

    return sample;
}

/* Fixed symmetric taps only: b holds the first (length + 1) / 2 taps.
 * Keep the full history ring; pair newest/oldest samples without wrapping
 * inside a dot-product segment. Adaptive filters must continue to use firf.
 */
static inline float firf_symmetric(float sample, FIRF_FILTER *filter)
{
    const size_t length = filter->length;
    const size_t pairs = length / 2;
    size_t left = filter->i;
    size_t right = left + 1;
    size_t tap = 0;
    float result = 0;

    filter->hist[left] = sample;
    if (right == length) right = 0;
    filter->i = right;

    while (tap < pairs)
    {
        size_t count = pairs - tap;
        if (count > left + 1) count = left + 1;
        if (count > length - right) count = length - right;
        for (size_t k = 0; k < count; ++k)
        {
            result += filter->b[tap + k] *
                      (filter->hist[left - k] + filter->hist[right + k]);
        }
        tap += count;
        left = count > left ? length - 1 : left - count;
        right += count;
        if (right == length) right = 0;
    }
    if (length & 1) result += filter->b[pairs] * filter->hist[left];

    return result;
}

/* Fixed symmetric taps; hist holds 2 * length floats initialized as identical
 * halves of the history ring.
 * Mirroring keeps both sides of each pair contiguous across ring wrap while
 * preserving firf_symmetric's tap and accumulation order.
 */
static inline float firf_symmetric_mirrored(float sample, FIRF_FILTER *filter)
{
    const size_t length = filter->length;
    const size_t pairs = length / 2;
    const size_t left = filter->i + length;
    const size_t right = filter->i + 1;
    float result = 0;

    filter->hist[filter->i] = sample;
    filter->hist[left] = sample;
    filter->i = right == length ? 0 : right;

    for (size_t tap = 0; tap < pairs; ++tap)
        result += filter->b[tap] *
                  (filter->hist[left - tap] + filter->hist[right + tap]);
    if (length & 1) result += filter->b[pairs] * filter->hist[left - pairs];

    return result;
}

void firf_lms(float mu_e, FIRF_FILTER *filter)
{
    float *b = &filter->b[filter->length-1];
    size_t i;
    float *hist;

    i = filter->i;
    hist = &filter->hist[i];
    while (i++ < filter->length)
    {
        *b-- += mu_e * *hist++;
    }

    i = 0;
    hist = &filter->hist[i];
    while (i++ < filter->i)
    {
        *b-- += mu_e * *hist++;
    }
}

typedef struct
{
    const size_t length;
    const fixedpt *const b;

    size_t i;
    fixedpt *hist;
} FIRFP_FILTER;

fixedpt firfp(fixedpt sample, FIRFP_FILTER *filter);

fixedpt firfp(fixedpt sample, FIRFP_FILTER *filter)
{
    const fixedpt *b = filter->b;
    fixedpt *hist = &filter->hist[filter->i++];
    size_t i;

    *hist = sample;

    sample = 0; // will be "y"

    for (i = filter->i; i--;)
    {
        sample = fixedpt_add(sample, fixedpt_mul((*b++), (*hist--)));
    }

    hist = &filter->hist[filter->length-1];
    for (i = filter->length; i-- > filter->i;)
    {
        sample = fixedpt_add(sample, fixedpt_mul((*b++), (*hist--)));
    }

    if (filter->i >= filter->length) filter->i = 0;

    return sample;
}

#endif /* FIR_H */

