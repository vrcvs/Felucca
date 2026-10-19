unsigned long long __udivdi3(unsigned long long n, unsigned long long d);
long long __divdi3(long long n, long long d)
{
    unsigned long long un, ud, q;
    int negative;
    if (!d)
        return 0;
    negative = (n < 0) != (d < 0);
    un = n < 0 ? 0ULL - (unsigned long long)n : (unsigned long long)n;
    ud = d < 0 ? 0ULL - (unsigned long long)d : (unsigned long long)d;
    q = __udivdi3(un, ud);
    return negative ? (long long)(0ULL - q) : (long long)q;
}

unsigned long long __udivdi3(unsigned long long n, unsigned long long d);
/* JieLi's freestanding link omits the compiler's 64-bit unsigned divide helper. */
unsigned long long __udivdi3(unsigned long long n, unsigned long long d)
{
    unsigned long long q = 0, r = 0;
    int i;
    if (!d)
        return ~0ULL;
    for (i = 63; i >= 0; --i) {
        unsigned long long carry = r >> 63;
        r = (r << 1) | ((n >> i) & 1ULL);
        if (carry || r >= d) {
            r -= d;
            q |= 1ULL << i;
        }
    }
    return q;
}

/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* The few libc routines the compiler may call implicitly (struct copies,
 * zeroing). Freestanding: no other libc. */
void *memset(void *d, int c, unsigned n)
{
    unsigned char *p = d;
    while (n--)
        *p++ = (unsigned char)c;
    return d;
}

void *memcpy(void *d, const void *s, unsigned n)
{
    unsigned char *p = d;
    const unsigned char *q = s;
    while (n--)
        *p++ = *q++;
    return d;
}

int memcmp(const void *a, const void *b, unsigned n)
{
    const unsigned char *p = a, *q = b;
    for (; n; n--, p++, q++)
        if (*p != *q)
            return *p - *q;
    return 0;
}

static int str_eq(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

static unsigned str_len(const char *s)
{
    unsigned n = 0;
    while (s[n])
        n++;
    return n;
}

static void str_cpy(char *d, const char *s, unsigned max)
{
    unsigned i = 0;
    while (s[i] && i + 1 < max) {
        d[i] = s[i];
        i++;
    }
    d[i] = 0;
}

/* decimal with an optional fixed point: fmt_fix(buf, -63, 1) -> "-6.3" */
static void fmt_fix(char *b, int32_t v, int decimals)
{
    char t[12];
    int n = 0, neg = v < 0, i;
    uint32_t u = (uint32_t)(neg ? -v : v);
    do {
        t[n++] = (char)('0' + u % 10u);
        u /= 10u;
        if (n == decimals)
            t[n++] = '.';
    } while (u || n <= decimals);
    if (neg)
        t[n++] = '-';
    for (i = 0; i < n; i++)
        b[i] = t[n - 1 - i];
    b[n] = 0;
}

static void fmt_int(char *b, int32_t v) { fmt_fix(b, v, 0); }

static uint32_t rng_state = 0x1234567u;
static uint32_t rng(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}
