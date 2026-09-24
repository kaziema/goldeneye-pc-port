/*
 * pcconv_cg.c — port of tools_pc/d69_emit.py (bg .seg / Tbg_ stan),
 * d88_emit.py (Usetup*Z) and d88_propdefs.py.
 *
 * is64 selects the x86-64 PC layout (pointers widened, records grown) or the
 * 32-bit layout (N64 sizes and field offsets, little-endian scalars only).
 */

#include "pcconv_int.h"

#define O(a, b) (is64 ? (uint32_t)(a) : (uint32_t)(b))

/* ------------------------------------------------------------------ helpers */

typedef struct V32 { uint32_t *p; size_t n, cap; } V32;

static int v32Push(V32 *v, uint32_t x)
{
    if (v->n == v->cap) {
        size_t nc = v->cap ? v->cap * 2 : 64;
        uint32_t *np = (uint32_t *)realloc(v->p, nc * sizeof(uint32_t));
        if (!np)
            return -1;
        v->p = np;
        v->cap = nc;
    }
    v->p[v->n++] = x;
    return 0;
}
static void v32Free(V32 *v) { free(v->p); memset(v, 0, sizeof(*v)); }
static int cmp32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : (x > y);
}
static void v32SortUniq(V32 *v)
{
    if (v->n < 2)
        return;
    qsort(v->p, v->n, sizeof(uint32_t), cmp32);
    size_t w = 1;
    for (size_t i = 1; i < v->n; i++)
        if (v->p[i] != v->p[w - 1])
            v->p[w++] = v->p[i];
    v->n = w;
}

/* bounds-checked destination image */
typedef struct Dst { uint8_t *p; size_t n; int oob; } Dst;

static int dNew(Dst *d, size_t n)
{
    d->p = (uint8_t *)calloc(n ? n : 1, 1);
    d->n = n;
    d->oob = 0;
    return d->p ? 0 : -1;
}
static int dOk(Dst *d, size_t o, size_t len)
{
    if (o + len > d->n) { d->oob = 1; return 0; }
    return 1;
}
static void dLe32(Dst *d, size_t o, uint32_t v) { if (dOk(d, o, 4)) putLe32(d->p + o, v); }
static void dLe16(Dst *d, size_t o, uint32_t v) { if (dOk(d, o, 2)) putLe16(d->p + o, v); }
static void dByte(Dst *d, size_t o, uint32_t v) { if (dOk(d, o, 1)) d->p[o] = (uint8_t)v; }
static void dCopy(Dst *d, size_t o, const uint8_t *s, size_t len)
{
    if (len && dOk(d, o, len))
        memcpy(d->p + o, s, len);
}

static void takeDst(Buf *out, Dst *d)
{
    out->p = d->p;
    out->len = out->cap = d->n;
    d->p = NULL;
}

/* -------------------------------------------------------------- bg .seg --- */

typedef struct BlobReg { uint32_t start, size; int rec; } BlobReg;

static int convertSeg(Ctx *c, const char *name, const uint8_t *data, uint32_t D, Buf *out)
{
    int is64 = c->L->is64;
    Src sv = { data, D, 0 };
    Src *s = &sv;
    V32 ptargets = {0}, atargets = {0}, targets = {0};
    BlobReg *br = NULL;
    size_t nbr = 0, capbr = 0;
    Dst d = {0};
    int rc = -1;

    if (D < 0x40) {
        cvErr(c, name, "too small", D, 0);
        return -1;
    }
    uint32_t h0 = sBe32r(s, 0);
    uint32_t h1 = sBe32o(s, 4), h2 = sBe32o(s, 8), h3 = sBe32o(s, 12), h4 = sBe32o(s, 16);
    if (h0 != 0)
        cvErr(c, name, "header word0 != 0", h0, 0);
    if (h4 != 0) {
        cvErr(c, name, "header word4 != 0", h4, 0);
        return -1;
    }
    if (h2 == 0 || h2 <= h1) {
        cvErr(c, name, "portal offset <= room offset", h2, h1);
        return -1;
    }
    uint32_t roomEnd = (h3 && h3 > h1 && h3 < h2) ? h3 : h2;
    if ((roomEnd - h1) % 24) {
        cvErr(c, name, "room table extent not a multiple of 24", roomEnd - h1, 0);
        return -1;
    }

    /* portal table walk */
    uint32_t o = h2, nPortals = 0;
    int found = 0;
    while ((uint64_t)o + 8 <= D) {
        uint32_t v = sBe32o(s, o);
        if (v == 0) { found = 1; break; }
        if (v32Push(&ptargets, v)) goto oom;
        nPortals++;
        o += 8;
    }
    if (!found) {
        cvErr(c, name, "portal table terminator not found before EOF", 0, 0);
        goto done;
    }
    uint32_t portalTableEnd = o + 8;
    uint32_t delta = is64 ? 8 * (nPortals + 1) : 0;

    /* envdata ALT targets */
    const uint32_t ENV_ALT = 100;
    if (h3 && roomEnd == h3) {
        uint32_t o2 = h3;
        while (o2 < h2) {
            uint32_t et = sByte(s, o2);
            int32_t dv = sBs32(s, o2 + 4);
            if (et == ENV_ALT && dv && v32Push(&atargets, (uint32_t)dv & MASK24)) goto oom;
            o2 += 8;
            if (et == 0)
                break;
        }
    }

    for (size_t i = 0; i < ptargets.n; i++) if (v32Push(&targets, ptargets.p[i])) goto oom;
    for (size_t i = 0; i < atargets.n; i++) if (v32Push(&targets, atargets.p[i])) goto oom;
    v32SortUniq(&targets);

#define BR_PUSH(st, sz, rc_)                                                        \
    do {                                                                            \
        if (nbr == capbr) {                                                         \
            size_t nc_ = capbr ? capbr * 2 : 64;                                    \
            BlobReg *np_ = (BlobReg *)realloc(br, nc_ * sizeof(BlobReg));           \
            if (!np_) goto oom;                                                     \
            br = np_; capbr = nc_;                                                  \
        }                                                                           \
        br[nbr].start = (st); br[nbr].size = (sz); br[nbr].rec = (rc_); nbr++;      \
    } while (0)

    uint32_t cursor = portalTableEnd;
    for (size_t i = 0; i < targets.n; i++) {
        uint32_t t = targets.p[i];
        if (t < cursor) {
            cvErr(c, name, "portal target overlaps previous region", t, cursor);
            goto done;
        }
        if (t > cursor)
            BR_PUSH(cursor, t - cursor, 0);
        if (t >= D) {
            cvErr(c, name, "portal target outside file", t, D);
            goto done;
        }
        uint32_t np = data[t];
        if (np == 0 || np > 20) {
            cvErr(c, name, "portal numPoints implausible", t, np);
            goto done;
        }
        uint32_t recsize = 4 + 12 * np;
        if ((uint64_t)t + recsize > D) {
            cvErr(c, name, "portal target overruns EOF", t, recsize);
            goto done;
        }
        BR_PUSH(t, recsize, 1);
        cursor = t + recsize;
    }
    if (cursor < D)
        BR_PUSH(cursor, D - cursor, 0);

    if (dNew(&d, (size_t)D + delta))
        goto oom;

    dLe32(&d, 0, h0);
    dLe32(&d, 4, 0x0F000000u | h1);
    dLe32(&d, 8, 0x0F000000u | h2);
    dLe32(&d, 12, h3 ? (0x0F000000u | h3) : 0);
    dLe32(&d, 16, 0);
    dCopy(&d, 20, data + 20, 0x40 - 20);

    /* room table: first 3 words are offsets into the portal blob tail */
    for (uint32_t o2 = h1; o2 < roomEnd; o2 += 24) {
        for (int w = 0; w < 3; w++) {
            uint32_t v = sBe32o(s, o2 + 4 * w);
            dLe32(&d, o2 + 4 * w, v ? (0x0F000000u | ((v + delta) & MASK24)) : 0);
        }
        for (int w = 0; w < 3; w++) {
            uint32_t fo = o2 + 12 + 4 * w;
            dLe32(&d, fo, sBe32r(s, fo));
        }
    }

    /* envdata table */
    if (h3 && roomEnd == h3) {
        uint32_t o2 = h3;
        while (o2 < h2) {
            uint32_t et = sByte(s, o2);
            dByte(&d, o2, et);
            dCopy(&d, o2 + 1, data + o2 + 1, 3);
            int32_t dv = sBs32(s, o2 + 4);
            if (et == ENV_ALT && dv)
                dv += (int32_t)delta;
            dLe32(&d, o2 + 4, (uint32_t)dv);
            o2 += 8;
            if (et == 0)
                break;
        }
    }

    /* portal table */
    {
        uint32_t o2 = h2, pcPos = h2;
        for (uint32_t idx = 0; idx <= nPortals; idx++) {
            uint32_t ov = sBe32o(s, o2);
            dLe32(&d, pcPos, ov ? (0x0F000000u | ((ov + delta) & MASK24)) : 0);
            dCopy(&d, pcPos + (is64 ? 8 : 4), data + o2 + 4, 4);
            o2 += 8;
            pcPos += is64 ? 16 : 8;
        }
    }

    /* portal point-data blob */
    for (size_t i = 0; i < nbr; i++) {
        uint32_t start = br[i].start, size = br[i].size, dst = start + delta;
        if (!br[i].rec) {
            dCopy(&d, dst, data + start, size);
            continue;
        }
        dByte(&d, dst, data[start]);
        dCopy(&d, dst + 1, data + start + 1, 3);
        for (uint32_t k = 0; k < (size - 4) / 4; k++)
            dLe32(&d, dst + 4 + 4 * k, sBe32r(s, start + 4 + 4 * k));
    }

    if (s->oob || d.oob) {
        cvErr(c, name, "seg: out-of-range access", 0, 0);
        goto done;
    }
    takeDst(out, &d);
    rc = 0;
    goto done;
oom:
    cvErr(c, name, "out of memory", 0, 0);
done:
    free(d.p);
    free(br);
    v32Free(&ptargets); v32Free(&atargets); v32Free(&targets);
    return rc;
#undef BR_PUSH
}

/* ------------------------------------------------------------- stan file -- */

static const uint8_t kTileSizes[12] = { 0x20, 0x20, 0x20, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0 };

static int convertStan(Ctx *c, const char *name, const uint8_t *data, uint32_t D, Buf *out)
{
    int is64 = c->L->is64;
    Src sv = { data, D, 0 };
    Src *s = &sv;
    Dst d = {0};
    int rc = -1;

    if (D < 8) {
        cvErr(c, name, "too small", D, 0);
        return -1;
    }
    uint32_t stanfile = sBe32r(s, 0);
    uint32_t o = 4, nOffs = 0;
    int found = 0;
    while ((uint64_t)o + 4 <= D) {
        uint32_t v = sBe32r(s, o);
        nOffs++;
        o += 4;
        if (v == 0) { found = 1; break; }
    }
    if (!found) {
        cvErr(c, name, "room-offset array terminator not found", 0, 0);
        return -1;
    }
    uint32_t N = nOffs - 1;
    uint32_t oldArrayEnd = o;
    uint32_t arrayDelta = is64 ? 4 * (N + 2) : 0;
    uint32_t newTileStart = oldArrayEnd + arrayDelta;

    if (dNew(&d, (size_t)D + arrayDelta)) {
        cvErr(c, name, "out of memory", 0, 0);
        return -1;
    }
    dLe32(&d, 0, stanfile);
    uint32_t p = is64 ? 8 : 4;
    for (uint32_t i = 0; i < nOffs; i++) {
        uint32_t v = sBe32r(s, 4 + 4 * i);
        dLe32(&d, p, v ? v + arrayDelta : 0);
        p += is64 ? 8 : 4;
    }

    uint32_t srcO = oldArrayEnd, dstO = newTileStart;
    int sentinel = 0;
    while ((uint64_t)srcO + 4 <= D) {
        if (rawBe32(data + srcO) == 0) {
            dLe32(&d, dstO, 0);
            sentinel = 1;
            break;
        }
        const uint8_t *w = data + srcO;
        dByte(&d, dstO, w[1]); dByte(&d, dstO + 1, w[0]);
        dByte(&d, dstO + 2, w[2]); dByte(&d, dstO + 3, w[3]);
        uint32_t mid = sBu16(s, srcO + 4), tail = sBu16(s, srcO + 6);
        dLe16(&d, dstO + 4, mid);
        dLe16(&d, dstO + 6, tail);
        uint32_t pointCount = (tail >> 12) & 0xF;
        uint32_t sz = kTileSizes[pointCount];
        if (sz == 0) {
            cvErr(c, name, "bad pointCount", pointCount, srcO);
            goto done;
        }
        if ((uint64_t)srcO + sz > D) {
            cvErr(c, name, "tile overruns EOF", srcO, sz);
            goto done;
        }
        for (uint32_t i = 0; i < pointCount; i++) {
            uint32_t po = srcO + 8 + 8 * i, qo = dstO + 8 + 8 * i;
            for (int k = 0; k < 4; k++)
                dLe16(&d, qo + 2 * k, sBu16(s, po + 2 * k));
        }
        srcO += sz;
        dstO += sz;
    }
    if (!sentinel) {
        cvErr(c, name, "ran off EOF without an all-zero EOF sentinel", 0, 0);
        goto done;
    }
    {
        uint32_t tailSrc = srcO + 4, tailDst = dstO + 4;
        if (tailSrc < D)
            dCopy(&d, tailDst, data + tailSrc, D - tailSrc);
    }
    if (s->oob || d.oob) {
        cvErr(c, name, "stan: out-of-range access", 0, 0);
        goto done;
    }
    takeDst(out, &d);
    rc = 0;
done:
    free(d.p);
    return rc;
}

/* -------------------------------------------------------------- propdefs -- */

static const uint8_t kPdWords[49] = {
    [1] = 64, [2] = 2, [3] = 32, [4] = 33, [5] = 32, [6] = 59, [7] = 33, [8] = 34,
    [9] = 7, [10] = 64, [11] = 149, [12] = 32, [13] = 54, [14] = 3, [17] = 32,
    [18] = 3, [19] = 4, [20] = 45, [21] = 34, [22] = 4, [23] = 4, [24] = 1,
    [25] = 2, [26] = 2, [27] = 2, [28] = 2, [29] = 2, [30] = 4, [31] = 1,
    [32] = 4, [33] = 5, [34] = 3, [35] = 4, [36] = 32, [37] = 10, [38] = 4,
    [39] = 44, [40] = 45, [42] = 32, [43] = 32, [44] = 5, [45] = 56, [46] = 7,
    [47] = 37, [48] = 1,
};

static const uint16_t kPdPcBytes[49] = {
    [1] = 296, [2] = 8, [3] = 144, [4] = 152, [5] = 144, [6] = 272, [7] = 152,
    [8] = 160, [9] = 32, [10] = 288, [11] = 664, [12] = 144, [13] = 248,
    [14] = 32, [17] = 144, [18] = 12, [19] = 32, [20] = 200, [21] = 152,
    [22] = 24, [23] = 16, [24] = 4, [25] = 8, [26] = 8, [27] = 8, [28] = 8,
    [29] = 8, [30] = 24, [31] = 4, [32] = 24, [33] = 24, [34] = 12, [35] = 24,
    [36] = 144, [37] = 48, [38] = 32, [39] = 208, [40] = 208, [42] = 144,
    [43] = 144, [44] = 40, [45] = 248, [46] = 28, [47] = 168, [48] = 4,
};

/* File-record word count for a propDef type (0 = unknown). */
uint32_t pcconvPropdefWords(uint32_t type)
{
    return type < 49 ? kPdWords[type] : 0;
}

static void wSwap(uint8_t *o, const uint8_t *w) { o[0] = w[3]; o[1] = w[2]; o[2] = w[1]; o[3] = w[0]; }
static void wHdr(uint8_t *o, const uint8_t *w) { o[0] = w[1]; o[1] = w[0]; o[2] = w[2]; o[3] = w[3]; }
static void wHH(uint8_t *o, const uint8_t *w) { o[0] = w[1]; o[1] = w[0]; o[2] = w[3]; o[3] = w[2]; }
static void wA(uint8_t *o, const uint8_t *w) { o[0] = w[0]; o[1] = w[1]; o[2] = w[3]; o[3] = w[2]; }
static void wCopy(uint8_t *o, const uint8_t *w) { memcpy(o, w, 4); }

static void emitObjectPrefix(uint8_t *out, const uint8_t *src, uint32_t so, int is64)
{
    uint32_t pc = 0;
    for (uint32_t i = 0; i < 32; i++) {
        const uint8_t *w = src + so + 4 * i;
        if (i == 4 || i == 5 || i == 26 || i == 27) {
            if (is64) {
                pc = (pc + 7) & ~7u;
                pc += 8;
            } else {
                wSwap(out + pc, w);
                pc += 4;
            }
            continue;
        }
        if (i == 0) wHdr(out + pc, w);
        else if (i == 1) wHH(out + pc, w);
        else if (i == 30 || i == 31) wCopy(out + pc, w);
        else wSwap(out + pc, w);
        pc += 4;
    }
}

/* tail slot kinds for ObjectRecord-derived types (D122/D123) */
enum { TK_PLAIN, TK_PTR, TK_HH, TK_ID };
static int hasTailDesc(uint32_t t)
{
    return t == 47 || t == 39 || t == 40 || t == 45 || t == 13 || t == 20;
}
static int tailKind(uint32_t t, uint32_t i)
{
    switch (t) {
    case 39: if (i == 32) return TK_ID; if (i == 41 || i == 43) return TK_PTR; if (i == 33) return TK_HH; break;
    case 40: if (i == 32) return TK_ID; if (i == 43 || i == 44) return TK_PTR; if (i == 33) return TK_HH; break;
    case 45: if (i == 32) return TK_PTR; break;
    case 13: if (i == 49 || i == 50 || i == 51) return TK_PTR; break;
    case 20: if (i >= 32 && i <= 44) return TK_HH; break;
    default: break;
    }
    return TK_PLAIN;
}

/* One record -> out (already zeroed, outLen bytes). Returns outLen. */
static uint32_t convertRecord(uint8_t *out, const uint8_t *src, uint32_t so, uint32_t t, int is64)
{
    uint32_t n64w = kPdWords[t];
    uint32_t outLen = is64 ? kPdPcBytes[t] : 4 * n64w;
    uint32_t i, pc;
#define W(i_) (src + so + 4 * (i_))

    if (t == 3 || t == 5 || t == 12 || t == 17 || t == 36 || t == 42 || t == 43) {
        emitObjectPrefix(out, src, so, is64);
        return outLen;
    }
    if (t == 1) {
        emitObjectPrefix(out, src, so, is64);
        pc = is64 ? 144 : 128;
        for (i = 32; i < n64w; i++) {
            const uint8_t *w = W(i);
            if (i == 50 || i == 51 || i == 61 || i == 62) {
                if (is64) { pc = (pc + 7) & ~7u; pc += 8; }
                else { wSwap(out + pc, w); pc += 4; }
            } else if (i == 38) { wHH(out + pc, w); pc += 4; }
            else if (i == 47) { wA(out + pc, w); pc += 4; }
            else if (i == 49) { wHdr(out + pc, w); pc += 4; }
            else { wSwap(out + pc, w); pc += 4; }
        }
        return outLen;
    }
    if (t == 4 || t == 21 || t == 7) {
        emitObjectPrefix(out, src, so, is64);
        pc = is64 ? 144 : 128;
        for (i = 32; i < n64w; i++) { wSwap(out + pc, W(i)); pc += 4; }
        return outLen;
    }
    if (t == 8) {
        emitObjectPrefix(out, src, so, is64);
        pc = is64 ? 144 : 128;
        wA(out + pc, W(32));
        if (!is64)
            for (i = 33; i < n64w; i++) wSwap(out + 4 * i, W(i));
        return outLen;
    }
    if (t == 10 || t == 11) {
        emitObjectPrefix(out, src, so, is64);
        uint32_t nmon = t == 10 ? 1 : 4, tailStart = 32 + 29 * nmon;
        if (!is64)
            for (i = 32; i < tailStart; i++) wSwap(out + 4 * i, W(i));
        pc = is64 ? 144 + 128 * nmon : 4 * tailStart;
        for (i = tailStart; i < n64w; i++) {
            if (t == 11) wCopy(out + pc, W(i));
            else wSwap(out + pc, W(i));
            pc += 4;
        }
        return outLen;
    }
    if (t == 6) {
        emitObjectPrefix(out, src, so, is64);
        if (is64) {
            wSwap(out + 144, W(32));
            pc = 212;
            for (i = 49; i < n64w; i++) { wSwap(out + pc, W(i)); pc += 4; }
        } else {
            for (i = 32; i < n64w; i++) wSwap(out + 4 * i, W(i));
        }
        return outLen;
    }
    if (t == 22) {
        wHdr(out, W(0));
        wHH(out + 4, W(1));
        if (!is64) { wSwap(out + 8, W(2)); wSwap(out + 12, W(3)); }
        return outLen;
    }
    if (t == 37) {
        wHdr(out, W(0));
        for (i = 1; i < 9; i++) wSwap(out + 4 * i, W(i));
        if (!is64) wSwap(out + 36, W(9));
        return outLen;
    }
    if (t == 14 || t == 19 || t == 38 || t == 44) {
        wHdr(out, W(0));
        if (is64) {
            uint32_t nidx = t == 44 ? 3 : 2;
            for (uint32_t k = 0; k < nidx; k++) wSwap(out + 8 + 8 * k, W(k + 1));
        } else {
            for (i = 1; i < n64w; i++) wSwap(out + 4 * i, W(i));
        }
        return outLen;
    }
    if (t == 18) {
        wHdr(out, W(0));
        wSwap(out + 4, W(1));
        wHdr(out + 8, W(2));
        return outLen;
    }
    if (t == 9) {
        wHdr(out, W(0));
        for (i = 1; i < 6; i++) wHH(out + 4 * i, W(i));
        if (!is64) wSwap(out + 24, W(6));
        return outLen;
    }
    if (hasTailDesc(t)) {
        emitObjectPrefix(out, src, so, is64);
        pc = is64 ? 144 : 128;
        for (i = 32; i < n64w; i++) {
            const uint8_t *w = W(i);
            int k = tailKind(t, i);
            if (k == TK_ID) {
                if (is64) { pc = (pc + 7) & ~7u; wSwap(out + pc, w); pc += 8; }
                else { wSwap(out + pc, w); pc += 4; }
            } else if (k == TK_PTR) {
                if (is64) { pc = (pc + 7) & ~7u; pc += 8; }
                else { wSwap(out + pc, w); pc += 4; }
            } else if (k == TK_HH) { wHH(out + pc, w); pc += 4; }
            else { wSwap(out + pc, w); pc += 4; }
        }
        return outLen;
    }
    if (t == 30 || t == 32 || t == 33 || t == 35) {
        wHdr(out, W(0));
        for (i = 1; i < (is64 ? n64w - 1 : n64w); i++) wSwap(out + 4 * i, W(i));
        return outLen;
    }
    /* generic: header word + plain bswap32 */
    wHdr(out, W(0));
    for (i = 1; i < n64w; i++) wSwap(out + 4 * i, W(i));
    return outLen;
#undef W
}

int cvConvertPropdefs(Ctx *c, const char *name, Src *src, uint32_t start,
                      uint32_t end, Buf *out)
{
    int is64 = c->L->is64;
    uint32_t o = start, n = 0;
    uint8_t rec[700];

    out->len = 0;
    while (o < end) {
        if ((uint64_t)o + 4 > end) {
            cvErr(c, name, "propDefs: truncated header", o, 0);
            return -1;
        }
        uint32_t t = src->p[o + 3];
        if (t >= 49 || kPdWords[t] == 0) {
            cvErr(c, name, "propDefs: unknown record type", t, o);
            return -1;
        }
        uint32_t n64w = kPdWords[t];
        if ((uint64_t)o + 4ull * n64w > end) {
            cvErr(c, name, "propDefs: record overruns region end", t, o);
            return -1;
        }
        uint32_t len = is64 ? kPdPcBytes[t] : 4 * n64w;
        if (len > sizeof(rec)) {
            cvErr(c, name, "propDefs: record too large", t, len);
            return -1;
        }
        memset(rec, 0, len);
        convertRecord(rec, src->p, o, t, is64);
        if (bufAppend(out, rec, len)) {
            cvErr(c, name, "out of memory", 0, 0);
            return -1;
        }
        o += 4 * n64w;
        n++;
        if (t == 48)
            break;
    }
    if (o != end) {
        uint32_t pad = end - o;
        if (end < o || pad >= 16) {
            cvErr(c, name, "propDefs: walk ended short of region end", o, end);
            return -1;
        }
        for (uint32_t k = o; k < end; k++) {
            if (src->p[k]) {
                cvErr(c, name, "propDefs: nonzero trailing pad", o, end);
                return -1;
            }
        }
        if (bufAppend(out, src->p + o, pad)) {
            cvErr(c, name, "out of memory", 0, 0);
            return -1;
        }
    }
    (void)n;
    return 0;
}

/* ------------------------------------------------------------ Usetup*Z ---- */

enum { F_PATHWP, F_WPGROUPS, F_INTRO, F_PROPDEFS, F_PATROL, F_AILISTS, F_PADS,
       F_BOUNDPADS, F_PADNAMES, F_BPNAMES };

static const uint8_t kIntroSz[10] = { 12, 16, 16, 32, 8, 8, 40, 12, 8, 4 };

enum { K_HEADER, K_GROW, K_INTRO, K_PROPDEFS, K_LEAF_S32, K_LEAF_CSTR, K_LEAF_AI, K_GAP };

typedef struct Rg {
    uint32_t start, end;
    int kind, tbl, order, endNone;
    int64_t delta;
} Rg;

typedef struct GrowTbl {
    int field;
    uint32_t recsize, termOff;
    int neg;
    uint32_t oldsz, newsz64;
    V32 starts;           /* record starts, excluding the terminator */
    uint32_t term;
    int has;
} GrowTbl;

static int cmpRg(const void *a, const void *b)
{
    const Rg *x = (const Rg *)a, *y = (const Rg *)b;
    if (x->start != y->start) return x->start < y->start ? -1 : 1;
    return x->order - y->order;
}

static int walkFixed(Ctx *c, const char *name, Src *s, GrowTbl *g, uint32_t off)
{
    g->has = 0;
    if (off == 0)
        return 0;
    uint32_t o = off;
    for (int guard = 0; guard < 100000; guard++) {
        uint32_t v = sBe32r(s, o + g->termOff);
        if (s->oob) {
            cvErr(c, name, "table walk past EOF", off, 0);
            return -1;
        }
        int term = g->neg ? ((int32_t)v < 0) : (v == 0);
        if (term) {
            g->term = o;
            g->has = 1;
            return 0;
        }
        if (v32Push(&g->starts, o)) {
            cvErr(c, name, "out of memory", 0, 0);
            return -1;
        }
        o += g->recsize;
    }
    cvErr(c, name, "runaway walk_fixed", off, 0);
    return -1;
}

static int64_t relocLookup(const Rg *r, size_t n, uint32_t off)
{
    if (off == 0)
        return 0;
    size_t lo = 0, hi = n;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (r[mid].start > off) hi = mid; else lo = mid + 1;
    }
    if (lo && r[lo - 1].start == off)
        return (int64_t)off + r[lo - 1].delta;
    return off;
}

static int leafEndOf(Ctx *c, const char *name, Src *s, const uint8_t *data, uint32_t D,
                     uint32_t start, int kind, uint32_t *end);

static int convertUsetup(Ctx *c, const char *name, const uint8_t *data, uint32_t D, Buf *out)
{
    int is64 = c->L->is64;
    Src sv = { data, D, 0 };
    Src *s = &sv;
    uint32_t H[10];
    GrowTbl gt[8];
    V32 introOff = {0}, introTy = {0};
    V32 leafStart = {0}, leafEnd = {0}, leafKind = {0};
    uint8_t *leafSeen = NULL, *aiSeen = NULL;
    Rg *rg = NULL, *tiled = NULL;
    size_t nrg = 0, caprg = 0, ntiled = 0;
    Buf pd = {0};
    Dst d = {0};
    int rc = -1, i;

    memset(gt, 0, sizeof(gt));
    if (D < 0x28) {
        cvErr(c, name, "too small", D, 0);
        return -1;
    }
    for (i = 0; i < 10; i++)
        H[i] = sBe32r(s, 4 * i);
    for (i = 0; i < 10; i++) {
        if (H[i] != 0 && H[i] >= D) {
            cvErr(c, name, "header field offset >= filesize", i, H[i]);
            return -1;
        }
    }

    /* GROW_TABLES order: pads, boundpads, waypointgroups, pathwaypoints,
     * patrolpaths, ailists, padnames, boundpadnames */
    static const struct { int field; uint32_t rec, termOff; int neg; uint32_t oldsz, newsz; } gd[8] = {
        { F_PADS, 44, 0x24, 0, 44, 56 }, { F_BOUNDPADS, 68, 0x24, 0, 68, 80 },
        { F_WPGROUPS, 12, 0, 0, 12, 24 }, { F_PATHWP, 16, 0, 1, 16, 24 },
        { F_PATROL, 8, 0, 0, 8, 16 }, { F_AILISTS, 8, 0, 0, 8, 16 },
        { F_PADNAMES, 4, 0, 0, 4, 8 }, { F_BPNAMES, 4, 0, 0, 4, 8 },
    };
    for (i = 0; i < 8; i++) {
        gt[i].field = gd[i].field; gt[i].recsize = gd[i].rec; gt[i].termOff = gd[i].termOff;
        gt[i].neg = gd[i].neg; gt[i].oldsz = gd[i].oldsz; gt[i].newsz64 = gd[i].newsz;
    }
    /* walk order in the Python: wp, wg, pp, al, pd, bp, pn, bpn */
    static const int walkOrder[8] = { 3, 2, 4, 5, 0, 1, 6, 7 };
    for (i = 0; i < 8; i++) {
        GrowTbl *g = &gt[walkOrder[i]];
        if (walkFixed(c, name, s, g, H[g->field]))
            goto done;
    }
#define TBL(f) (&gt[(f) == F_PADS ? 0 : (f) == F_BOUNDPADS ? 1 : (f) == F_WPGROUPS ? 2 : \
                    (f) == F_PATHWP ? 3 : (f) == F_PATROL ? 4 : (f) == F_AILISTS ? 5 : \
                    (f) == F_PADNAMES ? 6 : 7])

    /* intro: type-tag walk */
    uint32_t introEnd = H[F_INTRO];
    if (H[F_INTRO]) {
        uint32_t o = H[F_INTRO];
        int guard, done2 = 0;
        for (guard = 0; guard < 5000; guard++) {
            int32_t t = sBs32(s, o);
            if (s->oob || t < 0 || t > 9) {
                cvErr(c, name, "unknown intro type", (uint32_t)t, o);
                goto done;
            }
            if (v32Push(&introOff, o) || v32Push(&introTy, (uint32_t)t)) goto oom;
            o += kIntroSz[t];
            if (t == 9) { done2 = 1; break; }
        }
        if (!done2) {
            cvErr(c, name, "intro walk runaway", 0, 0);
            goto done;
        }
        introEnd = o;
    }

    leafSeen = (uint8_t *)calloc((size_t)D + 1, 1);
    aiSeen = (uint8_t *)calloc((size_t)D + 1, 1);
    if (!leafSeen || !aiSeen) goto oom;

    /* leaf sub-regions: s32 ID arrays / C strings */
#define ADD_LEAF(startv, kindv)                                                     \
    do {                                                                            \
        uint32_t st_ = (startv);                                                    \
        if (st_ != 0 && !leafSeen[st_ < D ? st_ : D]) {                             \
            if (st_ >= D) {                                                         \
                cvErr(c, name, "leaf offset outside file", st_, 0);                 \
                goto done;                                                          \
            }                                                                       \
            uint32_t en_ = 0;                                                       \
            if (leafEndOf(c, name, s, data, D, st_, (kindv), &en_)) goto done;      \
            leafSeen[st_] = 1;                                                      \
            if (v32Push(&leafStart, st_) || v32Push(&leafEnd, en_) ||               \
                v32Push(&leafKind, (kindv)))                                        \
                goto oom;                                                           \
        }                                                                           \
    } while (0)

    {
        GrowTbl *wp = TBL(F_PATHWP), *wg = TBL(F_WPGROUPS), *pp = TBL(F_PATROL);
        GrowTbl *pd_ = TBL(F_PADS), *bp = TBL(F_BOUNDPADS);
        GrowTbl *pn = TBL(F_PADNAMES), *bpn = TBL(F_BPNAMES), *al = TBL(F_AILISTS);
        size_t k;
        for (k = 0; k < wp->starts.n; k++) ADD_LEAF(sBe32r(s, wp->starts.p[k] + 4), 0);
        for (k = 0; k < wg->starts.n; k++) {
            ADD_LEAF(sBe32r(s, wg->starts.p[k] + 0), 0);
            ADD_LEAF(sBe32r(s, wg->starts.p[k] + 4), 0);
        }
        for (k = 0; k < pp->starts.n; k++) ADD_LEAF(sBe32r(s, pp->starts.p[k] + 0), 0);
        for (k = 0; k < pd_->starts.n; k++) ADD_LEAF(sBe32r(s, pd_->starts.p[k] + 0x24), 1);
        for (k = 0; k < bp->starts.n; k++) ADD_LEAF(sBe32r(s, bp->starts.p[k] + 0x24), 1);
        for (k = 0; k < pn->starts.n; k++) ADD_LEAF(sBe32r(s, pn->starts.p[k]), 1);
        for (k = 0; k < bpn->starts.n; k++) ADD_LEAF(sBe32r(s, bpn->starts.p[k]), 1);
        if (s->oob) {
            cvErr(c, name, "leaf scan past EOF", 0, 0);
            goto done;
        }
        /* AI opcode streams: boundary markers only */
        for (k = 0; k < al->starts.n; k++) {
            uint32_t a = sBe32r(s, al->starts.p[k]);
            if (a && a < D && !leafSeen[a]) aiSeen[a] = 1;
            else if (a >= D) {
                cvErr(c, name, "ailist offset outside file", a, 0);
                goto done;
            }
        }
    }

    /* region list */
#define RG_PUSH(st_, en_, kd_, tb_, none_)                                          \
    do {                                                                            \
        if (nrg == caprg) {                                                         \
            size_t nc_ = caprg ? caprg * 2 : 256;                                   \
            Rg *np_ = (Rg *)realloc(rg, nc_ * sizeof(Rg));                          \
            if (!np_) goto oom;                                                     \
            rg = np_; caprg = nc_;                                                  \
        }                                                                           \
        rg[nrg].start = (st_); rg[nrg].end = (en_); rg[nrg].kind = (kd_);           \
        rg[nrg].tbl = (tb_); rg[nrg].order = (int)nrg; rg[nrg].endNone = (none_);   \
        rg[nrg].delta = 0; nrg++;                                                   \
    } while (0)

    RG_PUSH(0, 0x28, K_HEADER, -1, 0);
    for (i = 0; i < 8; i++)
        if (gt[i].has)
            RG_PUSH(H[gt[i].field], gt[i].term + gt[i].oldsz, K_GROW, i, 0);
    if (H[F_INTRO])
        RG_PUSH(H[F_INTRO], introEnd, K_INTRO, -1, 0);
    if (H[F_PROPDEFS])
        RG_PUSH(H[F_PROPDEFS], 0, K_PROPDEFS, -1, 1);
    for (size_t k = 0; k < leafStart.n; k++)
        RG_PUSH(leafStart.p[k], leafEnd.p[k], leafKind.p[k] ? K_LEAF_CSTR : K_LEAF_S32, -1, 0);
    for (uint32_t a = 1; a < D; a++)
        if (aiSeen[a])
            RG_PUSH(a, 0, K_LEAF_AI, -1, 1);

    qsort(rg, nrg, sizeof(Rg), cmpRg);
    for (size_t k = 0; k < nrg; k++)
        if (rg[k].endNone)
            rg[k].end = (k + 1 < nrg) ? rg[k + 1].start : D;

    /* tile [0, D): unclaimed gaps become opaque passthrough regions */
    tiled = (Rg *)malloc((2 * nrg + 2) * sizeof(Rg));
    if (!tiled) goto oom;
    {
        uint32_t cursor = 0;
        for (size_t k = 0; k < nrg; k++) {
            if (rg[k].start < cursor) {
                cvErr(c, name, "region overlap", rg[k].start, cursor);
                goto done;
            }
            if (rg[k].start > cursor) {
                Rg g = { cursor, rg[k].start, K_GAP, -1, 0, 0, 0 };
                tiled[ntiled++] = g;
            }
            if (rg[k].end < rg[k].start) {
                cvErr(c, name, "region negative length", rg[k].start, rg[k].end);
                goto done;
            }
            tiled[ntiled++] = rg[k];
            cursor = rg[k].end;
        }
        if (cursor < D) {
            Rg g = { cursor, D, K_GAP, -1, 0, 0, 0 };
            tiled[ntiled++] = g;
        } else if (cursor > D) {
            cvErr(c, name, "regions overrun EOF", cursor, D);
            goto done;
        }
    }

    /* propDefs conversion (feeds the cumulative delta) */
    int havePd = 0;
    if (H[F_PROPDEFS]) {
        uint32_t pdEnd = 0;
        for (size_t k = 0; k < ntiled; k++)
            if (tiled[k].kind == K_PROPDEFS) { pdEnd = tiled[k].end; break; }
        if (cvConvertPropdefs(c, name, s, H[F_PROPDEFS], pdEnd, &pd))
            goto done;
        havePd = 1;
    }

    /* pass 1: cumulative delta at each region start */
    int64_t cum = is64 ? 0x28 : 0;
    for (size_t k = 0; k < ntiled; k++) {
        Rg *r = &tiled[k];
        if (r->kind == K_HEADER) { r->delta = 0; continue; }
        r->delta = cum;
        if (r->kind == K_GROW) {
            GrowTbl *g = &gt[r->tbl];
            uint32_t newsz = is64 ? g->newsz64 : g->oldsz;
            cum += (int64_t)(g->starts.n + 1) * ((int64_t)newsz - g->oldsz);
        } else if (r->kind == K_PROPDEFS && havePd) {
            cum += (int64_t)pd.len - (int64_t)(r->end - r->start);
        }
    }
#define RELOC(off_) ((uint32_t)relocLookup(tiled, ntiled, (off_)))

    if (dNew(&d, (size_t)((int64_t)D + cum)))
        goto oom;

    /* header */
    for (i = 0; i < 10; i++)
        dLe32(&d, (is64 ? 8u : 4u) * i, RELOC(H[i]));

    /* growing tables */
    for (i = 0; i < 8; i++) {
        GrowTbl *g = &gt[i];
        if (!g->has)
            continue;
        uint32_t newsz = is64 ? g->newsz64 : g->oldsz;
        uint32_t tableBase = RELOC(H[g->field]);
        for (size_t idx = 0; idx <= g->starts.n; idx++) {
            uint32_t so = idx < g->starts.n ? g->starts.p[idx] : g->term;
            uint32_t dst = tableBase + (uint32_t)idx * newsz;
            switch (g->field) {
            case F_PADS:
            case F_BOUNDPADS: {
                for (int k = 0; k < 9; k++)
                    dLe32(&d, dst + 4 * k, sBe32r(s, so + 4 * k));
                dLe32(&d, dst + O(0x28, 0x24), RELOC(sBe32r(s, so + 0x24)));
                if (g->field == F_BOUNDPADS)
                    for (int k = 0; k < 6; k++)
                        dLe32(&d, dst + O(0x38, 0x2c) + 4 * k, sBe32r(s, so + 0x2c + 4 * k));
                break;
            }
            case F_WPGROUPS:
                dLe32(&d, dst, RELOC(sBe32r(s, so)));
                dLe32(&d, dst + O(8, 4), RELOC(sBe32r(s, so + 4)));
                dLe32(&d, dst + O(16, 8), sBe32r(s, so + 8));
                break;
            case F_PATHWP:
                dLe32(&d, dst, sBe32r(s, so));
                dLe32(&d, dst + O(8, 4), RELOC(sBe32r(s, so + 4)));
                dLe32(&d, dst + O(16, 8), sBe32r(s, so + 8));
                dLe32(&d, dst + O(20, 12), sBe32r(s, so + 12));
                break;
            case F_PATROL:
                dLe32(&d, dst, RELOC(sBe32r(s, so)));
                dByte(&d, dst + O(8, 4), sByte(s, so + 4));
                dByte(&d, dst + O(9, 5), sByte(s, so + 5));
                dLe16(&d, dst + O(10, 6), sBu16(s, so + 6));
                break;
            case F_AILISTS:
                dLe32(&d, dst, RELOC(sBe32r(s, so)));
                dLe32(&d, dst + O(8, 4), sBe32r(s, so + 4));
                break;
            default:   /* padnames / boundpadnames */
                dLe32(&d, dst, RELOC(sBe32r(s, so)));
                break;
            }
        }
    }

    /* intro records */
    {
        uint32_t base = H[F_INTRO] ? (uint32_t)relocLookup(tiled, ntiled, H[F_INTRO]) : 0;
        for (size_t k = 0; k < introOff.n; k++) {
            uint32_t so = introOff.p[k], t = introTy.p[k], dst = base + (so - H[F_INTRO]);
            uint32_t sz = kIntroSz[t];
            for (uint32_t w = 0; w < sz / 4; w++) {
                if (t == 6 && w == 7) {   /* lang1c: two u16 halves, keep order */
                    dLe16(&d, dst + 4 * w, sBu16(s, so + 4 * w));
                    dLe16(&d, dst + 4 * w + 2, sBu16(s, so + 4 * w + 2));
                } else {
                    dLe32(&d, dst + 4 * w, sBe32r(s, so + 4 * w));
                }
            }
        }
    }

    /* leaves */
    for (size_t k = 0; k < leafStart.n; k++) {
        uint32_t st = leafStart.p[k], en = leafEnd.p[k], dst = RELOC(st);
        if (!leafKind.p[k]) {
            for (uint32_t o = st; o < en; o += 4)
                dLe32(&d, dst + (o - st), sBe32r(s, o));
        } else {
            dCopy(&d, dst, data + st, en - st);
        }
    }

    /* propdefs / ai streams / gaps */
    for (size_t k = 0; k < ntiled; k++) {
        Rg *r = &tiled[k];
        if (r->kind == K_PROPDEFS) {
            if (havePd)
                dCopy(&d, RELOC(r->start), pd.p, pd.len);
        } else if (r->kind == K_LEAF_AI || r->kind == K_GAP) {
            dCopy(&d, RELOC(r->start), data + r->start, r->end - r->start);
        }
    }

    if (s->oob || d.oob) {
        cvErr(c, name, "usetup: out-of-range access", 0, 0);
        goto done;
    }
    takeDst(out, &d);
    rc = 0;
    goto done;
oom:
    cvErr(c, name, "out of memory", 0, 0);
done:
    free(d.p);
    free(rg); free(tiled);
    free(leafSeen); free(aiSeen);
    bufFree(&pd);
    v32Free(&introOff); v32Free(&introTy);
    v32Free(&leafStart); v32Free(&leafEnd); v32Free(&leafKind);
    for (i = 0; i < 8; i++) v32Free(&gt[i].starts);
    return rc;
#undef TBL
#undef ADD_LEAF
#undef RG_PUSH
#undef RELOC
}

/* leaf extents (walk_s32arr / walk_cstr); declared before use via prototype */
static int leafEndOf(Ctx *c, const char *name, Src *s, const uint8_t *data, uint32_t D,
                     uint32_t start, int kind, uint32_t *end)
{
    if (kind == 0) {
        uint32_t o = start;
        for (int guard = 0; guard < 50000; guard++) {
            int32_t v = sBs32(s, o);
            if (s->oob) {
                cvErr(c, name, "s32 array runs past EOF", start, 0);
                return -1;
            }
            if (v < 0) {
                *end = o + 4;
                return 0;
            }
            o += 4;
        }
        cvErr(c, name, "runaway s32 array", start, 0);
        return -1;
    }
    for (uint32_t o = start; o < D; o++) {
        if (data[o] == 0) {
            *end = o + 1;
            return 0;
        }
    }
    cvErr(c, name, "unterminated string", start, 0);
    return -1;
}

/* ------------------------------------------------------------------ driver */

static int convertCompressed(Ctx *c, const char *name, uint32_t addr, uint32_t size,
                             int (*fn)(Ctx *, const char *, const uint8_t *, uint32_t, Buf *),
                             Sidecar *sc)
{
    Buf dec = {0}, conv = {0}, comp = {0};
    int rc = -1;

    if ((uint64_t)addr + size > c->romSize || size < 2) {
        cvErr(c, name, "filelist row outside ROM", addr, size);
        return -1;
    }
    const uint8_t *comp0 = c->rom + addr;
    if (comp0[0] != 0x11 || comp0[1] != 0x72) {
        cvErr(c, name, "bad RZ magic", comp0[0], comp0[1]);
        return -1;
    }
    if (cvInflateRaw(comp0 + 2, size - 2, &dec)) {
        cvErr(c, name, "decompress failed", addr, size);
        goto done;
    }
    if (fn(c, name, dec.p, (uint32_t)dec.len, &conv))
        goto done;
    if (cvDeflateRZ(conv.p, conv.len, &comp, CV_LEVEL(c))) {
        cvErr(c, name, "deflate failed", 0, 0);
        goto done;
    }
    if (scEmit(sc, name, comp.p, comp.len)) {
        cvErr(c, name, "out of memory", 0, 0);
        goto done;
    }
    rc = 0;
done:
    bufFree(&dec); bufFree(&conv); bufFree(&comp);
    return rc;
}

int cvConvertCg(Ctx *c, const PcConvTables *t, Sidecar *sc)
{
    int i;

    /* d69: bg .seg then Tbg_ stan, table order */
    for (i = 0; i < t->nCg; i++) {
        const PcConvCg *e = &t->cg[i];
        if (!e->isStan) {
            Buf conv = {0};
            if ((uint64_t)e->addr + e->size > c->romSize) {
                cvErr(c, e->name, "filelist row outside ROM", e->addr, e->size);
                continue;
            }
            if (convertSeg(c, e->name, c->rom + e->addr, e->size, &conv) == 0) {
                if (scEmit(sc, e->name, conv.p, conv.len))
                    cvErr(c, e->name, "out of memory", 0, 0);
            }
            bufFree(&conv);
        } else {
            convertCompressed(c, e->name, e->addr, e->size, convertStan, sc);
        }
    }
    /* d88: Usetup*Z appended */
    for (i = 0; i < t->nSetup; i++)
        convertCompressed(c, t->setup[i].name, t->setup[i].addr, t->setup[i].size,
                          convertUsetup, sc);
    return c->nerr ? -1 : 0;
}
