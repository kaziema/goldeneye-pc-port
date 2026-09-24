#ifndef PORT_PCCONV_INT_H
#define PORT_PCCONV_INT_H

/* Internal to pcconv*.c. Kept free of game headers so the same sources also
 * build in the standalone host harness (tools_pc/pcconv_host.c, defines
 * PCCONV_HOST). */

#include "pcconv.h"

#ifdef PCCONV_HOST
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>
#else
#include <string.h>
/* The port include path shadows <stdlib.h> with the decomp's N64 stub; take
 * the host header via the generated wrapper (same as C++ TUs). */
#include "hoststdlib.h"
extern int snprintf(char *str, size_t maxsize, const char *format, ...);
#include "realzlib.h"
#endif

#define MASK24 0xFFFFFFu

typedef struct Ctx {
    const uint8_t *rom;
    uint32_t romSize;
    const PcConvLayout *L;
    char *err;
    size_t errSize, errLen;
    int nerr;
} Ctx;

/* Vita (32-bit layout) favors first-launch speed over size. */
#define CV_LEVEL(c) ((c)->L->is64 ? 6 : 1)

void cvErr(Ctx *c, const char *name, const char *msg, uint32_t a, uint32_t b);

/* Growable byte buffer. */
typedef struct Buf {
    uint8_t *p;
    size_t len, cap;
} Buf;

int  bufReserve(Buf *b, size_t extra);
int  bufAppend(Buf *b, const void *data, size_t n);
int  bufAppendZero(Buf *b, size_t n);
void bufFree(Buf *b);

/* Big-endian reads with bounds checking against a source file image. */
typedef struct Src {
    const uint8_t *p;
    uint32_t n;
    int oob;
} Src;

static inline uint32_t rawBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static inline void putLe32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void putLe16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static inline int sOk(Src *s, uint32_t o, uint32_t len)
{
    if ((uint64_t)o + len > s->n) { s->oob = 1; return 0; }
    return 1;
}
/* raw 32-bit BE word */
static inline uint32_t sBe32r(Src *s, uint32_t o) { return sOk(s, o, 4) ? rawBe32(s->p + o) : 0; }
/* BE word masked to a 24-bit offset */
static inline uint32_t sBe32o(Src *s, uint32_t o) { return sBe32r(s, o) & MASK24; }
static inline int32_t  sBs32(Src *s, uint32_t o)  { return (int32_t)sBe32r(s, o); }
static inline uint32_t sBu16(Src *s, uint32_t o)
{
    return sOk(s, o, 2) ? (((uint32_t)s->p[o] << 8) | s->p[o + 1]) : 0;
}
static inline int32_t sBs16(Src *s, uint32_t o) { return (int16_t)sBu16(s, o); }
static inline uint32_t sByte(Src *s, uint32_t o) { return sOk(s, o, 1) ? s->p[o] : 0; }

static inline uint32_t round8(uint32_t x)  { return (x + 7u) & ~7u; }

/* Raw-deflate the RZ payload of rom[addr, addr+size) (skips the 2-byte
 * 0x11 0x72 header) into out. */
int cvInflateRaw(const uint8_t *src, size_t n, Buf *out);
/* out = 0x11 0x72 + raw deflate of src. Level 6 matches the Python
 * tools' zlib.compressobj(6, DEFLATED, -15) byte for byte. */
int cvDeflateRZ(const uint8_t *src, size_t n, Buf *out, int level);

/* Concatenated sidecar image + manifest.csv builder (16-aligned entries). */
typedef struct Sidecar {
    Buf bin, csv;
    size_t off;
    int n;
} Sidecar;
int  scInit(Sidecar *s);
int  scEmit(Sidecar *s, const char *name, const uint8_t *data, size_t n);
void scFree(Sidecar *s);

/* models / cg entry points */
int cvConvertModels(Ctx *c, const PcConvTables *t, Sidecar *sc);
int cvConvertCg(Ctx *c, const PcConvTables *t, Sidecar *sc);

/* propdefs (d88_propdefs.py) */
int cvConvertPropdefs(Ctx *c, const char *name, Src *src, uint32_t start,
                      uint32_t end, Buf *out);

#endif /* PORT_PCCONV_INT_H */
