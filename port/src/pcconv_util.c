/*
 * pcconv_util.c — shared helpers + top-level driver for the in-process
 * sidecar converter (see port/include/pcconv.h).
 */

#include "pcconv_int.h"

const PcConvLayout kPcConvLayoutPC64 = { 1 };
const PcConvLayout kPcConvLayout32 = { 0 };

/* ------------------------------------------------------------------ errors */

void cvErr(Ctx *c, const char *name, const char *msg, uint32_t a, uint32_t b)
{
    c->nerr++;
    if (!c->err || c->errSize == 0 || c->errLen + 1 >= c->errSize)
        return;
    if (c->nerr > 40)
        return;
    int n = snprintf(c->err + c->errLen, c->errSize - c->errLen,
                     "%s: %s (0x%x 0x%x)\n", name ? name : "?", msg, a, b);
    if (n > 0) {
        c->errLen += (size_t)n;
        if (c->errLen >= c->errSize)
            c->errLen = c->errSize - 1;
    }
}

/* ------------------------------------------------------------------ buffer */

int bufReserve(Buf *b, size_t extra)
{
    if (b->cap - b->len >= extra)
        return 0;
    size_t ncap = b->cap ? b->cap : 4096;
    while (ncap - b->len < extra)
        ncap *= 2;
    uint8_t *np = (uint8_t *)realloc(b->p, ncap);
    if (!np)
        return -1;
    b->p = np;
    b->cap = ncap;
    return 0;
}

int bufAppend(Buf *b, const void *data, size_t n)
{
    if (bufReserve(b, n))
        return -1;
    if (n)
        memcpy(b->p + b->len, data, n);
    b->len += n;
    return 0;
}

int bufAppendZero(Buf *b, size_t n)
{
    if (bufReserve(b, n))
        return -1;
    if (n)
        memset(b->p + b->len, 0, n);
    b->len += n;
    return 0;
}

void bufFree(Buf *b)
{
    free(b->p);
    b->p = NULL;
    b->len = b->cap = 0;
}

/* -------------------------------------------------------------------- zlib */

int cvInflateRaw(const uint8_t *src, size_t n, Buf *out)
{
    z_stream z;
    memset(&z, 0, sizeof(z));
    if (inflateInit2(&z, -15) != Z_OK)
        return -1;
    z.next_in = (Bytef *)src;
    z.avail_in = (uInt)n;
    out->len = 0;
    int rc;
    do {
        if (bufReserve(out, 65536)) {
            inflateEnd(&z);
            return -1;
        }
        z.next_out = out->p + out->len;
        z.avail_out = (uInt)(out->cap - out->len);
        rc = inflate(&z, Z_NO_FLUSH);
        out->len = (size_t)(z.next_out - out->p);
    } while (rc == Z_OK);
    inflateEnd(&z);
    return rc == Z_STREAM_END ? 0 : -1;
}

int cvDeflateRZ(const uint8_t *src, size_t n, Buf *out, int level)
{
    z_stream z;
    memset(&z, 0, sizeof(z));
    if (deflateInit2(&z, level, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        return -1;
    size_t bound = (size_t)deflateBound(&z, (uLong)n) + 64;
    if (bufReserve(out, 2 + bound)) {
        deflateEnd(&z);
        return -1;
    }
    out->p[out->len++] = 0x11;
    out->p[out->len++] = 0x72;
    z.next_in = (Bytef *)src;
    z.avail_in = (uInt)n;
    z.next_out = out->p + out->len;
    z.avail_out = (uInt)bound;
    int rc = deflate(&z, Z_NO_FLUSH);
    if (rc != Z_OK && rc != Z_BUF_ERROR) {
        deflateEnd(&z);
        return -1;
    }
    do {
        rc = deflate(&z, Z_FINISH);
    } while (rc == Z_OK);
    if (rc != Z_STREAM_END) {
        deflateEnd(&z);
        return -1;
    }
    out->len = (size_t)(z.next_out - out->p);
    deflateEnd(&z);
    return 0;
}

/* --------------------------------------------------------------- sidecars */

static int csvRow(Buf *b, const char *name, size_t off, size_t size)
{
    char tmp[32];
    size_t nl = strlen(name);
    if (bufAppend(b, name, nl))
        return -1;
    if (bufAppend(b, ",", 1))
        return -1;
    int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)off);
    if (bufAppend(b, tmp, (size_t)n))
        return -1;
    if (bufAppend(b, ",", 1))
        return -1;
    n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)size);
    if (bufAppend(b, tmp, (size_t)n))
        return -1;
    return bufAppend(b, "\r\n", 2);
}

int scInit(Sidecar *s)
{
    memset(s, 0, sizeof(*s));
    /* Python's csv.writer terminates rows with \r\n */
    return bufAppend(&s->csv, "name,offset,size\r\n", 18);
}

int scEmit(Sidecar *s, const char *name, const uint8_t *data, size_t n)
{
    size_t start = (s->off + 15) & ~(size_t)15;
    if (start > s->off) {
        if (bufAppendZero(&s->bin, start - s->off))
            return -1;
        s->off = start;
    }
    if (csvRow(&s->csv, name, s->off, n))
        return -1;
    if (bufAppend(&s->bin, data, n))
        return -1;
    s->off += n;
    s->n++;
    return 0;
}

void scFree(Sidecar *s)
{
    bufFree(&s->bin);
    bufFree(&s->csv);
}

/* ------------------------------------------------------------------ driver */

static void moveBlob(PcConvBlob *dst, Buf *src)
{
    dst->data = src->p;
    dst->size = src->len;
    src->p = NULL;
    src->len = src->cap = 0;
}

void pcconvFree(PcConvResult *r)
{
    free(r->modelsBin.data);
    free(r->modelsCsv.data);
    free(r->cgBin.data);
    free(r->cgCsv.data);
    memset(r, 0, sizeof(*r));
}

int pcconvRun(const uint8_t *rom, uint32_t romSize, const PcConvTables *t,
              const PcConvLayout *layout, PcConvResult *out,
              char *err, size_t errSize)
{
    Ctx c;
    Sidecar models, cg;
    int rc = -1;

    memset(out, 0, sizeof(*out));
    memset(&c, 0, sizeof(c));
    c.rom = rom;
    c.romSize = romSize;
    c.L = layout;
    c.err = err;
    c.errSize = errSize;
    if (err && errSize)
        err[0] = 0;

    if (scInit(&models) || scInit(&cg)) {
        cvErr(&c, "pcconv", "out of memory", 0, 0);
        goto done;
    }
    if (cvConvertModels(&c, t, &models) || c.nerr)
        goto done;
    if (cvConvertCg(&c, t, &cg) || c.nerr)
        goto done;

    moveBlob(&out->modelsBin, &models.bin);
    moveBlob(&out->modelsCsv, &models.csv);
    moveBlob(&out->cgBin, &cg.bin);
    moveBlob(&out->cgCsv, &cg.csv);
    rc = 0;
done:
    scFree(&models);
    scFree(&cg);
    return rc;
}
