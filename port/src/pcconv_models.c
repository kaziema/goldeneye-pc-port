/*
 * pcconv_models.c — port of tools_pc/d43_emit.py (model sidecars).
 *
 * Structure and naming follow the Python so the two can be diffed side by
 * side. Layout differences between the x86-64 sidecars and the 32-bit ones
 * are confined to the PW/NODE/REC/field-offset selections (see `DO`).
 */

#include "pcconv_int.h"

#define ADDR_OP_VTX 0x04u
#define ADDR_OP_SETTIMG 0xFDu

/* ------------------------------------------------------------ small vectors */

typedef struct U32V { uint32_t *p; size_t n, cap; } U32V;

static int u32Push(U32V *v, uint32_t x)
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
static void u32Free(U32V *v) { free(v->p); memset(v, 0, sizeof(*v)); }
static int u32Has(const U32V *v, uint32_t x)
{
    for (size_t i = 0; i < v->n; i++)
        if (v->p[i] == x)
            return 1;
    return 0;
}
static int cmpU32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : (x > y);
}
/* sort + unique in place */
static void u32SortUniq(U32V *v)
{
    if (v->n < 2)
        return;
    qsort(v->p, v->n, sizeof(uint32_t), cmpU32);
    size_t w = 1;
    for (size_t i = 1; i < v->n; i++)
        if (v->p[i] != v->p[w - 1])
            v->p[w++] = v->p[i];
    v->n = w;
}
/* first element strictly greater than x in a sorted vector, or dflt */
static uint32_t u32NextAbove(const U32V *v, uint32_t x, uint32_t dflt)
{
    size_t lo = 0, hi = v->n;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (v->p[mid] > x)
            hi = mid;
        else
            lo = mid + 1;
    }
    return lo < v->n ? v->p[lo] : dflt;
}

typedef struct Reg { int64_t old, osz, nw; } Reg;
typedef struct RegV { Reg *p; size_t n, cap; } RegV;

static int regPush(RegV *v, int64_t old, int64_t osz, int64_t nw)
{
    if (v->n == v->cap) {
        size_t nc = v->cap ? v->cap * 2 : 128;
        Reg *np = (Reg *)realloc(v->p, nc * sizeof(Reg));
        if (!np)
            return -1;
        v->p = np;
        v->cap = nc;
    }
    v->p[v->n].old = old;
    v->p[v->n].osz = osz;
    v->p[v->n].nw = nw;
    v->n++;
    return 0;
}
static int cmpRegOld(const void *a, const void *b)
{
    const Reg *x = (const Reg *)a, *y = (const Reg *)b;
    if (x->old != y->old) return x->old < y->old ? -1 : 1;
    if (x->osz != y->osz) return x->osz < y->osz ? -1 : 1;
    if (x->nw != y->nw) return x->nw < y->nw ? -1 : 1;
    return 0;
}
static int cmpRegNew(const void *a, const void *b)
{
    const Reg *x = (const Reg *)a, *y = (const Reg *)b;
    if (x->nw != y->nw) return x->nw < y->nw ? -1 : 1;
    return 0;
}

/* (key, value) pair map with last-write-wins, linear (per-file sets are tiny) */
typedef struct KV { uint32_t k; int64_t v; } KV;
typedef struct KVV { KV *p; size_t n, cap; } KVV;
static int kvSet(KVV *m, uint32_t k, int64_t v)
{
    for (size_t i = 0; i < m->n; i++)
        if (m->p[i].k == k) { m->p[i].v = v; return 0; }
    if (m->n == m->cap) {
        size_t nc = m->cap ? m->cap * 2 : 16;
        KV *np = (KV *)realloc(m->p, nc * sizeof(KV));
        if (!np)
            return -1;
        m->p = np;
        m->cap = nc;
    }
    m->p[m->n].k = k;
    m->p[m->n].v = v;
    m->n++;
    return 0;
}
static int kvGet(const KVV *m, uint32_t k, int64_t *out)
{
    for (size_t i = 0; i < m->n; i++)
        if (m->p[i].k == k) { *out = m->p[i].v; return 1; }
    return 0;
}
static void kvFree(KVV *m) { free(m->p); memset(m, 0, sizeof(*m)); }

/* ---------------------------------------------------------------- the node map */

typedef struct Node {
    uint32_t op, data, parent, next, prev, child;
    int present;
} Node;

typedef struct ModelCtx {
    Ctx *c;
    const char *name;
    Src src;
    uint32_t D;
    Node *nodes;      /* indexed by offset >> 2 */
    uint32_t R0;
    /* per-file layout state */
    RegV regions;
    U32V vtxOff, vtxN;          /* (vo, nv) pairs; nv as uint32 (signed via cast) */
    U32V zeroVtx;
    KVV op24Collision;          /* cvo -> 1 */
    KVV op24PointUsage;         /* puo -> nv */
    int64_t *nodeNew, *recNew, *gdlNew;   /* by offset >> 2 ; -1 = absent */
} ModelCtx;

static Node *nodeAt(ModelCtx *m, uint32_t o)
{
    if (o >= m->D || (o & 3))
        return NULL;
    Node *n = &m->nodes[o >> 2];
    return n->present ? n : NULL;
}


/* build_nodes(): closure MUST match d43_convert.walk() exactly */
static int buildNodes(ModelCtx *m, uint32_t NS, uint32_t NT)
{
    Src *s = &m->src;
    uint32_t R0 = 4 * NS + 12 * NT;
    U32V stack = {0};
    int rc = -1;
    m->R0 = R0;
    if (u32Push(&stack, R0))
        goto done;
    while (stack.n) {
        uint32_t o = stack.p[--stack.n];
        if (o >= m->D)
            continue;
        if (o & 3) {
            cvErr(m->c, m->name, "unaligned node", o, 0);
            goto done;
        }
        Node *n = &m->nodes[o >> 2];
        if (n->present)
            continue;
        n->op = sBu16(s, o) & 0xff;
        n->data = sBe32o(s, o + 4);
        n->parent = sBe32o(s, o + 8);
        n->next = sBe32o(s, o + 0xC);
        n->prev = sBe32o(s, o + 0x10);
        n->child = sBe32o(s, o + 0x14);
        n->present = 1;
        if (n->op == 8) {
            uint32_t q = sBe32o(s, n->data + 8);
            if (q && u32Push(&stack, q)) goto done;
        } else if (n->op == 18) {
            uint32_t q = sBe32o(s, n->data + 0);
            if (q && u32Push(&stack, q)) goto done;
        } else {
            if (n->child && u32Push(&stack, n->child)) goto done;
        }
        if (n->next && u32Push(&stack, n->next))
            goto done;
    }
    rc = 0;
done:
    u32Free(&stack);
    return rc;
}

static void gdlOf(ModelCtx *m, const Node *n, uint32_t *p, uint32_t *s)
{
    Src *src = &m->src;
    *p = *s = 0;
    if (n->op == 4 || n->op == 24) {
        *p = sBe32o(src, n->data + 0);
        *s = sBe32o(src, n->data + 4);
    } else if (n->op == 22) {
        *p = sBe32o(src, n->data + 8);
    }
}

/* visit_seq(): EXACT modelIterateDisplayLists GDL visit order (mutable walk) */
static int visitSeq(ModelCtx *m, U32V *seq)
{
    Src *src = &m->src;
    uint32_t node = m->R0;
    int havePrev = 0;
    uint32_t prevNode = 0, prevGdl = 0;
    int guard = 0;

    while (node && guard < 200000) {
        guard++;
        Node *n = nodeAt(m, node);
        if (!n) {
            cvErr(m->c, m->name, "visit: missing node", node, 0);
            return -1;
        }
        uint32_t gdl = 0;
        if (n->op == 4 || n->op == 22 || n->op == 24) {
            uint32_t p, s;
            gdlOf(m, n, &p, &s);
            if (!havePrev || node != prevNode)
                gdl = p;
            else if (s && s != prevGdl)
                gdl = s;
        } else if (n->op == 8) {           /* LOD rewire */
            n->child = sBe32o(src, n->data + 8);
        } else if (n->op == 18) {          /* SWITCH rewire */
            n->child = sBe32o(src, n->data + 0);
        } else if (n->op == 9) {           /* BSP splice (visible=TRUE) */
            uint32_t lc = sBe32o(src, n->data + 0x18);
            uint32_t rc2 = sBe32o(src, n->data + 0x1C);
            uint32_t node1 = lc, node2 = rc2;
            if (node1) {
                Node *n1 = nodeAt(m, node1);
                if (!n1) { cvErr(m->c, m->name, "bsp: missing left", node1, 0); return -1; }
                n->child = node1;
                n1->prev = 0;
                uint32_t loop = node1;
                for (;;) {
                    Node *nl = nodeAt(m, loop);
                    if (!nl) { cvErr(m->c, m->name, "bsp: missing node", loop, 0); return -1; }
                    if (!(nl->next && nl->next != node2))
                        break;
                    loop = nl->next;
                }
                nodeAt(m, loop)->next = node2;
                if (node2) {
                    Node *n2 = nodeAt(m, node2);
                    if (!n2) { cvErr(m->c, m->name, "bsp: missing right", node2, 0); return -1; }
                    n2->prev = loop;
                    loop = node2;
                    for (;;) {
                        Node *nl = nodeAt(m, loop);
                        if (!nl) { cvErr(m->c, m->name, "bsp: missing node", loop, 0); return -1; }
                        if (!(nl->next && nl->next != node1))
                            break;
                        loop = nl->next;
                    }
                    nodeAt(m, loop)->next = 0;
                }
            } else {
                n->child = node2;
                if (node2) {
                    Node *n2 = nodeAt(m, node2);
                    if (!n2) { cvErr(m->c, m->name, "bsp: missing right", node2, 0); return -1; }
                    n2->prev = 0;
                }
            }
        }
        if (gdl) {
            if (u32Push(seq, gdl))
                return -1;
            havePrev = 1;
            prevNode = node;
            prevGdl = gdl;
            continue;
        }
        havePrev = 0;
        if (n->child) {
            node = n->child;
        } else {
            while (node) {
                Node *cur = nodeAt(m, node);
                if (!cur) { cvErr(m->c, m->name, "visit: missing node", node, 0); return -1; }
                uint32_t nn = cur->next;
                if (nn) {
                    node = nn;
                    break;
                }
                node = cur->parent;
            }
        }
    }
    return 0;
}

/* placement_order(): d43_convert walk(): next-first DFS stack */
static int placementOrder(ModelCtx *m, U32V *outOff, U32V *outOp)
{
    Src *src = &m->src;
    U32V stack = {0};
    unsigned char *seen = (unsigned char *)calloc((m->D >> 2) + 2, 1);
    int rc = -1;
    if (!seen)
        return -1;
    if (u32Push(&stack, m->R0))
        goto done;
    while (stack.n) {
        uint32_t o = stack.p[--stack.n];
        if (o >= m->D)
            continue;
        if (o & 3) {
            cvErr(m->c, m->name, "unaligned node", o, 0);
            goto done;
        }
        if (seen[o >> 2])
            continue;
        seen[o >> 2] = 1;
        Node *n = nodeAt(m, o);
        if (!n) {
            cvErr(m->c, m->name, "placement: missing node", o, 0);
            goto done;
        }
        if (u32Push(outOff, o) || u32Push(outOp, n->op))
            goto done;
        uint32_t child = n->child, nxt = n->next;
        if (n->op == 8) {
            uint32_t a = sBe32o(src, n->data + 8);
            if (a && u32Push(&stack, a)) goto done;
        } else if (n->op == 18) {
            uint32_t cc = sBe32o(src, n->data + 0);
            if (cc && u32Push(&stack, cc)) goto done;
        } else {
            if (child && u32Push(&stack, child)) goto done;
        }
        if (nxt && u32Push(&stack, nxt))
            goto done;
    }
    rc = 0;
done:
    free(seen);
    u32Free(&stack);
    return rc;
}

static uint32_t gdlEnd(ModelCtx *m, uint32_t g)
{
    Src *src = &m->src;
    uint32_t o = g;
    while ((uint64_t)o + 8 <= m->D) {
        if ((sBe32r(src, o) >> 24) == 0xB8)
            return o + 8;
        o += 8;
    }
    return m->D;
}

/* ------------------------------------------------------------ layout tables */

/* PC record footprint per opcode; 0 = not a known record */
static uint32_t recSize(int is64, uint32_t op)
{
    static const uint8_t pc[32] = { [1] = 24, [2] = 40, [3] = 40, [4] = 40,
        [8] = 24, [9] = 48, [10] = 28, [12] = 48, [13] = 48, [15] = 28,
        [18] = 16, [21] = 20, [22] = 32, [23] = 2, [24] = 64 };
    static const uint8_t p32[32] = { [1] = 16, [2] = 28, [3] = 28, [4] = 20,
        [8] = 16, [9] = 36, [10] = 28, [12] = 40, [13] = 32, [15] = 28,
        [18] = 8, [21] = 20, [22] = 16, [23] = 2, [24] = 32 };
    if (op >= 32)
        return 0;
    return is64 ? pc[op] : p32[op];
}

/* dst offset of a field: x86-64 layout vs the N64/32-bit (identity) one */
#define DO(is64, pcoff, n64off) ((is64) ? (uint32_t)(pcoff) : (uint32_t)(n64off))

/* ------------------------------------------------------------------ one file */

typedef struct Out {
    uint8_t *buf;
    uint32_t len;
} Out;

static int64_t remapOff(const Reg *rs, size_t n, uint32_t off, int *ok)
{
    *ok = 1;
    if (off == 0)
        return 0;
    /* bisect_right on old offsets, then -1 */
    size_t lo = 0, hi = n;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (rs[mid].old > (int64_t)off)
            hi = mid;
        else
            lo = mid + 1;
    }
    if (lo == 0) { *ok = 0; return 0; }
    const Reg *r = &rs[lo - 1];
    if (r->old <= (int64_t)off && (int64_t)off < r->old + r->osz)
        return r->nw + ((int64_t)off - r->old);
    *ok = 0;
    return 0;
}

typedef struct Emit {
    ModelCtx *m;
    uint8_t *buf;
    uint32_t dpc;
    int is64;
    const Reg *rs;      /* sorted by old */
    size_t nrs;
} Emit;

static int64_t eRemap(Emit *e, uint32_t off, int *ok) { return remapOff(e->rs, e->nrs, off, ok); }

static void putPtr(Emit *e, uint32_t o, uint32_t oldOff)
{
    if (!oldOff)
        return;
    int ok;
    int64_t n = eRemap(e, oldOff, &ok);
    if (!ok) {
        cvErr(e->m->c, e->m->name, "ptr not in region map", oldOff, o);
        n = 0;
    }
    if ((uint64_t)o + (e->is64 ? 8 : 4) > e->dpc) {
        cvErr(e->m->c, e->m->name, "emit overrun", o, 0);
        return;
    }
    putLe32(e->buf + o, 0x05000000u | ((uint32_t)n & MASK24));
    if (e->is64) {
        e->buf[o + 4] = e->buf[o + 5] = e->buf[o + 6] = e->buf[o + 7] = 0;
    }
}

static int eChk(Emit *e, uint32_t o, uint32_t len)
{
    if ((uint64_t)o + len > e->dpc) {
        cvErr(e->m->c, e->m->name, "emit overrun", o, len);
        return 0;
    }
    return 1;
}
static void putU16(Emit *e, uint32_t o, uint32_t v) { if (eChk(e, o, 2)) putLe16(e->buf + o, v); }
static void putU32(Emit *e, uint32_t o, uint32_t v) { if (eChk(e, o, 4)) putLe32(e->buf + o, v); }
/* bswap32 of a source word (BE -> LE), bit-pattern preserving */
static void putF32(Emit *e, uint32_t o, uint32_t doff)
{
    Src *s = &e->m->src;
    if (eChk(e, o, 4) && sOk(s, doff, 4)) {
        e->buf[o] = s->p[doff + 3]; e->buf[o + 1] = s->p[doff + 2];
        e->buf[o + 2] = s->p[doff + 1]; e->buf[o + 3] = s->p[doff];
    }
}
static void putBytes(Emit *e, uint32_t o, uint32_t doff, uint32_t n)
{
    Src *s = &e->m->src;
    if (eChk(e, o, n) && sOk(s, doff, n))
        memcpy(e->buf + o, s->p + doff, n);
}

static int64_t gdlNewOff(ModelCtx *m, uint32_t g)
{
    if (g >= m->D || (g & 3))
        return -1;
    return m->gdlNew[g >> 2];
}

static void putGdlPtr(Emit *e, uint32_t o, uint32_t g)
{
    if (!g)
        return;
    int64_t n = gdlNewOff(e->m, g);
    if (n < 0) {
        cvErr(e->m->c, e->m->name, "GDL ptr without slot", g, o);
        return;
    }
    if (eChk(e, o, 4))
        putLe32(e->buf + o, 0x05000000u | ((uint32_t)n & MASK24));
}

static void emitVtxMain(Emit *e, uint32_t vo, int32_t nv, uint32_t npos)
{
    Src *s = &e->m->src;
    for (int32_t i = 0; i < nv; i++) {
        uint32_t o = vo + 16u * (uint32_t)i, q = npos + 16u * (uint32_t)i;
        if (!eChk(e, q, 16) || !sOk(s, o, 16))
            return;
        for (int k = 0; k < 6; k++) {          /* 6 x s16 */
            e->buf[q + 2 * k] = s->p[o + 2 * k + 1];
            e->buf[q + 2 * k + 1] = s->p[o + 2 * k];
        }
        memcpy(e->buf + q + 12, s->p + o + 12, 4);
    }
}

static void emitVtxCollision(Emit *e, uint32_t vo, int32_t nv, uint32_t npos)
{
    Src *s = &e->m->src;
    for (int32_t i = 0; i < nv; i++) {
        uint32_t o = vo + 16u * (uint32_t)i, q = npos + 16u * (uint32_t)i;
        if (!eChk(e, q, 16) || !sOk(s, o, 16))
            return;
        for (int k = 0; k < 4; k++) {
            e->buf[q + 2 * k] = s->p[o + 2 * k + 1];
            e->buf[q + 2 * k + 1] = s->p[o + 2 * k];
        }
        uint32_t v = rawBe32(s->p + o + 8);
        if (v == 0) {
            /* leave zero */
        } else if ((v >> 24) == 5) {
            int ok;
            int64_t n = eRemap(e, v & MASK24, &ok);
            if (!ok) {
                cvErr(e->m->c, e->m->name, "LinkedTo not in map", v, 0);
                n = 0;
            }
            putLe32(e->buf + q + 8, 0x05000000u | ((uint32_t)n & MASK24));
        } else {
            cvErr(e->m->c, e->m->name, "LinkedTo neither null nor seg-5", v, 0);
        }
        e->buf[q + 12] = s->p[o + 13]; e->buf[q + 13] = s->p[o + 12];
        e->buf[q + 14] = s->p[o + 15]; e->buf[q + 15] = s->p[o + 14];
    }
}

/* record fields for one node; r = new offset of the record, d = N64 offset */
static void emitRecord(Emit *e, uint32_t op, uint32_t r, uint32_t d)
{
    ModelCtx *m = e->m;
    Src *src = &m->src;
    int is64 = e->is64;
    uint32_t i;

    switch (op) {
    case 1:   /* HeaderRecord */
        putU16(e, r, sBu16(src, d));
        putU16(e, r + 2, (uint32_t)sBs16(src, d + 2));
        putPtr(e, r + DO(is64, 8, 4), sBe32o(src, d + 4));
        putU16(e, r + DO(is64, 16, 8), sBu16(src, d + 8));
        putU16(e, r + DO(is64, 18, 0xA), sBu16(src, d + 0xA));
        putU16(e, r + DO(is64, 20, 0xC), sBu16(src, d + 0xC));
        break;
    case 2: case 3:   /* GroupRecord / OP03 */
        for (i = 0; i < 3; i++) putF32(e, r + 4 * i, d + 4 * i);
        putU16(e, r + 12, sBu16(src, d + 0xC));
        for (i = 0; i < 3; i++) putU16(e, r + 14 + 2 * i, (uint32_t)sBs16(src, d + 0xE + 2 * i));
        putPtr(e, r + DO(is64, 24, 0x14), sBe32o(src, d + 0x14));
        putF32(e, r + DO(is64, 32, 0x18), d + 0x18);
        break;
    case 4: { /* DisplayListRecord */
        uint32_t p = sBe32o(src, d), s2 = sBe32o(src, d + 4);
        putGdlPtr(e, r, p);
        putGdlPtr(e, r + DO(is64, 8, 4), s2);
        putPtr(e, r + DO(is64, 24, 0xC), sBe32o(src, d + 0xC));
        putU16(e, r + DO(is64, 32, 0x10), sBu16(src, d + 0x10));
        if (eChk(e, r + DO(is64, 34, 0x12), 1))
            e->buf[r + DO(is64, 34, 0x12)] = (uint8_t)sByte(src, d + 0x12);
        break;
    }
    case 8:   /* LODRecord */
        putF32(e, r, d); putF32(e, r + 4, d + 4);
        putPtr(e, r + 8, sBe32o(src, d + 8));
        putU16(e, r + DO(is64, 16, 0xC), sBu16(src, d + 0xC));
        break;
    case 9:   /* BSPRecord */
        for (i = 0; i < 3; i++) putF32(e, r + 4 * i, d + 4 * i);
        for (i = 0; i < 3; i++) putF32(e, r + 12 + 4 * i, d + 0xC + 4 * i);
        putPtr(e, r + DO(is64, 24, 0x18), sBe32o(src, d + 0x18));
        putPtr(e, r + DO(is64, 32, 0x1C), sBe32o(src, d + 0x1C));
        putU16(e, r + DO(is64, 42, 0x22), sBu16(src, d + 0x22));
        break;
    case 10:  /* BoundingBoxRecord */
        putU32(e, r, sBe32r(src, d));
        for (i = 0; i < 6; i++) putF32(e, r + 4 + 4 * i, d + 4 + 4 * i);
        break;
    case 12:  /* GunfireRecord */
        for (i = 0; i < 3; i++) putF32(e, r + 4 * i, d + 4 * i);
        for (i = 0; i < 3; i++) putF32(e, r + 12 + 4 * i, d + 0xC + 4 * i);
        putPtr(e, r + DO(is64, 24, 0x18), sBe32o(src, d + 0x18));
        putF32(e, r + DO(is64, 32, 0x1C), d + 0x1C);
        putU16(e, r + DO(is64, 36, 0x20), sBu16(src, d + 0x20));
        break;
    case 13:  /* ShadowRecord */
        for (i = 0; i < 2; i++) putF32(e, r + 4 * i, d + 4 * i);
        for (i = 0; i < 2; i++) putF32(e, r + 8 + 4 * i, d + 8 + 4 * i);
        putPtr(e, r + DO(is64, 16, 0x10), sBe32o(src, d + 0x10));
        putPtr(e, r + DO(is64, 24, 0x14), sBe32o(src, d + 0x14));
        putF32(e, r + DO(is64, 32, 0x18), d + 0x18);
        break;
    case 15:  /* InterlinkageRecord */
        for (i = 0; i < 3; i++) putF32(e, r + 4 * i, d + 4 * i);
        for (i = 0; i < 3; i++) putF32(e, r + 12 + 4 * i, d + 0xC + 4 * i);
        putF32(e, r + 24, d + 0x18);
        break;
    case 18:  /* SwitchRecord */
        putPtr(e, r, sBe32o(src, d));
        putU16(e, r + DO(is64, 8, 4), sBu16(src, d + 4));
        break;
    case 21:  /* GroupSimpleRecord */
        for (i = 0; i < 3; i++) putF32(e, r + 4 * i, d + 4 * i);
        putU16(e, r + 12, (uint32_t)sBs16(src, d + 0xC));
        putU16(e, r + 14, sBu16(src, d + 0xE));
        putF32(e, r + 16, d + 0x10);
        break;
    case 22: { /* DisplayListPrimaryRecord */
        putU32(e, r, sBe32r(src, d));
        putPtr(e, r + DO(is64, 8, 4), sBe32o(src, d + 4));
        putGdlPtr(e, r + DO(is64, 16, 8), sBe32o(src, d + 8));
        break;
    }
    case 23:  /* HeadPlaceholderRecord */
        putU16(e, r, sBu16(src, d));
        break;
    case 24: { /* DisplayList_CollisionRecord */
        uint32_t p = sBe32o(src, d), s2 = sBe32o(src, d + 4);
        putGdlPtr(e, r, p);
        putGdlPtr(e, r + DO(is64, 8, 4), s2);
        putPtr(e, r + DO(is64, 16, 8), sBe32o(src, d + 8));
        putU16(e, r + DO(is64, 24, 0xC), (uint32_t)sBs16(src, d + 0xC));
        putU16(e, r + DO(is64, 26, 0xE), (uint32_t)sBs16(src, d + 0xE));
        putPtr(e, r + DO(is64, 32, 0x10), sBe32o(src, d + 0x10));
        putPtr(e, r + DO(is64, 40, 0x14), sBe32o(src, d + 0x14));
        putU16(e, r + DO(is64, 48, 0x18), (uint32_t)sBs16(src, d + 0x18));
        putU16(e, r + DO(is64, 50, 0x1A), sBu16(src, d + 0x1A));
        break;
    }
    default:
        break;
    }
}

/* PC/32-bit offsets of pointer fields per (op, n64 field), for validation */
static int ptrFieldOff(int is64, uint32_t op, uint32_t f, uint32_t *dst)
{
    static const struct { uint8_t op, f, pc; } t[] = {
        {1, 4, 8}, {2, 0x14, 24}, {3, 0x14, 24}, {8, 8, 8},
        {9, 0x18, 24}, {9, 0x1C, 32}, {12, 0x18, 24}, {13, 0x10, 16},
        {13, 0x14, 24}, {18, 0, 0}, {24, 8, 16}, {24, 0x10, 32},
        {24, 0x14, 40}, {4, 0xC, 24}, {22, 4, 8},
    };
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        if (t[i].op == op && t[i].f == f) {
            *dst = is64 ? t[i].pc : f;
            return 1;
        }
    }
    return 0;
}

static uint64_t readPtrLe(const uint8_t *b, int is64)
{
    uint64_t v = (uint64_t)b[0] | ((uint64_t)b[1] << 8) | ((uint64_t)b[2] << 16) | ((uint64_t)b[3] << 24);
    if (is64)
        v |= ((uint64_t)b[4] << 32) | ((uint64_t)b[5] << 40) | ((uint64_t)b[6] << 48) | ((uint64_t)b[7] << 56);
    return v;
}
static uint32_t rdLe32(const uint8_t *b) { return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24); }

/* round-trip validation (validate() in d43_emit.py) */
static void validateModel(ModelCtx *m, Emit *e, const U32V *placedOff, const U32V *placedOp,
                          const U32V *allGdls)
{
    Src *src = &m->src;
    Ctx *c = m->c;
    int is64 = e->is64;
    const char *nm = m->name;
    uint32_t D = m->D, DPC = e->dpc;

    /* tiling: regions cover [0, D_PC) exactly */
    {
        Reg *rs = (Reg *)malloc(m->regions.n * sizeof(Reg));
        if (!rs) { cvErr(c, nm, "RT oom", 0, 0); return; }
        memcpy(rs, m->regions.p, m->regions.n * sizeof(Reg));
        qsort(rs, m->regions.n, sizeof(Reg), cmpRegNew);   /* stable enough: only nw compared */
        int64_t pos = 0;
        for (size_t i = 0; i < m->regions.n; i++) {
            if (rs[i].nw != pos || rs[i].nw + rs[i].osz > (int64_t)DPC) {
                cvErr(c, nm, "RT tiling gap/overlap", (uint32_t)pos, (uint32_t)rs[i].old);
                free(rs);
                return;
            }
            pos += rs[i].osz;
        }
        free(rs);
        if (pos != (int64_t)DPC) {
            cvErr(c, nm, "RT tiling ends short", (uint32_t)pos, DPC);
            return;
        }
    }

    /* node fields */
    static const uint8_t nf[5][2] = { {4, 8}, {8, 16}, {0xC, 24}, {0x10, 32}, {0x14, 40} };
    for (size_t k = 0; k < placedOff->n; k++) {
        uint32_t no = placedOff->p[k];
        int64_t nposv = m->nodeNew[no >> 2];
        uint32_t npos = (uint32_t)nposv;
        if (sBu16(src, no) != (uint32_t)(e->buf[npos] | (e->buf[npos + 1] << 8)))
            cvErr(c, nm, "RT node opcode mismatch", no, 0);
        for (int f = 0; f < 5; f++) {
            uint32_t old = sBe32o(src, no + nf[f][0]);
            uint32_t pf = is64 ? nf[f][1] : nf[f][0];
            uint64_t v = readPtrLe(e->buf + npos + pf, is64);
            if (old == 0) {
                if (v != 0) cvErr(c, nm, "RT node: expected null", no, nf[f][0]);
            } else {
                int ok;
                int64_t want = eRemap(e, old, &ok);
                if ((v >> 24) != 5 || (v & MASK24) != (uint64_t)want)
                    cvErr(c, nm, "RT node ptr mismatch", no, nf[f][0]);
            }
        }
    }

    /* record pointer fields */
    static const struct { uint8_t op, n; uint8_t f[3]; } pfl[] = {
        {1, 1, {4}}, {2, 1, {0x14}}, {3, 1, {0x14}}, {8, 1, {8}},
        {9, 2, {0x18, 0x1C}}, {12, 1, {0x18}}, {13, 2, {0x10, 0x14}},
        {18, 1, {0}}, {24, 3, {8, 0x10, 0x14}}, {4, 1, {0xC}}, {22, 1, {4}},
    };
    for (size_t k = 0; k < placedOff->n; k++) {
        uint32_t no = placedOff->p[k], op = placedOp->p[k];
        uint32_t d = sBe32o(src, no + 4);
        int64_t r = m->recNew[d >> 2];
        for (size_t t = 0; t < sizeof(pfl) / sizeof(pfl[0]); t++) {
            if (pfl[t].op != op)
                continue;
            for (int q = 0; q < pfl[t].n; q++) {
                uint32_t f = pfl[t].f[q], dst;
                if (!ptrFieldOff(is64, op, f, &dst))
                    continue;
                uint32_t old = sBe32o(src, d + f);
                uint64_t v = readPtrLe(e->buf + (uint32_t)r + dst, is64);
                if (old == 0) {
                    if (v != 0) cvErr(c, nm, "RT rec: expected null", d, f);
                } else {
                    int ok;
                    int64_t want = eRemap(e, old, &ok);
                    if ((v >> 24) != 5 || (v & MASK24) != (uint64_t)want)
                        cvErr(c, nm, "RT rec ptr mismatch", d, f);
                }
            }
        }
    }

    /* GDL slots */
    for (size_t k = 0; k < allGdls->n; k++) {
        uint32_t g = allGdls->p[k];
        uint32_t end = gdlEnd(m, g);
        uint32_t npos = (uint32_t)m->gdlNew[g >> 2];
        uint32_t j = 0, o = g;
        while ((uint64_t)o + 8 <= end) {
            uint32_t w0 = sBe32r(src, o), w1 = sBe32r(src, o + 4);
            uint32_t q = npos + 16 * j;
            uint32_t c0 = w0 >> 24;
            uint32_t w1off = is64 ? 8 : 4;
            if (rdLe32(e->buf + q) != w0)
                cvErr(c, nm, "RT gdl w0 mismatch", g, j);
            if (is64) {
                if (rdLe32(e->buf + q + 4) != 0) cvErr(c, nm, "RT gdl pad[4..8) nonzero", g, j);
                if (rdLe32(e->buf + q + 12) != 0) cvErr(c, nm, "RT gdl pad[12..16) nonzero", g, j);
            } else {
                if (rdLe32(e->buf + q + 8) != 0 || rdLe32(e->buf + q + 12) != 0)
                    cvErr(c, nm, "RT gdl pad[8..16) nonzero", g, j);
            }
            uint32_t w1p = rdLe32(e->buf + q + w1off);
            if ((c0 == ADDR_OP_VTX || c0 == ADDR_OP_SETTIMG) && (w1 >> 24) == 5) {
                int ok;
                int64_t want = eRemap(e, w1 & MASK24, &ok);
                if (w1p != (0x05000000u | (uint32_t)want))
                    cvErr(c, nm, "RT gdl w1 remap mismatch", g, j);
            } else if (w1p != w1) {
                cvErr(c, nm, "RT gdl w1 passthrough mismatch", g, j);
            }
            if (c0 == 0xB8)
                break;
            j++;
            o += 8;
        }
    }

    /* PointUsage */
    for (size_t k = 0; k < m->op24PointUsage.n; k++) {
        uint32_t puo = m->op24PointUsage.p[k].k;
        int32_t punv = (int32_t)m->op24PointUsage.p[k].v;
        int ok;
        int64_t pn = eRemap(e, puo, &ok);
        if (!ok) { cvErr(c, nm, "RT PointUsage not in region map", puo, 0); continue; }
        int allzero = 1;
        for (int32_t q = 0; q < punv; q++) {
            int32_t want = sBs16(src, puo + 2u * (uint32_t)q);
            int32_t got = (int16_t)(e->buf[(uint32_t)pn + 2u * q] | (e->buf[(uint32_t)pn + 2u * q + 1] << 8));
            if (got != want) cvErr(c, nm, "RT PointUsage mismatch", puo, (uint32_t)q);
            if (got != 0) allzero = 0;
            if (got >= punv) cvErr(c, nm, "RT PointUsage >= numVertices", puo, (uint32_t)q);
        }
        if (allzero && punv > 1)
            cvErr(c, nm, "RT PointUsage all zero (emit gap?)", puo, (uint32_t)punv);
    }
    (void)D;
}

static int64_t addRegion(RegV *rv, int64_t old, int64_t osz, int64_t cur)
{
    regPush(rv, old, osz, cur);
    return cur + osz;
}

static int processModel(Ctx *c, const PcConvModel *pm, Sidecar *sc)
{
    ModelCtx mc;
    Buf dec = {0}, comp = {0};
    U32V seq = {0}, placedOff = {0}, placedOp = {0}, refG = {0}, unvisited = {0}, allG = {0};
    U32V objOffs = {0}, blobT = {0}, mergedS = {0}, mergedE = {0}, placedOffs = {0};
    uint8_t *buf = NULL;
    int rc = -1;
    int is64 = c->L->is64;
    const uint32_t PW = is64 ? 8 : 4;
    const uint32_t NODE = is64 ? 48 : 24;
    Emit e;

    memset(&mc, 0, sizeof(mc));
    mc.c = c;
    mc.name = pm->name;
    int nerr0 = c->nerr;

    if ((uint64_t)pm->addr + pm->size > c->romSize || pm->size < 2) {
        cvErr(c, pm->name, "filelist row outside ROM", pm->addr, pm->size);
        return -1;
    }
    if (cvInflateRaw(c->rom + pm->addr + 2, pm->size - 2, &dec)) {
        cvErr(c, pm->name, "inflate failed", pm->addr, pm->size);
        goto done;
    }
    mc.src.p = dec.p;
    mc.src.n = (uint32_t)dec.len;
    mc.D = (uint32_t)dec.len;
    uint32_t D = mc.D;
    uint32_t NS = pm->ns, NT = pm->nt;

    mc.nodes = (Node *)calloc((D >> 2) + 2, sizeof(Node));
    mc.nodeNew = (int64_t *)malloc(((D >> 2) + 2) * sizeof(int64_t));
    mc.recNew = (int64_t *)malloc(((D >> 2) + 2) * sizeof(int64_t));
    mc.gdlNew = (int64_t *)malloc(((D >> 2) + 2) * sizeof(int64_t));
    if (!mc.nodes || !mc.nodeNew || !mc.recNew || !mc.gdlNew) {
        cvErr(c, pm->name, "out of memory", 0, 0);
        goto done;
    }
    memset(mc.nodeNew, 0xFF, ((D >> 2) + 2) * sizeof(int64_t));
    memset(mc.recNew, 0xFF, ((D >> 2) + 2) * sizeof(int64_t));
    memset(mc.gdlNew, 0xFF, ((D >> 2) + 2) * sizeof(int64_t));

    if (buildNodes(&mc, NS, NT))
        goto done;
    {
        int any = 0;
        for (uint32_t i = 0; i < (D >> 2) + 1 && !any; i++) any = mc.nodes[i].present;
        if (!any) { cvErr(c, pm->name, "empty tree", 0, 0); goto done; }
    }
    if (visitSeq(&mc, &seq))
        goto done;
    if (placementOrder(&mc, &placedOff, &placedOp))
        goto done;

    /* ---- layout pass (d43_convert verbatim) ---- */
    RegV *regions = &mc.regions;
    int64_t dstpos = 0;
    for (uint32_t i = 0; i < NS; i++)
        dstpos = addRegion(regions, 4 * i, PW, dstpos);
    for (uint32_t i = 0; i < NT; i++)
        dstpos = addRegion(regions, 4 * NS + 12 * i, 12, dstpos);

    for (size_t k = 0; k < placedOff.n; k++) {
        uint32_t no = placedOff.p[k], op = placedOp.p[k];
        uint32_t data = sBe32o(&mc.src, no + 4);
        dstpos = addRegion(regions, no, NODE, dstpos);
        mc.nodeNew[no >> 2] = dstpos - NODE;
        uint32_t psz = recSize(is64, op);
        if (op == 17 || psz == 0) {
            cvErr(c, pm->name, "unknown opcode", op, no);
            continue;
        }
        if (data >= D || (data & 3)) {
            cvErr(c, pm->name, "record offset unaligned/outside", data, no);
            continue;
        }
        mc.recNew[data >> 2] = dstpos;
        dstpos = addRegion(regions, data, psz, dstpos);
        if (op == 4) {
            uint32_t nv = sBu16(&mc.src, data + 0x10), vo = sBe32o(&mc.src, data + 0xC);
            if (vo) {
                if (nv == 0) {
                    u32Push(&mc.zeroVtx, vo);
                } else {
                    u32Push(&mc.vtxOff, vo); u32Push(&mc.vtxN, nv);
                    dstpos = addRegion(regions, vo, 16 * nv, dstpos);
                }
            }
        } else if (op == 24) {
            int32_t nv = sBs16(&mc.src, data + 0xC), ncv = sBs16(&mc.src, data + 0xE);
            uint32_t vo = sBe32o(&mc.src, data + 8), cvo = sBe32o(&mc.src, data + 0x10);
            uint32_t puo = sBe32o(&mc.src, data + 0x14);
            if (nv && vo) {
                u32Push(&mc.vtxOff, vo); u32Push(&mc.vtxN, (uint32_t)nv);
                dstpos = addRegion(regions, vo, 16 * (int64_t)nv, dstpos);
            }
            if (ncv && cvo) {
                kvSet(&mc.op24Collision, cvo, 1);
                u32Push(&mc.vtxOff, cvo); u32Push(&mc.vtxN, (uint32_t)ncv);
                dstpos = addRegion(regions, cvo, 16 * (int64_t)ncv, dstpos);
            }
            if (nv && puo) {
                dstpos = addRegion(regions, puo, 2 * (int64_t)nv, dstpos);
                kvSet(&mc.op24PointUsage, puo, nv);
            }
        } else if (op == 22) {
            int32_t nv = sBs32(&mc.src, data);
            uint32_t vo = sBe32o(&mc.src, data + 4);
            if (nv && vo) {
                u32Push(&mc.vtxOff, vo); u32Push(&mc.vtxN, (uint32_t)nv);
                dstpos = addRegion(regions, vo, 16 * (int64_t)nv, dstpos);
            }
        }
    }
    if (mc.src.oob) { cvErr(c, pm->name, "layout: read past end of file", 0, 0); goto done; }
    if (c->nerr > nerr0)
        goto done;

    /* All record-referenced GDLs (unique, in placement order) */
    for (size_t k = 0; k < placedOff.n; k++) {
        uint32_t no = placedOff.p[k], op = placedOp.p[k];
        uint32_t data = sBe32o(&mc.src, no + 4);
        if (op == 4 || op == 24) {
            for (uint32_t off = 0; off <= 4; off += 4) {
                uint32_t q = sBe32o(&mc.src, data + off);
                if (q && !u32Has(&refG, q)) u32Push(&refG, q);
            }
        } else if (op == 22) {
            uint32_t q = sBe32o(&mc.src, data + 8);
            if (q && !u32Has(&refG, q)) u32Push(&refG, q);
        }
    }
    for (size_t k = 0; k < refG.n; k++)
        if (!u32Has(&seq, refG.p[k]))
            u32Push(&unvisited, refG.p[k]);
    u32SortUniq(&unvisited);
    for (size_t k = 0; k < unvisited.n; k++) u32Push(&allG, unvisited.p[k]);
    for (size_t k = 0; k < seq.n; k++) u32Push(&allG, seq.p[k]);

    /* obj_offs_now */
    for (size_t k = 0; k < placedOff.n; k++) u32Push(&objOffs, placedOff.p[k]);
    for (uint32_t i = 0; i < (D >> 2) + 1; i++)
        if (mc.recNew[i] >= 0) u32Push(&objOffs, i << 2);
    for (size_t k = 0; k < mc.vtxOff.n; k++) u32Push(&objOffs, mc.vtxOff.p[k]);
    for (size_t k = 0; k < allG.n; k++) u32Push(&objOffs, allG.p[k]);
    u32SortUniq(&objOffs);
    for (size_t k = 0; k < mc.zeroVtx.n; k++) {
        uint32_t vo = mc.zeroVtx.p[k];
        uint32_t nxt = u32NextAbove(&objOffs, vo, D);
        int64_t sz = (int64_t)nxt - vo;
        if (sz <= 0 || (sz % 16)) {
            cvErr(c, pm->name, "zero-vtx array bad size", vo, (uint32_t)sz);
        } else {
            u32Push(&mc.vtxOff, vo); u32Push(&mc.vtxN, (uint32_t)(sz / 16));
            dstpos = addRegion(regions, vo, sz, dstpos);
        }
    }

    /* Embedded image blobs: G_SETTIMG seg-5 targets */
    for (size_t k = 0; k < allG.n; k++) {
        uint32_t o = allG.p[k];
        while ((uint64_t)o + 8 <= D) {
            uint32_t w0 = sBe32r(&mc.src, o), cc = w0 >> 24;
            if (cc == 0xB8)
                break;
            if (cc == 0xFD && (sBe32r(&mc.src, o + 4) >> 24) == 5)
                u32Push(&blobT, sBe32r(&mc.src, o + 4) & MASK24);
            o += 8;
        }
    }
    for (size_t k = 0; k < regions->n; k++) u32Push(&placedOffs, (uint32_t)regions->p[k].old);
    u32SortUniq(&placedOffs);
    u32SortUniq(&blobT);
    for (size_t k = 0; k < blobT.n; k++) {
        uint32_t t = blobT.p[k];
        uint32_t end = u32NextAbove(&placedOffs, t, D);
        if (mergedS.n && t <= mergedE.p[mergedS.n - 1]) {
            if (end > mergedE.p[mergedE.n - 1]) mergedE.p[mergedE.n - 1] = end;
        } else {
            u32Push(&mergedS, t); u32Push(&mergedE, end);
        }
    }
    for (size_t k = 0; k < mergedS.n; k++)
        dstpos = addRegion(regions, mergedS.p[k], (int64_t)mergedE.p[k] - mergedS.p[k], dstpos);

    /* GDLs last, tight-packed 16B slots */
    int totalSlots = 0;
    for (int pass = 0; pass < 2; pass++) {
        const U32V *list = pass == 0 ? &unvisited : &seq;
        for (size_t k = 0; k < list->n; k++) {
            uint32_t g = list->p[k];
            uint32_t end = gdlEnd(&mc, g);
            uint32_t nslots = (end - g) / 8;
            if ((end - g) % 8)
                cvErr(c, pm->name, "GDL end not 8B aligned", g, 0);
            if (g >= D || (g & 3)) { cvErr(c, pm->name, "GDL offset unaligned/outside", g, 0); continue; }
            mc.gdlNew[g >> 2] = dstpos;
            dstpos += 16 * (int64_t)nslots;
            totalSlots += (int)nslots;
            regPush(regions, g, 16 * (int64_t)nslots, mc.gdlNew[g >> 2]);
        }
    }
    (void)totalSlots;
    if (mc.src.oob) { cvErr(c, pm->name, "layout: read past end of file", 1, 0); goto done; }
    if (c->nerr > nerr0)
        goto done;

    uint32_t DPC = (uint32_t)dstpos;

    /* ---- remap machinery ---- */
    qsort(regions->p, regions->n, sizeof(Reg), cmpRegOld);

    /* ---- emit pass ---- */
    buf = (uint8_t *)calloc(DPC ? DPC : 1, 1);
    if (!buf) { cvErr(c, pm->name, "out of memory", DPC, 0); goto done; }
    e.m = &mc; e.buf = buf; e.dpc = DPC; e.is64 = is64; e.rs = regions->p; e.nrs = regions->n;
    Src *src = &mc.src;

    for (uint32_t i = 0; i < NS; i++)
        putPtr(&e, PW * i, sBe32o(src, 4 * i));
    for (uint32_t i = 0; i < NT; i++) {
        uint32_t doff = 4 * NS + 12 * i, npos = PW * NS + 12 * i;
        uint32_t tid = sBe32r(src, doff);
        putU32(&e, npos, tid);
        putBytes(&e, npos + 4, doff + 4, 8);
        if ((tid >> 24) == 5) {
            int ok;
            (void)eRemap(&e, tid & MASK24, &ok);
            if (!ok)
                cvErr(c, pm->name, "texconfig seg-5 target not in a blob region", tid & MASK24, 0);
        }
    }

    for (size_t k = 0; k < placedOff.n; k++) {
        uint32_t no = placedOff.p[k], op = placedOp.p[k];
        uint32_t npos = (uint32_t)mc.nodeNew[no >> 2];
        putU16(&e, npos, sBu16(src, no));
        putPtr(&e, npos + DO(is64, 8, 4), sBe32o(src, no + 4));
        putPtr(&e, npos + DO(is64, 16, 8), sBe32o(src, no + 8));
        putPtr(&e, npos + DO(is64, 24, 0xC), sBe32o(src, no + 0xC));
        putPtr(&e, npos + DO(is64, 32, 0x10), sBe32o(src, no + 0x10));
        putPtr(&e, npos + DO(is64, 40, 0x14), sBe32o(src, no + 0x14));

        uint32_t d = sBe32o(src, no + 4);
        int64_t r = mc.recNew[d >> 2];
        if (r < 0) {
            cvErr(c, pm->name, "no record slot for node", no, d);
            continue;
        }
        emitRecord(&e, op, (uint32_t)r, d);
    }

    /* vertex arrays */
    for (size_t k = 0; k < mc.vtxOff.n; k++) {
        uint32_t vo = mc.vtxOff.p[k];
        int32_t nv = (int32_t)mc.vtxN.p[k];
        int ok;
        int64_t npos = eRemap(&e, vo, &ok);
        if (!ok) {
            cvErr(c, pm->name, "vertex array not in region map", vo, 0);
            continue;
        }
        int64_t dummy;
        if (kvGet(&mc.op24Collision, vo, &dummy))
            emitVtxCollision(&e, vo, nv, (uint32_t)npos);
        else
            emitVtxMain(&e, vo, nv, (uint32_t)npos);
    }
    /* D120: PointUsage[] byteswap */
    for (size_t k = 0; k < mc.op24PointUsage.n; k++) {
        uint32_t puo = mc.op24PointUsage.p[k].k;
        int32_t punv = (int32_t)mc.op24PointUsage.p[k].v;
        int ok;
        int64_t pn = eRemap(&e, puo, &ok);
        if (!ok) {
            cvErr(c, pm->name, "PointUsage not in region map", puo, 0);
            continue;
        }
        for (int32_t q = 0; q < punv; q++) {
            uint32_t o2 = (uint32_t)pn + 2u * (uint32_t)q;
            if (eChk(&e, o2, 2) && sOk(src, puo + 2u * (uint32_t)q, 2)) {
                buf[o2] = src->p[puo + 2u * (uint32_t)q + 1];
                buf[o2 + 1] = src->p[puo + 2u * (uint32_t)q];
            }
        }
    }

    /* GDLs (pack order: unvisited first, then visit order), 16B LE slots */
    for (size_t k = 0; k < allG.n; k++) {
        uint32_t g = allG.p[k];
        uint32_t end = gdlEnd(&mc, g);
        int64_t npos = gdlNewOff(&mc, g);
        if (npos < 0) { cvErr(c, pm->name, "GDL without slot", g, 0); continue; }
        uint32_t j = 0, o = g;
        while ((uint64_t)o + 8 <= end) {
            uint32_t w0 = sBe32r(src, o), w1 = sBe32r(src, o + 4);
            uint32_t q = (uint32_t)npos + 16 * j;
            uint32_t cc = w0 >> 24;
            if ((cc == ADDR_OP_VTX || cc == ADDR_OP_SETTIMG) && (w1 >> 24) == 5) {
                int ok;
                int64_t n = eRemap(&e, w1 & MASK24, &ok);
                if (!ok) {
                    cvErr(c, pm->name, "GDL w1 not in map", g, w1);
                    n = 0;
                }
                w1 = 0x05000000u | ((uint32_t)n & MASK24);
            }
            if (eChk(&e, q, 16)) {
                putLe32(buf + q, w0);
                putLe32(buf + q + (is64 ? 8 : 4), w1);
            }
            if (cc == 0xB8)
                break;
            j++;
            o += 8;
        }
    }
    if (src->oob) { cvErr(c, pm->name, "emit: read past end of file", 2, 0); goto done; }
    if (c->nerr > nerr0)
        goto done;

    validateModel(&mc, &e, &placedOff, &placedOp, &allG);
    if (c->nerr > nerr0)
        goto done;

    /* ---- compress + append to sidecar ---- */
    if (cvDeflateRZ(buf, DPC, &comp, CV_LEVEL(c))) {
        cvErr(c, pm->name, "deflate failed", DPC, 0);
        goto done;
    }
    if (scEmit(sc, pm->name, comp.p, comp.len)) {
        cvErr(c, pm->name, "out of memory", 0, 0);
        goto done;
    }
    rc = 0;
done:
    free(buf);
    bufFree(&dec); bufFree(&comp);
    free(mc.nodes); free(mc.nodeNew); free(mc.recNew); free(mc.gdlNew);
    free(mc.regions.p);
    u32Free(&mc.vtxOff); u32Free(&mc.vtxN); u32Free(&mc.zeroVtx);
    kvFree(&mc.op24Collision); kvFree(&mc.op24PointUsage);
    u32Free(&seq); u32Free(&placedOff); u32Free(&placedOp); u32Free(&refG);
    u32Free(&unvisited); u32Free(&allG); u32Free(&objOffs); u32Free(&blobT);
    u32Free(&mergedS); u32Free(&mergedE); u32Free(&placedOffs);
    return rc;
}

int cvConvertModels(Ctx *c, const PcConvTables *t, Sidecar *sc)
{
    for (int i = 0; i < t->nModels; i++) {
        if (processModel(c, &t->models[i], sc))
            return -1;
    }
    /* pcmodels.bin is padded out to a 16-byte multiple */
    if (sc->off % 16) {
        if (bufAppendZero(&sc->bin, 16 - (sc->off % 16)))
            return -1;
        sc->off += 16 - (sc->off % 16);
    }
    return 0;
}
