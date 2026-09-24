#ifndef PORT_PCCONV_H
#define PORT_PCCONV_H

/*
 * In-process port of tools_pc/d43_emit.py, d69_emit.py, d88_emit.py and
 * d88_propdefs.py: builds the pcmodels-<region>/ and pccg-<region>/ sidecars
 * from a ROM image held in memory. Same output as the Python tools for the
 * x86-64 layout (PCCONV_LAYOUT_PC64); PCCONV_LAYOUT_32 emits the equivalent
 * for a 32-bit little-endian target (Vita), where records keep their N64
 * sizes and only display-list slots widen.
 *
 * Pure memory-in/memory-out: no file I/O, no game headers.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PcConvLayout {
    int is64;   /* 1: x86-64 PC layout (8-byte pointers); 0: 32-bit */
} PcConvLayout;

extern const PcConvLayout kPcConvLayoutPC64;
extern const PcConvLayout kPcConvLayout32;

/* One model file (file_resource_table order). ns/nt from its
 * MODELFILEHEADER; addr/size = its row in scripts/filelist.u.csv. */
typedef struct PcConvModel {
    const char *name;
    uint32_t addr, size;
    uint16_t ns, nt;
} PcConvModel;

/* One bg .seg or Tbg_ stan file (file_resource_table order). */
typedef struct PcConvCg {
    const char *name;   /* manifest name, e.g. "bg/bg_sev_all_p.seg" */
    uint32_t addr, size;
    int isStan;         /* 0: bg .seg (raw), 1: Tbg_ file (RZ-compressed) */
} PcConvCg;

/* One Usetup*Z stage-setup file (sorted by name). */
typedef struct PcConvSetup {
    const char *name;
    uint32_t addr, size;
} PcConvSetup;

typedef struct PcConvTables {
    const PcConvModel *models; int nModels;
    const PcConvCg *cg;        int nCg;
    const PcConvSetup *setup;  int nSetup;
} PcConvTables;

/* Generated from the repo's ROM-independent tables (pcconv_tables.c). */
extern const PcConvTables kPcConvTables;

typedef struct PcConvBlob {
    uint8_t *data;
    size_t size;
} PcConvBlob;

typedef struct PcConvResult {
    PcConvBlob modelsBin, modelsCsv;   /* pcmodels.bin / manifest.csv */
    PcConvBlob cgBin, cgCsv;           /* pccg.bin / manifest.csv */
} PcConvResult;

/* Convert. Returns 0 on success (result blobs filled, free with
 * pcconvFree), -1 on failure (err holds the first messages). */
int pcconvRun(const uint8_t *rom, uint32_t romSize, const PcConvTables *t,
              const PcConvLayout *layout, PcConvResult *out,
              char *err, size_t errSize);

void pcconvFree(PcConvResult *r);

/* propDef file-record size in 32-bit words (0 = unknown type). */
uint32_t pcconvPropdefWords(uint32_t type);

#ifdef __cplusplus
}
#endif

#endif /* PORT_PCCONV_H */
