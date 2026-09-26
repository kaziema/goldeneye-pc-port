/*
 * romconvert.c — first-run sidecar conversion ("drop in your ROM and play").
 *
 * The engine needs two ROM-derived sidecar trees that are NOT shipped with a
 * release (they are copyrighted game data derived from the user's own ROM):
 *
 *     $S/pcmodels-<region>/   (pcmodels.bin + manifest.csv)  — port/src/pcmodels.c
 *     $S/pccg-<region>/       (pccg.bin    + manifest.csv)  — port/src/pccg.c
 *
 * The release bundle ships a frozen copy of the asset-prep tool
 * (prepare-assets.py, PyInstaller --onefile) as
 * <exedir>/prepare-assets/ge007-convert[.exe]. When the sidecars are missing
 * at boot, romdataInit() calls romConvertEnsureSidecars(), which spawns that
 * binary with `--rom <this ROM> --out <exedir>`, waits for it to finish, and
 * re-checks. The conversion is deterministic and takes a few seconds; after
 * the first run the sidecars exist and this path is a no-op (two fsExists).
 *
 * Vita: no external binary; the in-process converter (pcconv*.c) builds the
 * 32-bit-layout sidecars from the ROM on first launch.
 *
 * Returns 1 when both sidecars are present (already or after converting),
 * -1 when they could not be produced (caller should abort boot with a
 * message rather than crash later in modelPromoteNodeOffsetsToPointers, D179).
 */

#include <stdlib.h>
#include <string.h>

/* D38: <stdio.h>/<stdlib.h> resolve to the decomp's N64 stubs via the include
 * path; declare the host functions this file uses. */
extern void *malloc(unsigned long long size);
extern int snprintf(char *str, size_t maxsize, const char *format, ...);

#include "platform.h"

#if defined(PLATFORM_WINDOWS)
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#elif defined(__vita__)
  #include <psp2/io/stat.h>
  #include <psp2/io/fcntl.h>
  #include "pcconv.h"
#else
  #include <unistd.h>
  #include <sys/wait.h>
#endif

#include "system.h"
#include "fs.h"

/* Same country→region map as pcmodels.c / pccg.c (finding: plan Q3). */
static const char *rcRegionForCountry(unsigned char country)
{
    switch (country) {
    case 'E': return "ntsc-final";
    case 'P': return "pal-final";
    case 'J': return "jpn-final";
    default:  return NULL;
    }
}

/* sysResolvePath() returns a static buffer — copy before the next call. */
static int rcExistsResolved(const char *rel)
{
    const char *p = sysResolvePath(rel);
    return fsExists(p);
}

#if defined(__vita__)
static int rcVitaStampOk(void);
#endif

static int rcSidecarsPresent(const char *region)
{
    char rel[256];
    snprintf(rel, sizeof(rel), "$S/pcmodels-%s/pcmodels.bin", region);
    if (!rcExistsResolved(rel))
        return 0;
    snprintf(rel, sizeof(rel), "$S/pccg-%s/pccg.bin", region);
    if (!rcExistsResolved(rel))
        return 0;
#if defined(__vita__)
    return rcVitaStampOk();
#else
    return 1;
#endif
}

#if defined(PLATFORM_WINDOWS)
/* Spawn `exePath --rom "<rom>" --out "<exedir>"` and wait for it.
 * Returns the exit code, or -1 if it could not be started. */
static int rcRunConverter(const char *exePath, const char *rom, const char *out)
{
    char cmd[4096];
    int n = snprintf(cmd, sizeof(cmd), "\"%s\" --rom \"%s\" --out \"%s\"",
                     exePath, rom, out);
    if (n < 0 || (size_t)n >= sizeof(cmd))
        return -1;   /* paths too long to build a command line */

    int wlen = MultiByteToWideChar(CP_UTF8, 0, cmd, -1, NULL, 0);
    if (wlen <= 0)
        return -1;
    WCHAR *wcmd = (WCHAR *)malloc((size_t)wlen * sizeof(WCHAR));
    if (!wcmd)
        return -1;
    MultiByteToWideChar(CP_UTF8, 0, cmd, -1, wcmd, wlen);

    STARTUPINFOW si;
    PROCESS_INFORMATION pi;
    int rc = -1;
    memset(&si, 0, sizeof(si));
    si.cb = sizeof(si);
    memset(&pi, 0, sizeof(pi));
    if (CreateProcessW(NULL, wcmd, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi)) {
        WaitForSingleObject(pi.hProcess, INFINITE);
        DWORD code = 0;
        GetExitCodeProcess(pi.hProcess, &code);
        rc = (int)code;
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
    }
    free(wcmd);
    return rc;
}
#elif defined(__vita__)
/* Bump when the converter output format changes; stale sidecars are rebuilt. */
#define RC_VITA_STAMP "1\n"

static int rcVitaStampOk(void)
{
    char buf[8] = "";
    FSFile *f = fsOpen(sysResolvePath("$S/pcconv.ver"), "rb");
    if (!f)
        return 0;
    int n = fsRead(f, buf, sizeof(buf) - 1);
    fsClose(f);
    return n > 0 && !strncmp(buf, RC_VITA_STAMP, sizeof(RC_VITA_STAMP) - 1);
}

/* Write via a temp name so an interrupted run never leaves a partial file. */
static int rcVitaWrite(const char *relDir, const char *name, const void *data, size_t size)
{
    char dir[256], fin[300], tmp[300];
    snprintf(dir, sizeof(dir), "%s", sysResolvePath(relDir));
    size_t dl = strlen(dir);
    if (dl && dir[dl - 1] == '/')
        dir[dl - 1] = 0;
    sceIoMkdir(dir, 0777);
    snprintf(fin, sizeof(fin), "%s/%s", dir, name);
    snprintf(tmp, sizeof(tmp), "%s/%s.tmp", dir, name);

    FSFile *f = fsOpen(tmp, "wb");
    if (!f) {
        sysLogPrintf(LOG_ERROR, "romconvert: cannot create %s", tmp);
        return -1;
    }
    const unsigned char *p = (const unsigned char *)data;
    size_t left = size;
    while (left) {
        int chunk = left > (1u << 20) ? (1 << 20) : (int)left;
        if (fsWrite(f, p, chunk) != chunk) {
            sysLogPrintf(LOG_ERROR, "romconvert: write failed for %s", tmp);
            fsClose(f);
            return -1;
        }
        p += chunk;
        left -= (size_t)chunk;
    }
    fsClose(f);
    sceIoRemove(fin);
    if (sceIoRename(tmp, fin) < 0) {
        sysLogPrintf(LOG_ERROR, "romconvert: cannot rename to %s", fin);
        return -1;
    }
    return 0;
}

static int rcVitaConvert(const unsigned char *rom, unsigned int romSize, const char *region)
{
    PcConvResult r;
    char err[1024], dm[64], dc[64];

    sysLogPrintf(LOG_INFO, "romconvert: first launch, converting ROM data (this can take a minute)");
    if (pcconvRun(rom, romSize, &kPcConvTables, &kPcConvLayout32, &r, err, sizeof(err))) {
        sysLogPrintf(LOG_ERROR, "romconvert: conversion failed:\n%s", err);
        return -1;
    }
    snprintf(dm, sizeof(dm), "$S/pcmodels-%s", region);
    snprintf(dc, sizeof(dc), "$S/pccg-%s", region);
    /* .bin last in each dir: rcSidecarsPresent() keys off it. */
    int bad = rcVitaWrite(dm, "manifest.csv", r.modelsCsv.data, r.modelsCsv.size) ||
              rcVitaWrite(dm, "pcmodels.bin", r.modelsBin.data, r.modelsBin.size) ||
              rcVitaWrite(dc, "manifest.csv", r.cgCsv.data, r.cgCsv.size) ||
              rcVitaWrite(dc, "pccg.bin", r.cgBin.data, r.cgBin.size) ||
              rcVitaWrite("$S/", "pcconv.ver", RC_VITA_STAMP, sizeof(RC_VITA_STAMP) - 1);
    pcconvFree(&r);
    if (bad)
        sysLogPrintf(LOG_ERROR, "romconvert: could not write converted data (memory card full?)");
    return bad ? -1 : 0;
}
#else
static int rcRunConverter(const char *exePath, const char *rom, const char *out)
{
    pid_t pid = fork();
    if (pid < 0)
        return -1;
    if (pid == 0) {
        /* Child: stdio is inherited (console apps see the converter's
         * progress lines; GUI builds simply discard them). */
        execv(exePath, (char *const[]){ exePath, "--rom", (char *)rom,
                                        "--out", (char *)out, NULL });
        _exit(127);   /* execv failed */
    }
    int status = 0;
    while (waitpid(pid, &status, 0) < 0)
        ;   /* EINTR */
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    return -1;
}
#endif

#if defined(__vita__)
int romConvertEnsureSidecars(const unsigned char *romImg, unsigned int romSize,
                             const char *romRelPath)
{
    (void)romRelPath;
    const char *region = rcRegionForCountry(romImg[0x3E]);
    if (!region)
        return 1;
    if (rcSidecarsPresent(region))
        return 1;
    /* The converter tables are built from the US ROM's file list. */
    if (strcmp(region, "ntsc-final")) {
        sysLogPrintf(LOG_ERROR, "romconvert: only the US ROM is supported on Vita");
        return -1;
    }
    if (rcVitaConvert(romImg, romSize, region))
        return -1;
    return rcSidecarsPresent(region) ? 1 : -1;
}
#else
int romConvertEnsureSidecars(const unsigned char *romImg, unsigned int romSize,
                             const char *romRelPath)
{
    (void)romSize;
    const char *region = rcRegionForCountry(romImg[0x3E]);
    if (!region)
        return 1;   /* unknown country — romHeaderValid() rejects it anyway */

    if (rcSidecarsPresent(region))
        return 1;

    /* Resolve the ROM path we were given ("$S/...", "$E/..." or "./...") to
     * an absolute form for the child's argv. */
    char romPath[1024];
    snprintf(romPath, sizeof(romPath), "%s", sysResolvePath(romRelPath));

    const char *exedir = sysGetExeDir();
    char outDir[1024];
    snprintf(outDir, sizeof(outDir), "%s", exedir);

    /* Converter candidates: bundled under <exedir>/prepare-assets/ (the
     * release layout), then the CWD-relative form (running from a source
     * tree or an unpacked bundle in the CWD). */
#if defined(PLATFORM_WINDOWS)
    static const char *candRel[4] = {
        "$E/prepare-assets/ge007-convert.exe",
        "./prepare-assets/ge007-convert.exe",
        "$E/ge007-convert.exe",
        "./ge007-convert.exe",
    };
#else
    static const char *candRel[4] = {
        "$E/prepare-assets/ge007-convert",
        "./prepare-assets/ge007-convert",
        "$E/ge007-convert",
        "./ge007-convert",
    };
#endif
    char convPath[1024] = "";
    for (int i = 0; i < 4; i++) {
        if (rcExistsResolved(candRel[i])) {
            /* sysResolvePath() returns a static buffer — snapshot it. */
            snprintf(convPath, sizeof(convPath), "%s", sysResolvePath(candRel[i]));
            break;
        }
    }

    if (!convPath[0]) {
        sysLogPrintf(LOG_ERROR,
            "romconvert: sidecars for '%s' are missing and no ge007-convert "
            "binary was found (looked in <exedir>/prepare-assets/). Put your "
            "ROM at data/ge007.%s.z64 and re-run from the bundle folder, or "
            "run prepare-assets manually.", region, region);
        return -1;
    }

    sysLogPrintf(LOG_INFO,
        "romconvert: sidecars missing — running %s (one-time, a few seconds)…",
        convPath);
    int code = rcRunConverter(convPath, romPath, outDir);
    if (code != 0)
        sysLogPrintf(LOG_ERROR,
            "romconvert: ge007-convert exited with code %d — see its output "
            "above (a non-retail or byte-swapped ROM will fail here)", code);

    if (!rcSidecarsPresent(region)) {
        sysLogPrintf(LOG_ERROR,
            "romconvert: data/pcmodels-%s/ and data/pccg-%s/ are still "
            "missing — cannot continue", region, region);
        return -1;
    }
    sysLogPrintf(LOG_INFO, "romconvert: sidecars generated — continuing boot");
    return 1;
}
#endif /* __vita__ */
