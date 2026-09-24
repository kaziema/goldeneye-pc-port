/* Host harness for the in-game sidecar converter.
 *   cc -O2 -DPCCONV_HOST -Iport/include -Iport/src tools_pc/pcconv_host.c \
 *      port/src/pcconv_*.c -lz -o pcconv_host
 *   ./pcconv_host <rom.z64> <outdir> [pc64|32]
 * Writes <outdir>/pcmodels/{pcmodels.bin,manifest.csv} and pccg/... . */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "pcconv.h"

static int writeFile(const char *dir, const char *sub, const char *name, const PcConvBlob *b)
{
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, sub);
    mkdir(dir, 0777);
    mkdir(path, 0777);
    snprintf(path, sizeof(path), "%s/%s/%s", dir, sub, name);
    FILE *f = fopen(path, "wb");
    if (!f || fwrite(b->data, 1, b->size, f) != b->size) {
        perror(path);
        return -1;
    }
    fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: %s rom.z64 outdir [pc64|32]\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *rom = (uint8_t *)malloc((size_t)n);
    if (!rom || fread(rom, 1, (size_t)n, f) != (size_t)n) { fprintf(stderr, "read failed\n"); return 1; }
    fclose(f);

    const PcConvLayout *L = (argc > 3 && !strcmp(argv[3], "32")) ? &kPcConvLayout32 : &kPcConvLayoutPC64;
    PcConvResult r;
    char err[4096];
    if (pcconvRun(rom, (uint32_t)n, &kPcConvTables, L, &r, err, sizeof(err))) {
        fprintf(stderr, "convert failed:\n%s", err);
        return 1;
    }
    int rc = writeFile(argv[2], "pcmodels", "pcmodels.bin", &r.modelsBin) ||
             writeFile(argv[2], "pcmodels", "manifest.csv", &r.modelsCsv) ||
             writeFile(argv[2], "pccg", "pccg.bin", &r.cgBin) ||
             writeFile(argv[2], "pccg", "manifest.csv", &r.cgCsv);
    printf("models.bin %zu, cg.bin %zu\n", r.modelsBin.size, r.cgBin.size);
    pcconvFree(&r);
    return rc ? 1 : 0;
}
