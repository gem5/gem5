#include <c_asm.h>

#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "m5op.h"

char *progname;

void
usage()
{
    char *name = basename(progname);
    printf("usage: %s ivlb <interval>\n"
           "       %s ivle <interval>\n"
           "       %s initparam\n"
           "       %s sw99param\n"
           "       %s resetstats\n"
           "       %s exit\n", name, name, name, name, name, name);
    exit(1);
}

int
main(int argc, char *argv[])
{
    int start;
    int interval;
    unsigned long long param;

    progname = argv[0];
    if (argc < 2)
        usage();

    if (strncmp(argv[1], "ivlb", 5) == 0) {
        if (argc != 3) usage();
        ivlb((unsigned long)atoi(argv[2]));
    } else if (strncmp(argv[1], "ivle", 5) == 0) {
        if (argc != 3) usage();
        ivle((unsigned long)atoi(argv[2]));
    } else if (strncmp(argv[1], "exit", 5) == 0) {
        if (argc != 2) usage();
        m5exit();
    } else if (strncmp(argv[1], "initparam", 10) == 0) {
        if (argc != 2) usage();
        printf("%d", initparam());
    } else if (strncmp(argv[1], "sw99param", 10) == 0) {
        if (argc != 2) usage();

        param = initparam();
        // run-time, rampup-time, rampdown-time, warmup-time, connections
        printf("%d %d %d %d %d", (param >> 48) & 0xfff,
               (param >> 36) & 0xfff, (param >> 24) & 0xfff,
               (param >> 12) & 0xfff, (param >> 0) & 0xfff);
    } else if (strncmp(argv[1], "resetstats", 11) == 0) {
        if (argc != 2) usage();
        resetstats();
    }

    return 0;
}
