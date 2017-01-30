/***************** Read command line arguments ****************
 * Switch arguments: -Cxxx  /Cxxx  -C-  -C+
 * String arguments:  text1  text2
 * - Upper or lower case of switches makes no difference.
 * - Each function call gets next parameter of command line.
 * - Example:
 *    "program -A- -B123 text1 -a+ -B456 Text2"
 *    1st call: A = False, B = 124 Text = text1
 *    2nd call: A = True,  B = 456 Text = Text2
 *    this means after string retrieval "text1":
 *    all arguments before become invalid and the 1st argument after becomes valid
 *    (names with '-' at the beginning must be flagged with '--')
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "readargs.h"

int readargs(int argtyp,  char c, void *argp)
{
    static int nc = 0;
    int i, ai;
    long al;
    int argc = _argc;
    char **argv = _argv;
    for (i = nc; ++i < argc; )
        switch (argv[i][0]) {
            case '-':
            case '/':
                if (toupper(argv[i][1]) != toupper(c))
                    break;
                switch (argtyp) {
                    case ABOOL:
                        if (argv[i][2] == '-')
                            *((int *) argp) = 0;
                        else
                            *((int *) argp) = 1;
                        return (1);
                    case AINT:
                        if (sscanf(argv[i] + 2, "%d", &ai) != 1)
                            continue;
                        *((int *) argp) = ai;
                        return (1);
                    case AHEX:
                        if (sscanf(argv[i] + 2, "%x", &ai) != 1)
                            continue;
                        *((int *) argp) = ai;
                        return (1);
                    case ALONG:
                        if (sscanf(argv[i] + 2, "%ld", &al ) != 1)
                            continue;
                        *((long *) argp) = al;
                        return (1);
                    case ASTRING:
                        strcpy(argp, argv[i] + 2);
                        return (1);
                    case APOINTER:
                        argp = argv[i] + 2;
                        return (1);
                    default:
                        continue;
                }
            default:
                if (argtyp == ANAME && i > nc) {
                    nc = i;
                    strcpy(argp, argv[nc]);
                    return (1);
                }
        }
    return (0);
}
