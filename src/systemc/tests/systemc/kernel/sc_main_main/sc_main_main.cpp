#include "systemc.h"

int main()
{
    char *argv[] = { strdup("0"), strdup("1"), strdup("2"), strdup("3"),
                     strdup("4"), NULL };
    int argc = sizeof argv / sizeof argv[0] - 1;
    sc_elab_and_sim( argc, argv );
    for (int i = 0; i < argc; ++i) {
        free(argv[i]);
    }
}

int sc_main(int argc, char* argv[])
{
    // Number of arguments should be the same
    sc_assert(argc == sc_argc());

    // Ensure all arguments are the same as sc_argv
    for ( int argi = 0; argi < argc; argi++ ) {
        if ( strcmp( argv[argi], sc_argv()[argi] ) != 0 ) {
            cout << "sc_argv()[" << argi << "] mismatch: expected: '"
                 << argv[argi] << "' got: '" << sc_argv()[argi] << "'" << endl;
        }
    }

    // This check will most likely not do anything since we are likely to have
    // zeros on the stack, but let's add it anyway.
    sc_assert(argv[argc] == NULL);
    sc_assert(sc_argv()[argc] == NULL);

    // Ensure that modifying argv does not alter sc_argv
    argv[1][0] = '9';
    free(argv[2]);
    argv[2] = strdup("new-2");
    sc_assert(strcmp(sc_argv()[2], "2") == 0);
    sc_assert(strcmp(sc_argv()[1], "1") == 0);

    cout << "Program completed" << endl;

    return 0;
}
