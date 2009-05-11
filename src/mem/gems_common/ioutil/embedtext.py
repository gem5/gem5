
import sys

#---------------------------------------------------------------------------

class embedText:
    """
    embedText converts a text file into a file that can be embedded in C
    using an #include statement, that defines a \"const char *\" pointing
    to the same text.

    This is useful to embed scripts and configuration files in object files.
    """
    def __init__(self, filename):
        self.filename = filename
        self.escape        = [ "\'", "\"", "\\", "\?" ]

    def write(self, outputfile, varname):
        # reads the text file in, line by line, converting it to a C string
        fin = open( self.filename, 'r' )
        fout= open( outputfile, 'w' )
        fout.write("static const char *%s =\n" % varname);
        l = " "
        while l != "":
            l = fin.readline()

            # add escape sequences for the characters in escape
            fout.write("\"")
            for char in l:
                if char == "\n":
                    break
                if char in self.escape:
                    fout.write( "\\" )
                    fout.write( char )
                else:
                    fout.write( char )
            fout.write("\\n\"\n");
        fout.write(";\n");
        fin.close()
        fout.close()

#---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print len(sys.argv)
        print "usage:", sys.argv[0], " input-file output-file varname"
        sys.exit(1)
    inputfile = sys.argv[1]
    outputfile = sys.argv[2]
    varname   = sys.argv[3]
    print "generating embedded text file: %s from %s\n" % (outputfile, inputfile)
    inc = embedText( inputfile )
    inc.write( outputfile, varname )
