import os

# lanuage type for each file extension
lang_types = {
    '.c'     : "C",
    '.h'     : "C",
    '.cc'    : "C++",
    '.hh'    : "C++",
    '.cxx'   : "C++",
    '.hxx'   : "C++",
    '.cpp'   : "C++",
    '.hpp'   : "C++",
    '.C'     : "C++",
    '.H'     : "C++",
    '.i'     : "swig",
    '.py'    : "python",
    '.pl'    : "perl",
    '.pm'    : "perl",
    '.s'     : "asm",
    '.S'     : "asm",
    '.l'     : "lex",
    '.ll'    : "lex",
    '.y'     : "yacc",
    '.yy'    : "yacc",
    '.isa'   : "isa",
    '.sh'    : "shell",
    '.slicc' : "slicc",
    '.sm'    : "slicc",
    '.awk'   : "awk",
    '.el'    : "lisp",
    '.txt'   : "text",
    '.tex'   : "tex",
    }

# languages based on file prefix
lang_prefixes = (
    ('SCons',    'scons'),
    ('Make',     'make'),
    ('make',     'make'),
    ('Doxyfile', 'doxygen'),
    )

# languages based on #! line of first file
hash_bang = (
    ('python', 'python'),
    ('perl',   'perl'),
    ('sh',     'shell'),
    )

# the list of all languages that we detect
all_languages = frozenset(lang_types.itervalues())
all_languages |= frozenset(lang for start,lang in lang_prefixes)
all_languages |= frozenset(lang for start,lang in hash_bang)

def lang_type(filename, firstline=None, openok=True):
    '''identify the language of a given filename and potentially the
    firstline of the file.  If the firstline of the file is not
    provided and openok is True, open the file and read the first line
    if necessary'''

    basename = os.path.basename(filename)
    name,extension = os.path.splitext(basename)

    # first try to detect language based on file extension
    try:
        return lang_types[extension]
    except KeyError:
        pass

    # now try to detect language based on file prefix
    for start,lang in lang_prefixes:
        if basename.startswith(start):
            return start

    # if a first line was not provided but the file is ok to open,
    # grab the first line of the file.
    if firstline is None and openok:
        handle = file(filename, 'r')
        firstline = handle.readline()
        handle.close()

    # try to detect language based on #! in first line
    if firstline and firstline.startswith('#!'):
        for string,lang in hash_bang:
            if firstline.find(string) > 0:
                return lang

    # sorry, we couldn't detect the language
    return None
