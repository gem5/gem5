import os

# lanuage type for each file extension
lang_types = {
    '.c'     : "C",
    '.cl'    : "C",
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
    '.mk'    : "make",
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
            return lang

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

# directories and files to ignore by default
default_dir_ignore = frozenset(('.hg', '.svn', 'build', 'ext'))
default_file_ignore = frozenset(('parsetab.py', ))

def find_files(base, languages=all_languages,
               dir_ignore=default_dir_ignore,
               file_ignore=default_file_ignore):
    '''find all files in a directory and its subdirectories based on a
    set of languages, ignore directories specified in dir_ignore and
    files specified in file_ignore'''
    if base[-1] != '/':
        base += '/'

    def update_dirs(dirs):
        '''strip the ignored directories out of the provided list'''
        index = len(dirs) - 1
        for i,d in enumerate(reversed(dirs)):
            if d in dir_ignore:
                del dirs[index - i]

    # walk over base
    for root,dirs,files in os.walk(base):
        root = root.replace(base, '', 1)

        # strip ignored directories from the list
        update_dirs(dirs)

        for filename in files:
            if filename in file_ignore:
                # skip ignored files
                continue

            # try to figure out the language of the specified file
            fullpath = os.path.join(base, root, filename)
            language = lang_type(fullpath)

            # if the file is one of the langauges that we want return
            # its name and the language
            if language in languages:
                yield fullpath, language

def update_file(dst, src, language, mutator):
    '''update a file of the specified language with the provided
    mutator generator.  If inplace is provided, update the file in
    place and return the handle to the updated file.  If inplace is
    false, write the updated file to cStringIO'''

    # if the source and destination are the same, we're updating in place
    inplace = dst == src

    if isinstance(src, str):
        # if a filename was provided, open the file
        if inplace:
            mode = 'r+'
        else:
            mode = 'r'
        src = file(src, mode)

    orig_lines = []

    # grab all of the lines of the file and strip them of their line ending
    old_lines = list(line.rstrip('\r\n') for line in src.xreadlines())
    new_lines = list(mutator(old_lines, src.name, language))

    for line in src.xreadlines():
        line = line

    if inplace:
        # if we're updating in place and the file hasn't changed, do nothing
        if old_lines == new_lines:
            return

        # otherwise, truncate the file and seek to the beginning.
        dst = src
        dst.truncate(0)
        dst.seek(0)
    elif isinstance(dst, str):
        # if we're not updating in place and a destination file name
        # was provided, create a file object
        dst = file(dst, 'w')

    for line in new_lines:
        dst.write(line)
        dst.write('\n')
