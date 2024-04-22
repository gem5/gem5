/* $OpenBSD: netcat.c,v 1.57 2002/12/30 18:00:18 stevesk Exp $ */
/*
 * Copyright (c) 2001 Eric Jackson <ericj@monkey.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arpa/telnet.h>
#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#ifdef __APPLE__
    #include <sys/syslimits.h>
#else
    #include <linux/limits.h>
#endif
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

ssize_t atomicio(ssize_t (*)(), int, void *, size_t);
void    readwrite(int);
int     remote_connect_inet(char *, char *);
int     remote_connect_unix(const char *);

struct  termios saved_ios;
void    raw_term();
void    restore_term();

char    progname[256];
void    usage(int);

int
main(int argc, char *argv[])
{
    int ch, s, ret;
    char *host, *port, *endp;
    socklen_t len;

    ret = 1;
    s = 0;
    host = NULL;
    port = NULL;
    endp = NULL;

    strncpy(progname, argv[0], sizeof progname);

    /* Cruft to make sure options are clean, and used properly. */
    if (argc == 2) {
        host = "localhost";
        port = argv[1];
    } else if (argc == 3) {
        host = argv[1];
        port = argv[2];
    } else {
        usage(1);
    }

    if (!isatty(STDIN_FILENO))
        errx(1, "not attached to a terminal");

    raw_term();

    if (strcmp(host, "--unix") == 0) {
        s = remote_connect_unix(port);
    } else {
        s = remote_connect_inet(host, port);
    }

    if (s != -1) {
        readwrite(s);
        close(s);
    }

    exit(0);
}

/*
 * remote_connect_inet()
 * Return's a socket connected to a remote host. Properly bind's to a local
 * port or source address if needed. Return's -1 on failure.
 */
int
remote_connect_inet(char *host, char *port)
{
    struct addrinfo hints;
    struct addrinfo *res, *res0;
    int s, error;

    /* Initialize addrinfo structure */
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    if ((error = getaddrinfo(host, port, &hints, &res)))
        errx(1, "getaddrinfo: %s", gai_strerror(error));

    res0 = res;
    do {
        if ((s = socket(res0->ai_family, res0->ai_socktype,
                        res0->ai_protocol)) < 0)
            continue;

        if (connect(s, res0->ai_addr, res0->ai_addrlen) == 0)
            break;

        close(s);
        s = -1;
    } while ((res0 = res0->ai_next) != NULL);

    freeaddrinfo(res);

    return (s);
}

/*
 * remote_connect_inet()
 * Return's a socket connected to a remote host. Properly bind's to a local
 * port or source address if needed. Return's -1 on failure.
 */
int
remote_connect_unix(const char *cpath)
{
    struct sockaddr_un addr;

    // Create a copy of path so we can safely modify it in place.
    char *path = strdup(cpath);
    char *const path_buf = path;

    // Create a unix domain socket.
    int s = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s == -1)
        return s;

    // Prepare the scokaddr_un.
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;

    // Keep track of where we're filling in the path, and the remaining space.
    int path_size = sizeof(addr.sun_path);
    char *sun_path = &addr.sun_path[0];

    // Keep track of the current directory in case we change it to maximize
    // what we can fit in the limited space in sun_path.
    char *cwd = NULL;

    if (path[0] == '@') {
        // If this is an abstract socket, prefix it with a null byte.
        *sun_path++ = '\0';
        path++;
        path_size--;
        // Keep track of how much of sun_path is actual data since everything
        // we include will be part of the lookup.
        int len = strlen(path);
        if (len < path_size) {
            fprintf(stderr,
                "warning: Truncated abstract socket from %d to %d bytes.\n",
                len, path_size);
            path_size = len;
        }
    } else {
        // Switch to the parent directory of the socket.
        cwd = (char *)malloc(PATH_MAX);
        if (!cwd)
            errx(1, "Failed to allocate %d byte buffer.", PATH_MAX);
        if (!getcwd(cwd, PATH_MAX)) {
            perror("getcwd failed");
            exit(1);
        }
        char *dirc = strdup(path);
        if (!dirc) {
            perror("strdup failed");
            exit(1);
        }
        char *dname = dirname(dirc);
        if (chdir(dname) != 0) {
            perror("chdir to socket dir failed");
            exit(1);
        }
        free(dirc);

        // Replace the path with just the filename part. We still have a
        // pointer to the cpath argument, so we can clean it up later.
        path = basename(path);
    }

    // Copy the path into sun_path.
    strncpy(sun_path, path, path_size);

    // Figure out how much actual data we have in sockaddr_un.
    int struct_len = (char *)sun_path + path_size - (char *)&addr;

    // Actually connect to the socket.
    if (connect(s, (struct sockaddr *)&addr, struct_len) == -1) {
        // If that didn't work, switch our dir back and error out.
        if (cwd)
            chdir(cwd);
        errx(1, "Failed to connect");
    }

    // We're connected, clean up memory and switch the current dir back.
    free(path_buf);
    if (cwd) {
        if (chdir(cwd) != 0) {
            perror("chdir back failed:");
            exit(1);
        }
        free(cwd);
    }

    // Return the FD of our new connection.
    return s;
}

/*
 * readwrite()
 * Loop that selects on the network file descriptor and stdin.
 * Changed from poll() by Ali Saidi to make work on Mac OS X >= 10.4
 */
void
readwrite(int nfd)
{
    fd_set read_fds;
    char buf[BUFSIZ];
    int wfd = fileno(stdin), n, ret, max_fd;
    int lfd = fileno(stdout);
    int escape = 0;
    struct timeval timeout;

    if (nfd == -1)
        return;

    max_fd = nfd + 1;

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(wfd, &read_fds);
        FD_SET(nfd, &read_fds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        n = select(max_fd, &read_fds, NULL, NULL, &timeout);
        if (n < 0) {
            close(nfd);
            perror("Select Error");
            exit(1);
        }

        if (n == 0) {
            if (read(nfd, buf, 0) < 0)
                return;
            continue;
        }

        if (read(nfd, buf, 0) < 0)
            return;

        if (FD_ISSET(nfd, &read_fds)) {
            if ((n = read(nfd, buf, sizeof(buf))) < 0)
                return;
            else if (n == 0) {
                shutdown(nfd, SHUT_RD);
                return;
            } else {
                if ((ret = atomicio(write, lfd, buf, n)) != n)
                    return;
            }
        }

        if (FD_ISSET(wfd, &read_fds)) {
            if ((n = read(wfd, buf, sizeof(buf))) < 0)
                return;
            else if (n == 0) {
                shutdown(nfd, SHUT_WR);
            } else {
                if (escape) {
                    char buf2[] = "~";
                    if (*buf == '.') {
                        printf("quit!\n");
                        return;
                    }
                    escape = 0;
                    if (*buf != '~' &&
                        (ret = atomicio(write, nfd, buf2, 1)) != n)
                        return;
                } else {
                    escape = (*buf == '~');
                    if (escape)
                        continue;
                }

                if ((ret = atomicio(write, nfd, buf, n)) != n)
                    return;
            }
        }
    } // while
}

void
usage(int ret)
{
    fprintf(stderr, "usage: %s [hostname] port\n", progname);
    fprintf(stderr, "usage: %s --unix socket\n", progname);
    if (ret)
        exit(1);
}

/*
 * Copyright (c) 1995,1999 Theo de Raadt.  All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * ensure all of data on socket comes through. f==read || f==write
 */
ssize_t
atomicio(ssize_t (*f) (), int fd, void *_s, size_t n)
{
    char *s = _s;
    ssize_t res, pos = 0;

    while (n > pos) {
        res = (f) (fd, s + pos, n - pos);
        switch (res) {
          case -1:
            if (errno == EINTR || errno == EAGAIN)
                continue;
          case 0:
            return (res);
          default:
            pos += res;
        }
    }
    return (pos);
}

/*
 * Copyright (c) 2003 Nathan L. Binkert <binkertn@umich.edu>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

void
raw_term()
{
    struct termios ios;

    if (tcgetattr(STDIN_FILENO, &ios) < 0)
        errx(1, "tcgetagttr\n");

    memcpy(&saved_ios, &ios, sizeof(struct termios));

    ios.c_iflag &= ~(ISTRIP|ICRNL|IGNCR|ICRNL|IXOFF|IXON);
    ios.c_oflag |= OPOST;
    ios.c_oflag |= ONLCR;
    ios.c_lflag &= ~(ISIG|ICANON|ECHO);
    ios.c_cc[VMIN] = 1;
    ios.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &ios) < 0)
        errx(1, "tcsetattr\n");

    atexit(restore_term);
}

void
restore_term()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_ios);
}
