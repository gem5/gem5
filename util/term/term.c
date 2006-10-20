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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/un.h>

#include <netinet/in.h>
#include <arpa/telnet.h>

#include <err.h>
#include <errno.h>
#include <netdb.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

ssize_t	atomicio(ssize_t (*)(), int, void *, size_t);
void	readwrite(int);
int	remote_connect(char *, char *, struct addrinfo);

struct	termios saved_ios;
void	raw_term();
void	restore_term();

char	progname[256];
void	usage(int);

int
main(int argc, char *argv[])
{
    int ch, s, ret;
    char *host, *port, *endp;
    struct addrinfo hints;
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

    /* Initialize addrinfo structure */
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    s = remote_connect(host, port, hints);
    ret = 0;
    readwrite(s);

    if (s)
        close(s);

    exit(ret);
}

/*
 * remote_connect()
 * Return's a socket connected to a remote host. Properly bind's to a local
 * port or source address if needed. Return's -1 on failure.
 */
int
remote_connect(char *host, char *port, struct addrinfo hints)
{
    struct addrinfo *res, *res0;
    int s, error;

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
 * readwrite()
 * Loop that polls on the network file descriptor and stdin.
 */
void
readwrite(int nfd)
{
    struct pollfd pfd[2];
    char buf[BUFSIZ];
    int wfd = fileno(stdin), n, ret;
    int lfd = fileno(stdout);
    int escape = 0;

    /* Setup Network FD */
    pfd[0].fd = nfd;
    pfd[0].events = POLLIN;

    /* Setup STDIN FD */
    pfd[1].fd = wfd;
    pfd[1].events = POLLIN;

    while (pfd[0].fd != -1) {
        if ((n = poll(pfd, 2, -1)) < 0) {
            close(nfd);
            err(1, "Polling Error");
        }

        if (n == 0)
            return;

        if (pfd[0].revents & POLLIN) {
            if ((n = read(nfd, buf, sizeof(buf))) < 0)
                return;
            else if (n == 0) {
                shutdown(nfd, SHUT_RD);
                pfd[0].fd = -1;
                pfd[0].events = 0;
            } else {
                if ((ret = atomicio(write, lfd, buf, n)) != n)
                    return;
            }
        }

        if (pfd[1].revents & POLLIN) {
            if ((n = read(wfd, buf, sizeof(buf))) < 0)
                return;
            else if (n == 0) {
                shutdown(nfd, SHUT_WR);
                pfd[1].fd = -1;
                pfd[1].events = 0;
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
    }
}

void
usage(int ret)
{
    fprintf(stderr, "usage: %s hostname port\n", progname);
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
    ios.c_oflag &= ~(OPOST);
    ios.c_oflag &= (ONLCR);
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
