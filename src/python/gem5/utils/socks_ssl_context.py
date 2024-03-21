# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import ssl
from typing import Optional

_gem5_ssl_context = None


def get_proxy_context() -> Optional[ssl.SSLContext]:
    """
    This function returns a SSL context for https connection with SOCKS proxy
    support. It uses the environment variable GEM5_USE_PROXY to determine
    the proxy server to use. If the environment variable is not set, it
    returns None.
    """

    global _gem5_ssl_context
    use_proxy = os.getenv("GEM5_USE_PROXY")
    if use_proxy and not _gem5_ssl_context:
        import socket

        import socks

        ip_addr, host_port = use_proxy.split(":")
        port = int(host_port)
        socks.set_default_proxy(socks.SOCKS5, ip_addr, port)
        socket.socket = socks.socksocket

        # base SSL context for https connection
        _gem5_ssl_context = ssl.create_default_context()
        _gem5_ssl_context.check_hostname = False
        _gem5_ssl_context.verify_mode = ssl.CERT_NONE
    return _gem5_ssl_context
