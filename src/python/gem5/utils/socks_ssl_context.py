import os
import ssl
from typing import Optional

_gem5_ssl_context = None


def get_proxy_context() -> Optional[ssl.SSLContext]:
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
