import asyncio
import asyncio.core as _acore
import errno
import socket
import time
import _rosette

_EAGAIN      = errno.EAGAIN      if hasattr(errno, 'EAGAIN')      else 11
_EPERM       = errno.EPERM       if hasattr(errno, 'EPERM')       else 1
_ETIMEDOUT   = errno.ETIMEDOUT   if hasattr(errno, 'ETIMEDOUT')   else 110

_DISCOVERY_MCAST_IP   = "239.255.0.2"
_DISCOVERY_MCAST_PORT = 7400

def inet_aton(addr):
    ip_as_bytes = bytes(map(int, addr.split(".")))
    return ip_as_bytes

class Transport(_rosette.Transport):
    """Python-owned UDP transport for Rosette micro-ROS.

    Owns the socket. The C base class holds socket_ready / connected flags
    so the C++ CustomTransport adapter can check them without calling Python.
    All I/O goes through the Python methods below.

    Call open(agent_ip, agent_port) to create the socket and connect to the
    agent. If agent_ip is None, open() runs XRCE multicast discovery first.
    """

    def __init__(self):
        self.s: "socket.socket" = None
        self.remote = None
        self.connected = False
        self.agent_ip = None
        self.agent_port = 0
        super().__init__()

    def open(self, agent_ip=None, agent_port=8888, local_port=0):
        if self.s is not None:
            return
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if local_port:
            self.s.bind(socket.getaddrinfo("", local_port)[0][-1])
        self.s.setblocking(False)

        if agent_ip is None:
            agent_ip, agent_port = self._sync_discover_agent()

        self.connect(agent_ip, agent_port)
        self.agent_ip = agent_ip
        self.agent_port = agent_port

    def connect(self, addr, port):
        remote = socket.getaddrinfo(addr, port)[0][-1]
        self.remote = remote
        try:
            self.s.connect(remote)
            self.connected = True
        except OSError as e:
            if e.errno != _EPERM:
                raise
            self.connected = False

    def send(self, data):
        if self.connected:
            return self.s.send(data)
        return self.s.sendto(data, self.remote)

    def recv(self, size=2048):
        try:
            return self.s.recv(size)
        except OSError as e:
            if e.errno in (_EAGAIN, _ETIMEDOUT):
                return None
            raise

    def sendto(self, data, addr):
        return self.s.sendto(data, addr)

    def recvfrom(self, size=2048):
        return self.s.recvfrom(size)

    async def arecvfrom(self, size=2048):
        yield _acore._io_queue.queue_read(self.s)
        return self.s.recvfrom(size)

    def close(self):
        if self.s is not None:
            self.s.close()
        self.s = None

    # ------------------------------------------------------------------
    # Discovery helpers
    # ------------------------------------------------------------------

    def _sync_discover_agent(self, timeout_ms=3000, attempts=5,
                              retry_interval_ms=500):
        """Synchronous (blocking) XRCE multicast agent discovery.

        Used by open() when called outside an async context.
        Returns (ip_str, port_int).
        """
        return asyncio.run(self.discover_agent())

    async def discover_agent(self, timeout_ms=3000, attempts=5,  # type: ignore[override]
                             retry_interval_ms=500):
        """Async XRCE multicast agent discovery (MicroPython coroutine style).

        Use as: ``result = await transport.discover_agent()``.
        Suspends via queue_read without blocking the event loop.
        Returns (ip_str, port_int) on success, raises OSError on timeout.
        """
        mcast_addr = socket.getaddrinfo(_DISCOVERY_MCAST_IP,
                                        _DISCOVERY_MCAST_PORT)[0][-1]
        
        self.s.setsockopt(
            socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
            inet_aton(_DISCOVERY_MCAST_IP) + inet_aton("0.0.0.0"))
        probe = self._build_probe()

        for attempt in range(attempts):
            self.s.sendto(probe, mcast_addr)
            deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
            while True:
                remaining = time.ticks_diff(deadline, time.ticks_ms())
                if remaining <= 0:
                    break
                packet = await asyncio.wait_for(self.arecvfrom(), timeout=remaining)
                result = self._parse_reply(packet[0])
                if result is not None:
                    return result
            if attempt < attempts - 1:
                await asyncio.sleep_ms(retry_interval_ms)

        raise OSError("micro-ROS agent not found on network")

    def _build_probe(self):
        return _rosette.Transport._build_probe()

    def _parse_reply(self, packet):
        return _rosette.Transport._parse_reply(packet)
