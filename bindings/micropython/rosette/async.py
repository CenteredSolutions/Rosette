"""rosette.async — asyncio wrappers for Rosette pub/sub.

Import pattern::

    import rosette
    from rosette.async import AsyncContext, AsyncNode, AsyncPublisher, AsyncSubscription

The underlying C objects expose ``ioctl(MP_STREAM_POLL, ...)`` so they plug
directly into MicroPython's asyncio I/O queue.  The event loop suspends the
calling coroutine and resumes it as soon as the object reports ready.

A background pump drives the micro-ROS executor so hardware-interrupt-owned
ring buffers stay fresh.  ``AsyncContext`` starts and cancels the pump
automatically when used as an async context manager.

Typical usage with automatic agent discovery::

    async with AsyncContext(client_key=0x01020304, domain_id=0,
                            executor_handles=8, spin_timeout_ms=1) as ctx:
        ip, port = await ctx.discover_agent()
        await ctx.start()
        node = ctx.create_node("my_node", "/ns")
        schema = rosette.schema("sensor_msgs/Temperature")
        pub = node.create_publisher(schema, "temp")
        sub = node.create_subscription(schema, "temp")
        await pub.publish(schema(temperature=20.0, variance=0.0))
        msg = await sub.recv()

Or with a known agent address (no discovery)::

    async with AsyncContext(agent_ip="10.0.0.1", agent_port=8888,
                            client_key=0x01020304, domain_id=0,
                            executor_handles=8, spin_timeout_ms=1) as ctx:
        await ctx.start()
        node = ctx.create_node("my_node", "/ns")
        ...
"""

import asyncio
import rosette as _rosette

# uasyncio exposes its I/O queue as asyncio.core._io_queue (MicroPython ≥1.20).
# queue_read(obj) and queue_write(obj) register the object for the next poll
# pass and yield from the current task.
_ioq = asyncio.core._io_queue  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Pump
# ---------------------------------------------------------------------------

async def _tick_pump(ctx_raw, interval_ms):
    """Drive the micro-ROS executor in the background.

    Calls ``ctx.tick()`` on every iteration so the XRCE session stays alive
    and subscription callbacks can enqueue received frames.
    """
    while ctx_raw.started():
        ctx_raw.tick()
        await asyncio.sleep_ms(interval_ms)


# ---------------------------------------------------------------------------
# AsyncSubscription
# ---------------------------------------------------------------------------

class AsyncSubscription:
    """Awaitable wrapper around a ``rosette.Subscription``."""

    __slots__ = ("_raw",)

    def __init__(self, raw):
        self._raw = raw

    async def recv(self):
        """Wait until a message is in the ring buffer, then decode and return it.

        Suspends the calling coroutine without busy-waiting.  The event loop
        polls the subscription's ``ioctl(MP_STREAM_POLL_RD)`` and resumes
        this task as soon as it returns non-zero.
        """
        yield _ioq.queue_read(self._raw)
        return self._raw.recv()

    @property
    def readable(self):
        """True when a message is already available (non-blocking peek)."""
        return self._raw.queued_count() > 0


# ---------------------------------------------------------------------------
# AsyncPublisher
# ---------------------------------------------------------------------------

class AsyncPublisher:
    """Awaitable wrapper around a ``rosette.Publisher``."""

    __slots__ = ("_raw",)

    def __init__(self, raw):
        self._raw = raw

    async def publish(self, msg):
        """Wait until the publisher is writable, then publish *msg*.

        For best-effort publishers the object always reports writable so this
        returns almost immediately.  For reliable publishers it will block
        until the send queue has room.
        """
        yield _ioq.queue_write(self._raw)
        self._raw.publish(msg)


# ---------------------------------------------------------------------------
# AsyncNode
# ---------------------------------------------------------------------------

class AsyncNode:
    """Thin async façade over ``rosette.Node``."""

    __slots__ = ("_raw",)

    def __init__(self, raw):
        self._raw = raw

    def create_publisher(self, schema, topic, depth=1, reliable=False):
        """Return an :class:`AsyncPublisher` for *topic*."""
        return AsyncPublisher(
            self._raw.create_publisher(schema, topic, depth, reliable))

    def create_subscription(self, schema, topic, depth=4, reliable=False):
        """Return an :class:`AsyncSubscription` for *topic*."""
        return AsyncSubscription(
            self._raw.create_subscription(schema, topic, depth, reliable))

    @property
    def name(self):
        return self._raw.name()

    @property
    def namespace(self):
        return self._raw.node_namespace()


# ---------------------------------------------------------------------------
# AsyncContext
# ---------------------------------------------------------------------------

class AsyncContext:
    """Async context manager that owns the Rosette lifecycle and pump task.

    All keyword arguments except *pump_interval_ms* are forwarded verbatim to
    ``rosette.Context()``.  *pump_interval_ms* controls how often the
    background pump calls ``ctx.tick()`` (default: 5 ms).

    ``__aenter__`` constructs the ``rosette.Context`` and socket but does NOT
    call ``start()`` — this lets you run ``await ctx.discover_agent()`` first
    to find the agent before committing to an XRCE session.  Call
    ``await ctx.start()`` explicitly after discovery (or immediately if you
    already know the agent address).
    """

    __slots__ = ("_kwargs", "_pump_interval_ms", "_raw", "_pump_task")

    def __init__(self, pump_interval_ms=5, **kwargs):
        self._kwargs = kwargs
        self._pump_interval_ms = pump_interval_ms
        self._raw = None
        self._pump_task = None

    async def __aenter__(self):
        self._raw = _rosette.Context(**self._kwargs)
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self._pump_task is not None:
            self._pump_task.cancel()
            try:
                await self._pump_task
            except asyncio.CancelledError:
                pass
        if self._raw is not None:
            self._raw.close()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self):
        """Start the XRCE session and launch the background pump.

        Must be called after construction (and after :meth:`discover_agent`
        if discovery is used).  Raises ``RuntimeError`` if the backend cannot
        start.
        """
        self._raw.start()
        self._pump_task = asyncio.create_task(
            _tick_pump(self._raw, self._pump_interval_ms))

    async def discover_agent(self,
                              timeout_ms=3000,
                              attempts=5,
                              retry_interval_ms=500):
        """Probe the network for a micro-ROS agent using XRCE multicast discovery.

        Implements the Micro XRCE-DDS agent discovery exchange without ever
        blocking the event loop:

        1. C builds the ``GET_INFO`` probe bytes and resolves the multicast addr.
        2. Python sends via ``socket.sendto()`` (non-blocking, fire-and-forget).
        3. Python suspends on ``queue_read`` of the socket — the event loop
           resumes this coroutine when a UDP packet arrives.
        4. Python calls ``recv_packet()`` to collect the bytes.
        5. C parses the reply; if valid, it updates the context endpoint and
           returns ``(ip, port)``; if not, Python loops or times out.

        On success the context's agent endpoint is updated in place and the
        method returns an ``(ip_str, port_int)`` tuple.  On failure after all
        attempts, raises ``OSError``.

        Args:
            timeout_ms: Total wall-clock budget per attempt in milliseconds.
            attempts: Maximum number of probes to send (default 5).
            retry_interval_ms: Yield between failed attempts (default 500 ms).

        Returns:
            ``(ip_str, port_int)`` tuple of the discovered agent.

        Raises:
            OSError: Agent not found after all attempts.
        """
        import utime

        raw = self._raw
        sock = raw.socket()  # the underlying UDP socket object owned by Context

        for attempt in range(attempts):
            # Step 1 + 2: C builds probe; Python sends it to the multicast group.
            probe_bytes, mcast_addr = raw._discovery_build_probe()
            sock.sendto(probe_bytes, mcast_addr)

            # Step 3: await socket readability without blocking.
            # We poll in a tight sleep loop bounded by timeout_ms so we can
            # detect the deadline without holding the socket open indefinitely.
            deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
            while True:
                remaining = utime.ticks_diff(deadline, utime.ticks_ms())
                if remaining <= 0:
                    break

                # Step 3: suspend until the socket is readable or the deadline
                # expires.  queue_read yields to the event loop; it resumes us
                # when ioctl(POLL_RD) on the socket returns non-zero.
                yield _ioq.queue_read(sock)

                # Step 4: non-blocking recv — returns None on EAGAIN (spurious
                # wake from queue_read before data arrived).
                packet = raw.recv_packet()
                if packet is None:
                    continue

                # Step 5: C parses the reply; None means not a valid INFO reply
                # (wrong packet type, bad locator, etc.) — keep waiting.
                result = raw._discovery_parse_reply(packet)
                if result is not None:
                    return result

            # Attempt timed out; pause before retrying.
            if attempt < attempts - 1:
                await asyncio.sleep_ms(retry_interval_ms)

        raise OSError("micro-ROS agent not found on network")

    # ------------------------------------------------------------------
    # Node factory
    # ------------------------------------------------------------------

    def create_node(self, name, namespace="/"):
        """Create and return an :class:`AsyncNode`."""
        return AsyncNode(self._raw.create_node(name, namespace))

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def raw(self):
        """The underlying ``rosette.Context`` for low-level access."""
        return self._raw
