import asyncio
from asyncio_queue import Queue
from .context import Context

# uasyncio exposes its I/O queue as asyncio.core._io_queue (MicroPython ≥1.20).
_ioq = asyncio.core._io_queue  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Pump
# ---------------------------------------------------------------------------

async def _tick_pump(ctx_raw, interval_ms):
    """Drive the micro-ROS executor in the background."""
    while ctx_raw.started():
        ctx_raw.tick()
        await asyncio.sleep_ms(interval_ms)


# ---------------------------------------------------------------------------
# AsyncSubscription
# ---------------------------------------------------------------------------

class AsyncSubscription:
    """Awaitable wrapper around a ``rosette.Subscription``."""

    def __init__(self, raw_factory):
        self._queue = Queue(1)
        self._raw = raw_factory(self._recv)

    def _recv(self, msg):
        self._queue.put_nowait(msg)

    async def recv(self):
        """Wait until a message is in the ring buffer, then return it."""
        return await self._queue.get()

    @property
    def readable(self):
        return self._raw.queued_count() > 0


# ---------------------------------------------------------------------------
# AsyncPublisher
# ---------------------------------------------------------------------------

class AsyncPublisher:
    """Awaitable wrapper around a ``rosette.Publisher``."""

    def __init__(self, raw):
        self._raw = raw

    async def publish(self, msg):
        yield _ioq.queue_write(self._raw)
        self._raw.publish(msg)


# ---------------------------------------------------------------------------
# AsyncNode
# ---------------------------------------------------------------------------

class AsyncNode:
    """Thin async façade over ``rosette.Node``."""

    def __init__(self, raw):
        self._raw = raw

    def create_publisher(self, schema, topic, depth=1, reliable=False):
        return AsyncPublisher(
            self._raw.create_publisher(schema, topic, depth, reliable))

    def create_subscription(self, schema, topic, depth=4, reliable=False):
        return AsyncSubscription(
            lambda fn: self._raw.create_subscription(schema, topic, fn, depth, reliable))

    @property
    def name(self):
        return self._raw.name()

    @property
    def namespace(self):
        return self._raw.node_namespace()


# ---------------------------------------------------------------------------
# AsyncContext
# ---------------------------------------------------------------------------

class AsyncContext(Context):
    """Async context manager that owns the Rosette lifecycle and pump task.

    Keyword arguments (except *pump_interval_ms*, *agent_ip*, *agent_port*,
    *local_port*) are forwarded to ``rosette.Context()``.

    ``__aenter__`` constructs a ``Context`` but does NOT open or start it.
    Call ``await ctx.open()`` (with or without an agent address) to set up
    the transport, then ``await ctx.start()`` to begin the XRCE session.

    Typical usage with a known agent::

        async with AsyncContext(client_key=0x01020304) as ctx:
            ctx.open("10.0.0.1", 8888)
            await ctx.start()
            node = ctx.create_node("my_node", "/ns")

    With async discovery::

        async with AsyncContext(client_key=0x01020304) as ctx:
            t = Transport()
            t.open(local_port=0)          # socket open, no agent yet
            ip, port = await t.discover_agent()
            t.connect(ip, port)
            ctx._raw._set_transport(t)
            await ctx.start()
    """

    def __init__(self, pump_interval_ms=8, **kwargs):
        super().__init__(**kwargs)
        self._pump_interval_ms = pump_interval_ms
        self._pump_task = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self._pump_task is not None:
            self._pump_task.cancel()
        self.close()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self):
        """Start the XRCE session and launch the background pump.

        The transport must be set (via ``open()`` or ``_set_transport()``)
        before calling this. Raises ``RuntimeError`` if the backend fails.
        """
        async def pump(interval_ms):
            while True:
                self.tick()
                await asyncio.sleep_ms(interval_ms)

        self._pump_task = asyncio.create_task(
            pump(self._pump_interval_ms))

        super().start()

    # ------------------------------------------------------------------
    # Node factory
    # ------------------------------------------------------------------

    def create_node(self, name, namespace="/"):
        return AsyncNode(super().create_node(name, namespace))
