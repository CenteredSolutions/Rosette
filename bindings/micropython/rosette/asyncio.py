"""uasyncio integration for the Rosette MicroPython binding."""

import asyncio
import asyncio.core as _acore

from .context import Context

_ioq = _acore._io_queue


async def _wait_socket_read(sock):
    yield _ioq.queue_read(sock)


class AsyncNode:
    """Async-context node facade with synchronous ROS entities."""

    def __init__(self, raw):
        self._raw = raw

    def create_publisher(self, schema, topic, depth=1, reliable=False):
        return self._raw.create_publisher(schema, topic, depth, reliable)

    def create_subscription(self, schema, topic, callback=None, depth=4, reliable=False):
        return self._raw.create_subscription(schema, topic, callback, depth, reliable)

    @property
    def name(self):
        return self._raw.name()

    @property
    def namespace(self):
        return self._raw.node_namespace()


class AsyncContext(Context):
    """Async context manager that owns the Rosette lifecycle and pump task."""

    def __init__(self, **kwargs):
        kwargs['spin_timeout_ms'] = 0  # Don't block on spin, we'll pump manually in the async task.
        super().__init__(**kwargs)
        self._pump_task = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self._pump_task is not None:
            self._pump_task.cancel()
            self._pump_task = None
        self.close()

    async def start(self, pump_interval_ms=100):
        """Start the XRCE session and launch the cooperative context pump."""
        super().start()
        self._pump_task = asyncio.create_task(self._pump(pump_interval_ms))

    async def _pump(self, pump_interval_ms):
        """Wait for transport input while still ticking executor timers."""
        transport = self.transport()
        while self.started():
            sock = getattr(transport, "s", None) if transport is not None else None
            if sock is not None:
                try:
                    await asyncio.wait_for_ms(
                        _wait_socket_read(sock), pump_interval_ms)
                except asyncio.TimeoutError:
                    pass
            else:
                await asyncio.sleep_ms(pump_interval_ms)
            self.tick()

    def create_node(self, name, namespace="/"):
        return AsyncNode(super().create_node(name, namespace))
