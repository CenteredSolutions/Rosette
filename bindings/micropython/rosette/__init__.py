"""rosette — MicroPython ROS2/micro-ROS client library.

Typical usage with a known agent address::

    import rosette
    ctx = rosette.Context(client_key=0x01020304, domain_id=0,
                          executor_handles=8, spin_timeout_ms=1)
    ctx.open("10.0.0.1", 8888)
    ctx.start()
    node = ctx.create_node("my_node", "/ns")

Or with automatic agent discovery::

    ctx.open()   # agent_ip omitted → Transport runs XRCE multicast discovery
    ctx.start()

Async variant::

    from rosette.asyncio import AsyncContext
    async with AsyncContext(client_key=0x01020304) as ctx:
        await ctx.open()      # runs discovery if no agent_ip
        await ctx.start()
        node = ctx.create_node("my_node", "/ns")
"""

import asyncio
import _rosette

schema = _rosette.schema
from .asyncio import AsyncContext
from .context import Context


# ---------------------------------------------------------------------------
# Action base class
# ---------------------------------------------------------------------------

class Action:
    goal     = None
    feedback = None
    result   = None
    cancel   = None

    def __init__(self, node, topic_prefix):
        self._feedback_pub = node.create_publisher(self.feedback, topic_prefix + "/feedback")
        self._result_pub   = node.create_publisher(self.result,   topic_prefix + "/result")
        node.create_subscription(self.goal, topic_prefix + "/goal", self._dispatch_goal)
        cancel_schema = self.cancel if self.cancel is not None else self.goal
        node.create_subscription(cancel_schema, topic_prefix + "/cancel", self._dispatch_cancel)
        self._tasks = {}

    def goal_key(self, msg):
        return None

    def _dispatch_goal(self, msg):
        key = self.goal_key(msg)
        task = self._tasks.get(key)
        if task and not task.done():
            task.cancel()
        self._tasks[key] = asyncio.create_task(self._run(msg))

    def _dispatch_cancel(self, msg):
        key = self.goal_key(msg)
        task = self._tasks.get(key)
        if task and not task.done():
            task.cancel()

    def on_goal(self, msg):
        return True

    async def _run(self, goal):
        if not self.on_goal(goal):
            return
        try:
            result = await self.execute(goal)
            self._result_pub.publish(result)
        except asyncio.CancelledError:
            self._result_pub.publish(self.cancelled(goal))
        except Exception as e:
            self._result_pub.publish(self.failed(goal, e))

    def publish_feedback(self, msg):
        self._feedback_pub.publish(msg)

    async def execute(self, goal):
        raise NotImplementedError

    def cancelled(self, goal):
        return {"success": False, "message": "cancelled"}

    def failed(self, goal, exc):
        return {"success": False, "message": str(exc)}


# ---------------------------------------------------------------------------
# Service base class
# ---------------------------------------------------------------------------

class Service:
    request  = None
    response = None

    def __init__(self, node, topic_prefix):
        self._pub = node.create_publisher(self.response, topic_prefix + "/response")
        node.create_subscription(self.request, topic_prefix + "/request", self._dispatch)

    def _dispatch(self, msg):
        asyncio.create_task(self._run(msg))

    async def _run(self, req):
        try:
            self._pub.publish(await self.handle(req))
        except Exception as e:
            self._pub.publish({"success": False, "message": str(e)})

    async def handle(self, req):
        raise NotImplementedError
