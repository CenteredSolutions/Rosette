import _rosette
from .transport import Transport

# ---------------------------------------------------------------------------
# Context
# ---------------------------------------------------------------------------

class Context(_rosette.Context):
    """rosette.Context — config holder with lazy socket/transport setup.

    Construct freely even before the network is up. Call open() once the
    network interface is ready, then start() to begin the XRCE session.
    """
    def open(self, agent_ip=None, agent_port=8888, local_port=0):
        """Create the Transport, open the socket, and wire it into the backend.

        If agent_ip is None, Transport performs XRCE multicast discovery to
        find the agent. Idempotent — safe to call more than once.
        """
        if self.transport() is not None:
            return
        t = Transport()
        t.open(agent_ip=agent_ip, agent_port=agent_port, local_port=local_port)
        self._set_transport(t, t.agent_ip, t.agent_port)
