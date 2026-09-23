"""tcpdump-based RTPS packet capture for ROS 2 interop sessions.

Architecture:
    `rmw_interop_probe.py` optionally wraps a probe session in a capture so a
    failed interop run leaves behind a `.pcap` that Wireshark's RTPS dissector
    can decode. The capture is scoped to one DDS domain's port block, which is
    wide enough to hold SPDP multicast, SEDP metatraffic, and user traffic for
    every participant in that domain regardless of which RMW created it.

References:
    - OMG DDSI-RTPS 2.5 §9.6.1.1 (default port mapping: PB=7400, DG=250,
      PG=2, d0=0, d1=10, d2=1, d3=11).
"""

from __future__ import annotations

import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# DDSI-RTPS 2.5 §9.6.1.1 default port-mapping constants.
_PORT_BASE = 7400
_DOMAIN_GAIN = 250
# Participant ids 0..44 on unicast (d1/d3 + PG * id) stay under base + 100.
_DOMAIN_PORT_SPAN = 100


def choose_capture_iface(explicit_iface: str | None) -> str:
    """Pick the interface tcpdump listens on.

    Args:
        explicit_iface: Interface requested by the caller, or None to pick one.

    Returns:
        `explicit_iface` when given; Linux's synthetic `any` device otherwise;
        on macOS (which has no `any`), the default-route interface.

    Raises:
        RuntimeError: No interface was given and none could be determined.
    """
    if explicit_iface:
        return explicit_iface
    if sys.platform.startswith("linux"):
        return "any"
    try:
        result = subprocess.run(
            ["route", "-n", "get", "default"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(
            "could not determine a default capture interface; pass --capture-iface"
        ) from exc
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("interface:"):
            return stripped.split(":", 1)[1].strip()
    raise RuntimeError(
        "could not determine a default capture interface; pass --capture-iface"
    )


def build_capture_filter(domain: int) -> str:
    """Return a BPF filter covering one DDS domain's RTPS port block.

    Args:
        domain: DDS domain id (ROS_DOMAIN_ID), 0..232.

    Returns:
        A tcpdump filter for UDP ports `PB + DG * domain` through
        `PB + DG * domain + 99`.

    Verification:
        Domain 0 yields `portrange 7400-7499`, which holds SPDP multicast
        (7400), user multicast (7401), and participant 0's unicast
        metatraffic/user ports (7410/7411), matching DDSI-RTPS 2.5 §9.6.1.1.
    """
    base_port = _PORT_BASE + _DOMAIN_GAIN * domain
    return f"udp and portrange {base_port}-{base_port + _DOMAIN_PORT_SPAN - 1}"


@dataclass
class CaptureHandle:
    """A running tcpdump process and the files it writes."""

    process: subprocess.Popen[str]
    pcap_file: Path
    log_file: Path

    def stop(self) -> None:
        """Stop tcpdump with SIGINT so it flushes the pcap, killing it if it hangs."""
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)


def start_capture(*, domain: int, iface: str, pcap_file: Path) -> CaptureHandle:
    """Start tcpdump for one domain's RTPS traffic.

    Args:
        domain: DDS domain id whose port block is captured.
        iface: Interface to listen on (see `choose_capture_iface`).
        pcap_file: Output capture path; a sibling `.tcpdump.log` receives
            tcpdump's own output.

    Returns:
        A handle whose `stop()` must be called to finalize the pcap.

    Raises:
        RuntimeError: tcpdump is missing or exited during startup, usually
            for lack of capture privileges or a wrong interface.
    """
    tcpdump = shutil.which("tcpdump")
    if not tcpdump:
        raise RuntimeError("tcpdump not found in PATH")

    pcap_file.parent.mkdir(parents=True, exist_ok=True)
    log_file = pcap_file.with_suffix(".tcpdump.log")
    log_handle = log_file.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [tcpdump, "-i", iface, "-n", "-s", "0", "-U", "-w", str(pcap_file),
         build_capture_filter(domain)],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # tcpdump reports permission and interface errors by exiting right away.
    time.sleep(1.5)
    if process.poll() is not None:
        log_handle.close()
        log_text = log_file.read_text(encoding="utf-8", errors="replace")
        raise RuntimeError(
            "tcpdump exited immediately; it likely needs elevated privileges "
            f"or a different interface.\n{log_text}"
        )
    return CaptureHandle(process=process, pcap_file=pcap_file, log_file=log_file)
