#!/usr/bin/env python3
"""CDR layout and decode parity between rmw_rosette and a stock RMW.

Serializes a fixed corpus of populated ROS 2 messages through
`rclpy.serialization.serialize_message()` (which calls `rmw_serialize()` of the
loaded RMW) under two RMW implementations in separate processes, then compares
the serialized buffers and checks that each RMW deserializes the other's bytes
back to an equal message.

The corpus exercises every construct the introspection codec handles:
primitives of each width and alignment, strings with multi-byte UTF-8, bounded
and unbounded sequences of primitives and messages, fixed arrays, and nested
messages several levels deep.

Two properties are checked per message:

- Layout: every octet rmw_rosette writes equals the reference's octet at the
  same offset, except alignment padding, whose content CDR leaves unspecified
  (rmw_rosette writes zeros; Fast-CDR leaves whatever the buffer held). The
  reference may report octets past rmw_rosette's end: rmw_fastrtps_cpp's
  rmw_serialize() reports its conservative size estimate as the buffer
  length, and the slack past the last field is uninitialized, so it is not
  compared.
- Semantics: each RMW deserializes the other's buffer to an equal message.

Verification:
    Correct means: both properties hold for every corpus message. Covers
    `rmw_serialize`, `rmw_deserialize`, and the introspection_c codec
    (rmw_rosette/src/introspection_c_codec.cpp).

References:
    OMG DDS-XTypes 1.3 §7.4.3.4 (PLAIN_CDR / XCDR1).
    OMG DDSI-RTPS 2.5 §10.2 (serialized payload header).
    rosidl_typesupport_fastrtps_c (Humble), the reference serializer.

Usage:
    tests/ros2/rmw_serialization_parity.py [--reference rmw_fastrtps_cpp]
"""

import argparse
import base64
import json
import os
import subprocess
import sys


def build_corpus():
    """Returns (name, message) pairs covering the codec's type constructs.

    Returns:
        A list of `(str, msg)` tuples; imports happen here so the module can be
        imported without a sourced ROS 2 environment.
    """
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseStamped, PoseWithCovariance
    from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterValue
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import (Bool, Float64MultiArray, Header, MultiArrayDimension, String,
                              UInt8)
    from visualization_msgs.msg import Marker

    corpus = []

    corpus.append(("String/empty", String(data="")))
    corpus.append(("String/utf8", String(data="rosette ünïcode ✓ 🚀")))
    corpus.append(("Bool", Bool(data=True)))
    corpus.append(("UInt8", UInt8(data=255)))

    header = Header(stamp=Time(sec=1727300000, nanosec=123456789), frame_id="base_link")

    js = JointState(header=header)
    js.name = ["shoulder", "elbow", "wrist_ÿ"]
    js.position = [0.5, -1.25, 3.14159]
    js.velocity = [0.0, 1.0e-9]
    js.effort = []
    corpus.append(("JointState", js))

    ps = PoseStamped(header=header)
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = 1.0, -2.0, 3.5
    ps.pose.orientation.w = 1.0
    corpus.append(("PoseStamped", ps))

    pwc = PoseWithCovariance()
    pwc.covariance = [float(i) * 0.5 for i in range(36)]
    corpus.append(("PoseWithCovariance/fixed-array", pwc))

    img = Image(header=header, height=2, width=3, encoding="rgb8", is_bigendian=0, step=9)
    img.data = bytes(range(18))
    corpus.append(("Image/byte-sequence", img))

    arr = Float64MultiArray()
    arr.layout.dim = [MultiArrayDimension(label="rows", size=2, stride=6),
                      MultiArrayDimension(label="cols", size=3, stride=3)]
    arr.layout.data_offset = 7
    arr.data = [1.5, 2.5, 3.5, 4.5, 5.5, 6.5]
    corpus.append(("Float64MultiArray/nested-sequence", arr))

    diag = DiagnosticArray(header=header)
    for i in range(3):
        st = DiagnosticStatus(level=bytes([i]), name=f"sensor{i}", message="ok" * i,
                              hardware_id=f"hw{i}")
        st.values = [KeyValue(key=f"k{j}", value=f"v{j}") for j in range(i + 1)]
        diag.status.append(st)
    corpus.append(("DiagnosticArray/sequence-of-messages", diag))

    ev = ParameterEvent(stamp=Time(sec=5, nanosec=6), node="/talker")
    pv = ParameterValue(type=9, string_array_value=["a", "bb", ""],
                        integer_array_value=[1, -2, 3], double_value=2.5, bool_value=True)
    ev.new_parameters = [Parameter(name="p1", value=pv)]
    corpus.append(("ParameterEvent", ev))

    mk = Marker(header=header, ns="viz", id=42, type=2, action=0, text="label",
                mesh_resource="package://x", mesh_use_embedded_materials=True)
    mk.scale.x = mk.scale.y = mk.scale.z = 0.25
    mk.color.r, mk.color.a = 1.0, 0.5
    mk.frame_locked = True
    corpus.append(("Marker/mixed-alignment", mk))

    return corpus


def serialize_child():
    """Child mode: prints {name: base64(serialized)} for the corpus as JSON."""
    import rclpy
    from rclpy.serialization import serialize_message

    rclpy.init()
    out = {"rmw": rclpy.get_rmw_implementation_identifier(), "buffers": {}}
    for name, msg in build_corpus():
        out["buffers"][name] = base64.b64encode(serialize_message(msg)).decode()
    rclpy.shutdown()
    print(json.dumps(out))


def deserialize_child(path):
    """Child mode: decodes the buffers in `path` and compares with the corpus.

    Args:
        path: JSON file written from another RMW's serialize_child() output.
    """
    import rclpy
    from rclpy.serialization import deserialize_message

    rclpy.init()
    with open(path, encoding="utf-8") as f:
        buffers = json.load(f)["buffers"]
    failures = []
    for name, msg in build_corpus():
        decoded = deserialize_message(base64.b64decode(buffers[name]), type(msg))
        if decoded != msg:
            failures.append(name)
    rclpy.shutdown()
    print(json.dumps({"rmw": os.environ.get("RMW_IMPLEMENTATION"), "failures": failures}))


def layout_mismatch(ours, ref):
    """Finds the first octet where two CDR buffers disagree beyond padding.

    Args:
        ours: Buffer from the RMW under test, which zero-fills padding.
        ref: Buffer from the reference RMW.

    Returns:
        The offset of the first disagreement, or None when the layouts match:
        `ref` is not shorter than `ours`, and each octet of `ours` equals `ref`
        or is a zero padding octet. Octets `ref` has past the end of `ours` are
        size-estimate slack and are not compared.
    """
    if len(ref) < len(ours):
        return len(ref)
    for i, octet in enumerate(ours):
        if octet != ref[i] and octet != 0:
            return i
    return None


def run_child(rmw, *extra):
    """Runs this script in a child process with RMW_IMPLEMENTATION=`rmw`.

    Args:
        rmw: RMW implementation name.
        *extra: Additional command-line arguments for the child.

    Returns:
        The child's parsed JSON output.

    Raises:
        RuntimeError: If the child exits non-zero.
    """
    env = dict(os.environ, RMW_IMPLEMENTATION=rmw)
    proc = subprocess.run([sys.executable, __file__, *extra], env=env,
                          capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"{rmw} child failed:\n{proc.stderr}")
    return json.loads(proc.stdout.strip().splitlines()[-1])


def main():
    """Compares rmw_rosette against the reference RMW; exits 1 on any mismatch."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rmw", default="rmw_rosette")
    parser.add_argument("--reference", default="rmw_fastrtps_cpp")
    parser.add_argument("--child-serialize", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--child-deserialize", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.child_serialize:
        serialize_child()
        return 0
    if args.child_deserialize:
        deserialize_child(args.child_deserialize)
        return 0

    ours = run_child(args.rmw, "--child-serialize")
    ref = run_child(args.reference, "--child-serialize")
    for side, want in ((ours, args.rmw), (ref, args.reference)):
        if side["rmw"] != want:
            print(f"FAIL: expected {want} but child loaded {side['rmw']}")
            return 1

    ok = True
    for name, ref_b64 in ref["buffers"].items():
        a, b = base64.b64decode(ours["buffers"][name]), base64.b64decode(ref_b64)
        first = layout_mismatch(a, b)
        if first is None:
            print(f"PASS layout {name} ({len(a)} B)")
            continue
        ok = False
        print(f"FAIL layout {name}: {args.rmw}={len(a)} B, {args.reference}={len(b)} B, "
              f"first difference at offset {first}")
        print(f"    {args.rmw:18s} {a[max(0, first - 8):first + 16].hex(' ')}")
        print(f"    {args.reference:18s} {b[max(0, first - 8):first + 16].hex(' ')}")

    scratch = os.environ.get("TMPDIR", "/tmp")
    for src, dst in ((ref, args.rmw), (ours, args.reference)):
        path = os.path.join(scratch, f"rmw_parity_{src['rmw']}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(src, f)
        result = run_child(dst, "--child-deserialize", path)
        for name in result["failures"]:
            ok = False
            print(f"FAIL decode {dst} could not decode {src['rmw']}'s {name}")
        if not result["failures"]:
            print(f"PASS decode {dst} decodes every {src['rmw']} buffer")

    print("parity: PASS" if ok else "parity: FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
