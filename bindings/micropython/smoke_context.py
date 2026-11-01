# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

import rosette


def main():
    ctx = rosette.Context(
        client_key=0x01020304,
        domain_id=0,
        executor_handles=8,
        spin_timeout_ms=1,
    )
    print("context:", ctx)
    print("started before:", ctx.started())

    ctx.open("127.0.0.1", 8888, local_port=0)
    ctx.start()
    print("started after:", ctx.started())
    print("tick:", ctx.tick())

    node = ctx.create_node("demo", "/smoke")
    print("node:", node)

    schema = rosette.schema("std_msgs/String", "string data\n")
    print("schema:", schema)
    pub = node.create_publisher(schema, "smoke")
    print("publisher:", pub)
    print("publish result:", pub.publish({"data": "hello"}))

    seen = []
    sub = node.create_subscription(schema, "smoke", lambda msg: seen.append(msg))
    print("subscription:", sub)
    manual_sub = node.create_subscription(schema, "smoke/manual")
    print("manual subscription:", manual_sub)

    ctx.close()
    print("started final:", ctx.started())


main()
