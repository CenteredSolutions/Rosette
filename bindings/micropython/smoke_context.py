import rosette


def main():
    ctx = rosette.Context(
        agent_ip="127.0.0.1",
        agent_port=8888,
        local_port=0,
        client_key=0x01020304,
        domain_id=0,
        executor_handles=8,
        spin_timeout_ms=1,
    )
    print("context:", ctx)
    print("started before:", ctx.started())
    print("connected before:", ctx.connected())

    ctx.start()
    print("started after:", ctx.started())
    print("tick:", ctx.tick())
    print("recv before data:", ctx.recv_packet())

    node = ctx.create_node("demo", "/smoke")
    print("node:", node)

    schema = rosette.schema("std_msgs/String", "string data\n")
    print("schema:", schema)

    ctx.close()
    print("started final:", ctx.started())


main()
