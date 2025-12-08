#!/usr/bin/env python3
import argparse
import time
import zenoh as Zenoh
from zenoh import Config
from keelson import enclose
from keelson.payloads.Primitives_pb2 import TimestampedFloat

VESSEL = "MASS_0"
ROC_ID = "ROC_2"


class ROC2Controller:
    def __init__(self):
        cfg = Config()
        self.zenoh = Zenoh.open(cfg)

        self.pub_cog = self.zenoh.declare_publisher(f"{VESSEL}/control/roc/{ROC_ID}/COG")
        self.pub_sog = self.zenoh.declare_publisher(f"{VESSEL}/control/roc/{ROC_ID}/SOG")
        self.pub_takeover = self.zenoh.declare_publisher(f"{VESSEL}/handover/takeover")

    def send_cog(self, value):
        msg = TimestampedFloat()
        msg.timestamp.FromNanoseconds(time.time_ns())
        msg.value = float(value)
        self.pub_cog.put(enclose(msg.SerializeToString()))
        print(f"[ROC_2] Sent COG={value}")

    def send_sog(self, value):
        msg = TimestampedFloat()
        msg.timestamp.FromNanoseconds(time.time_ns())
        msg.value = float(value)
        self.pub_sog.put(enclose(msg.SerializeToString()))
        print(f"[ROC_2] Sent SOG={value}")

    def send_takeover(self):
        self.pub_takeover.put(ROC_ID)
        print("[ROC_2] Sent takeover")


def main():
    parser = argparse.ArgumentParser(description="ROC_2 CLI Controller")
    parser.add_argument(
        "command",
        choices=["cog", "sog", "takeover"],
        help="Command to send"
    )
    parser.add_argument(
        "value",
        nargs="?",
        help="Value for cog/sog"
    )

    args = parser.parse_args()
    roc = ROC2Controller()

    if args.command == "cog":
        if args.value is None:
            raise SystemExit("Error: cog requires numeric value")
        roc.send_cog(args.value)

    elif args.command == "sog":
        if args.value is None:
            raise SystemExit("Error: sog requires numeric value")
        roc.send_sog(args.value)

    elif args.command == "takeover":
        roc.send_takeover()


if __name__ == "__main__":
    main()
