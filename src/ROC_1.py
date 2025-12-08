#!/usr/bin/env python3
import argparse
import time
import zenoh as Zenoh
from zenoh import Config
from keelson import enclose
from keelson.payloads.Primitives_pb2 import TimestampedFloat

VESSEL = "MASS_0"
ROC_ID = "ROC_1"


class ROC1Controller:
    def __init__(self):
        cfg = Config()
        self.zenoh = Zenoh.open(cfg)

        self.pub_cog = self.zenoh.declare_publisher(f"{VESSEL}/control/roc/{ROC_ID}/COG")
        self.pub_sog = self.zenoh.declare_publisher(f"{VESSEL}/control/roc/{ROC_ID}/SOG")
        self.pub_relinquish = self.zenoh.declare_publisher(f"{VESSEL}/handover/relinquish")

    def send_cog(self, value):
        msg = TimestampedFloat()
        msg.timestamp.FromNanoseconds(time.time_ns())
        msg.value = float(value)
        self.pub_cog.put(enclose(msg.SerializeToString()))
        print(f"[ROC_1] Sent COG={value}")

    def send_sog(self, value):
        msg = TimestampedFloat()
        msg.timestamp.FromNanoseconds(time.time_ns())
        msg.value = float(value)
        self.pub_sog.put(enclose(msg.SerializeToString()))
        print(f"[ROC_1] Sent SOG={value}")

    def send_relinquish(self):
        self.pub_relinquish.put(ROC_ID)
        print("[ROC_1] Sent relinquish")


def main():
    parser = argparse.ArgumentParser(description="ROC_1 CLI Controller")
    parser.add_argument(
        "command",
        choices=["cog", "sog", "relinquish"],
        help="Command to send"
    )
    parser.add_argument(
        "value",
        nargs="?",
        help="Value for cog/sog"
    )

    args = parser.parse_args()
    roc = ROC1Controller()

    if args.command == "cog":
        if args.value is None:
            raise SystemExit("Error: cog requires numeric value")
        roc.send_cog(args.value)

    elif args.command == "sog":
        if args.value is None:
            raise SystemExit("Error: sog requires numeric value")
        roc.send_sog(args.value)

    elif args.command == "relinquish":
        roc.send_relinquish()


if __name__ == "__main__":
    main()
