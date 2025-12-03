import argparse
import zenoh as Zenoh
from zenoh import Config

# Ship controller CLI
# Sends Zenoh commands to control the simulation ship

class ShipController:
    def __init__(self, prefix="testship"):
        cfg = Config()
        self.zenoh = Zenoh.open(cfg)
        self.prefix = prefix

    def send_state(self, state):
        if state not in ["go", "no-go"]:
            raise ValueError("State must be 'go' or 'no-go'")
        self.zenoh.put(f"{self.prefix}/state", state)
        print(f"[Zenoh] Sent state: {state}")

    def send_sog(self, sog):

        #TODO - build keelson based sender

        print(f"[Zenoh] Sent SOG: {value} knots")

    def send_cog(self, cog):

        #TODO - build keelson based sender

        print(f"[Zenoh] Sent COG: {value} degrees")


def main():
    parser = argparse.ArgumentParser(description="Ship Controller CLI using Zenoh")
    parser.add_argument("command", choices=["go", "no-go", "sog", "cog"], help="Command to send")
    parser.add_argument("value", nargs="?", help="Value for SOG (if using sog command)")

    args = parser.parse_args()

    controller = ShipController()

    if args.command in ["go", "no-go"]:
        controller.send_state(args.command)
    elif args.command == "sog":
        if args.value is None:
            raise SystemExit("Error: SOG command requires a numeric value")
        controller.send_sog(args.value)
    elif args.command == "cog":
        if args.value is None:
            raise SystemExit("Error: COG command requires a numeric value")
        controller.send_cog(args.value)


if __name__ == "__main__":
    main()
