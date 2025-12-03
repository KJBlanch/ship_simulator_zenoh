#!/usr/bin/env python3
import time
import zenoh
from datetime import datetime

from keelson import uncover, construct_pubsub_key
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt, TimestampedString
from keelson.payloads.VesselNavStatus_pb2 import VesselNavStatus
from keelson.payloads.ROCStatus_pb2 import ROCStatus


class ShipTelemetryMonitor:
    def __init__(self, ship_name="MASS_0"):
        self.ship = ship_name

        # Latest telemetry
        self.location = None
        self.cog = None
        self.sog = None
        self.remote_status = None
        self.remote_time = None
        self.nav_status = None
        self.mmsi = None
        self.imo = None

        cfg = zenoh.Config()
        self.session = zenoh.open(cfg)

        self.base = "rise/@v0"

        # ----------- ALL SUBSCRIPTIONS NOW USING construct_pubsub_key() -------------

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "location_fix", "gnss/0"),
            self._handle_location)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "course_over_ground_deg", "gnss/0"),
            self._handle_cog)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "speed_over_ground_knots", "gnss/0"),
            self._handle_sog)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "name", "registrar/0"),
            self._handle_name)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "mmsi_number", "registrar/0"),
            self._handle_mmsi)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "imo_number", "registrar/0"),
            self._handle_imo)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "nav_status", "bridge/0"),
            self._handle_nav_status)

        self.session.declare_subscriber(
            construct_pubsub_key(self.base, self.ship, "roc_status", "bridge/0"),
            self._handle_roc_status)

        # RAW zenoh strings (not keelson)
        self.session.declare_subscriber(
            f"{self.base}/{self.ship}/pubsub/remote_status/bridge/0",
            self._handle_remote_status)

        self.session.declare_subscriber(
            f"{self.base}/{self.ship}/pubsub/remote_time/bridge/1",
            self._handle_remote_time)


    # -------------------------------------------------------------------
    # DECODING HELPERS
    # -------------------------------------------------------------------

    def _decode(self, sample, cls):
        try:
            _, _, payload = uncover(sample.payload.to_bytes())
            return cls.FromString(payload)
        except Exception as e:
            print(f"[WARN] Decode failed for {cls.__name__}: {e}")
            return None

    # -------------------------------------------------------------------
    # MESSAGE CALLBACKS
    # -------------------------------------------------------------------

    def _handle_location(self, sample):
        msg = self._decode(sample, LocationFix)
        if msg:
            self.location = (msg.latitude, msg.longitude)

    def _handle_cog(self, sample):
        msg = self._decode(sample, TimestampedFloat)
        if msg:
            self.cog = msg.value

    def _handle_sog(self, sample):
        msg = self._decode(sample, TimestampedFloat)
        if msg:
            self.sog = msg.value

    def _handle_name(self, sample):
        self._decode(sample, TimestampedString)

    def _handle_mmsi(self, sample):
        msg = self._decode(sample, TimestampedInt)
        if msg:
            self.mmsi = msg.value

    def _handle_imo(self, sample):
        msg = self._decode(sample, TimestampedInt)
        if msg:
            self.imo = msg.value

    def _handle_nav_status(self, sample):
        msg = self._decode(sample, VesselNavStatus)
        if msg:
            self.nav_status = VesselNavStatus.NavigationStatus.Name(msg.navigation_status)

    def _handle_roc_status(self, sample):
        self._decode(sample, ROCStatus)

    def _handle_remote_status(self, sample):
        try:
            self.remote_status = sample.payload.to_string()
        except:
            self.remote_status = None

    def _handle_remote_time(self, sample):
        try:
            self.remote_time = float(sample.payload.to_string())
        except:
            self.remote_time = None

    # -------------------------------------------------------------------
    # TERMINAL UI
    # -------------------------------------------------------------------

    def print_terminal(self):
        while True:
            try:
                print("\033c", end="")  # Clear screen

                print(f"================= {self.ship} Telemetry =================")

                if self.location:
                    print(f"Lat:      {self.location[0]:.6f}")
                    print(f"Lon:      {self.location[1]:.6f}")
                else:
                    print("Lat/Lon:  waiting...")

                print(f"COG:      {self.cog:.1f}°" if self.cog else "COG: waiting...")
                print(f"SOG:      {self.sog:.2f} kn" if self.sog else "SOG: waiting...")

                print()
                print(f"Nav Status: {self.nav_status}" if self.nav_status else "Nav Status: waiting...")

                if self.remote_status:
                    print(f"Remote Status: {self.remote_status}")
                if self.remote_time:
                    print(f"Gate ETA: {self.remote_time:.1f} sec")

                print()
                print(f"MMSI: {self.mmsi}" if self.mmsi else "MMSI: waiting...")
                print(f"IMO:  {self.imo}" if self.imo else "IMO: waiting...")

                print()
                print(f"Last update: {datetime.now().strftime('%H:%M:%S')}")
                print("====================================================")
            
            except KeyboardInterrupt:
                print("Interrupted")
                self.session.close()

            time.sleep(0.5)

# -------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------
if __name__ == "__main__":
    mon = ShipTelemetryMonitor("MASS_0")
    mon.print_terminal()
