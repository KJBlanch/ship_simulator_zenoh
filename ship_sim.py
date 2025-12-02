#!/usr/bin/env python3
import math
import random
import sys
import time

import matplotlib.pyplot as plt
import zenoh as Zenoh
from zenoh import Session, Config

# Keelson imports
import keelson
from keelson import enclose, construct_pubsub_key
from keelson.payloads.Primitives_pb2 import (
    TimestampedFloat,
    TimestampedInt,
    TimestampedString,
)
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.payloads.VesselNavStatus_pb2 import VesselNavStatus
from keelson.payloads.ROCStatus_pb2 import ROCStatus


# ---------------------------------------------------------------------
# Basic non-C++ struct for declaring a safety gate
# ---------------------------------------------------------------------
class SafteyGate:
    def __init__(self, latitude, longitude, radius, ident=0, active=True):
        self.latitude = latitude
        self.longitude = longitude
        self.radius = radius
        self.active = active
        self.ident = ident


# ---------------------------------------------------------------------
# Ship class
# ---------------------------------------------------------------------
class Ship:
    # Zenoh session shared among all ships (class-level singleton)
    _zenoh = None

    @staticmethod
    def init_zenoh():
        if Ship._zenoh is None:
            cfg = Config()
            Ship._zenoh = Zenoh.open(cfg)
        return Ship._zenoh

    def __init__(
        self,
        name="MASS_0",
        controller_id=1,
        latitude=30.0,
        longitude=-40.0,
        cog_deg=0.0,
        sog_knots=10.0,
        tonnage=5000,
        length_m=120,
        mmsi_number=265123000,
        imo_number=9319466,
    ):
        # Basic identity / kinematics
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.cog_deg = cog_deg  # Course over ground in degrees
        self.sog_knots = sog_knots  # Speed over ground in knots
        self.tonnage = tonnage
        self.length_m = length_m
        self.controller_id = controller_id

        # Registrar values
        self.mmsi_number = mmsi_number
        self.imo_number = imo_number

        # Navigation / ROC status
        self.nav_status_value = VesselNavStatus.NavigationStatus.UNDER_WAY
        # Default: one ROC entity
        self.roc_entities = [
            ("bridge-autopilot", ROCStatus.ROCEntity.State.MONITORING)
        ]

        # Track course history
        self.history_lat = [latitude]
        self.history_lon = [longitude]
        self.state = "go"  # 'go' or 'no-go'

        # Derived deceleration factor (simple model; larger ships slow down more slowly)
        self.decel_rate = max(0.01, min(0.5, 5000 / (self.tonnage + 1)))

        # Zenoh initialization
        self.zenoh = Ship.init_zenoh()
        self._start_zenoh_subscribers()

        # Safety-gate knowledge
        self.safety_gates = []

        # Keelson pubsub base path
        self._base_path = "rise/@v0"

        # Declare Keelson publishers according to schema
        # GNSS / kinematics
        self.pub_location_fix = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "location_fix", "gnss/0"
            )
        )
        self.pub_cog = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path,
                self.name,
                "course_over_ground_deg",
                "gnss/0",
            )
        )
        self.pub_sog = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path,
                self.name,
                "speed_over_ground_knots",
                "gnss/0",
            )
        )

        # Registrar
        self.pub_name = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "name", "registrar/0"
            )
        )
        self.pub_mmsi = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "mmsi_number", "registrar/0"
            )
        )
        self.pub_imo = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "imo_number", "registrar/0"
            )
        )

        # Bridge status
        self.pub_nav_status = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "nav_status", "bridge/0"
            )
        )
        self.pub_roc_status = self.zenoh.declare_publisher(
            construct_pubsub_key(
                self._base_path, self.name, "roc_status", "bridge/0"
            )
        )

        # Remote status / time (plain strings, no keelson encoding)
        self.pub_remote_status = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/pubsub/remote_status/bridge/0"
        )
        self.pub_remote_time = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/pubsub/remote_time/bridge/1"
        )

    # ------------------------------------------------------------------
    # Safety gate management
    # ------------------------------------------------------------------
    def add_s_gate(self, gate: SafteyGate):
        existing_ids = {g.ident for g in self.safety_gates}
        while gate.ident in existing_ids:
            gate.ident += 1
        self.safety_gates.append(gate)

    def clear_gate(self, ident):
        for gate in self.safety_gates:
            if gate.ident == ident:
                gate.active = False

    # ------------------------------------------------------------------
    # Navigation status helpers
    # ------------------------------------------------------------------
    def set_nav_status(self, status_enum):
        """Set navigation status using VesselNavStatus.NavigationStatus enum."""
        self.nav_status_value = status_enum

    def set_roc_entity(self, entity_id, state_enum):
        """Update or add a ROC entity."""
        for idx, (eid, _) in enumerate(self.roc_entities):
            if eid == entity_id:
                self.roc_entities[idx] = (entity_id, state_enum)
                return
        self.roc_entities.append((entity_id, state_enum))

    # ------------------------------------------------------------------
    # State / kinematics update
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        if new_state not in ["go", "no-go"]:
            raise ValueError("State must be 'go' or 'no-go'")
        self.state = new_state

    def update_speed(self, dt):
        """
        If in no-go state, reduce SOG slowly toward 0.
        dt: time delta in seconds
        """
        if self.state == "go":
            return  # keep current SOG

        decel_amount = self.decel_rate * dt
        self.sog_knots = max(0.0, self.sog_knots - decel_amount)

    def update_position(self, dt):
        """
        Update ship position based on COG/SOG with Earth curvature approximation.
        dt: time delta in seconds
        """
        speed_mps = self.sog_knots * 0.514444  # knots → m/s
        distance = speed_mps * dt

        cog_rad = math.radians(self.cog_deg)
        R = 6371000  # Earth radius (m)

        dlat = (distance * math.cos(cog_rad)) / R
        dlon = (distance * math.sin(cog_rad)) / (
            R * math.cos(math.radians(self.latitude))
        )

        self.latitude += math.degrees(dlat)
        self.longitude += math.degrees(dlon)

    # ------------------------------------------------------------------
    # Safety gate checks
    # ------------------------------------------------------------------
    def is_in_circle(self):
        """
        If inside any active gate, set state to 'no-go'.
        """
        R = 6371000  # Earth radius in meters

        for gate in self.safety_gates:
            if not gate.active:
                continue

            phi1 = math.radians(self.latitude)
            phi2 = math.radians(gate.latitude)
            dphi = math.radians(gate.latitude - self.latitude)
            dlambda = math.radians(gate.longitude - self.longitude)

            a = (
                math.sin(dphi / 2) ** 2
                + math.cos(phi1)
                * math.cos(phi2)
                * math.sin(dlambda / 2) ** 2
            )
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            dist = R * c

            if dist <= gate.radius:
                self.state = "no-go"

    def time_to_circle(self):
        R = 6371000  # meters

        best_gate_id = None
        best_time = None

        for gate in self.safety_gates:
            if not gate.active:
                continue

            # Convert to radians
            lat1 = math.radians(self.latitude)
            lon1 = math.radians(self.longitude)
            latc = math.radians(gate.latitude)
            lonc = math.radians(gate.longitude)

            # ENU coords
            dlat = latc - lat1
            dlon = lonc - lon1

            north = dlat * R
            east = dlon * R * math.cos(lat1)

            dx = east
            dy = north

            dist = math.hypot(dx, dy)

            if dist <= gate.radius:
                return gate.ident, 0.0

            # Velocity vector
            speed = self.sog_knots * 0.514444
            if speed <= 0:
                continue

            theta = math.radians(self.cog_deg)
            vx = speed * math.sin(theta)
            vy = speed * math.cos(theta)

            # Correct quadratic formulation
            A = vx*vx + vy*vy
            B = 2*(dx*vx + dy*vy)
            C = dx*dx + dy*dy - gate.radius*gate.radius

            discriminant = B*B - 4*A*C
            if discriminant < 0:
                continue  # No intersection

            sqrtD = math.sqrt(discriminant)
            t1 = (B - sqrtD) / (2*A)
            t2 = (B + sqrtD) / (2*A)

            print (t1, t2)

            # Only positive solutions matter
            times = [t for t in (t1, t2) if t > 0]

            if not times:
                continue

            t_hit = min(times)

            if best_time is None or t_hit < best_time:
                best_time = t_hit
                best_gate_id = gate.ident

        if best_time is None:
            return None

        return best_gate_id, best_time


    # ------------------------------------------------------------------
    # Zenoh subscribers for control / updates
    # ------------------------------------------------------------------
    def _start_zenoh_subscribers(self):
        # These control topics are just examples; adjust to your control plane as needed.
        self.zenoh.declare_subscriber(
            f"{self.name}/COG",
            lambda sample: self._update_cog(sample.payload),
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/SOG",
            lambda sample: self._update_sog(sample.payload),
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/state",
            lambda sample: self._update_state(sample.payload),
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/gates",
            lambda sample: self._update_gates(sample.payload),
        )

    def _update_cog(self, payload):
        print("Caught COG update")
        try:
            self.cog_deg = float(payload.to_string())
            print("Updating COG to:", self.cog_deg)
        except Exception:
            pass

    def _update_sog(self, payload):
        print("Caught SOG update")
        if self.state == "no-go":
            return
        try:
            self.sog_knots = max(0.0, float(payload.to_string()))
            print("Updating SOG to:", self.sog_knots)
        except Exception:
            pass

    def _update_state(self, payload):
        print("Caught go/no-go")
        try:
            new_state = payload.to_string().strip().lower()
            if new_state in ["go", "no-go"]:
                self.state = new_state
        except Exception:
            pass

    def _update_gates(self, payload):
        print("New gate update received (TODO: implement decoding)")
        # TODO: define serialized format for SafteyGate and decode here.
        # For now this is just a stub.

    # ------------------------------------------------------------------
    # Keelson publishing
    # ------------------------------------------------------------------
    def _publish_state(self):
        """
        Publish current state using Keelson well-known subjects, plus
        remote_status/remote_time as plain strings.
        """
        try:
            now_ns = time.time_ns()

            # location_fix (foxglove.LocationFix)
            loc = LocationFix()
            loc.timestamp.FromNanoseconds(now_ns)
            loc.latitude = float(self.latitude)
            loc.longitude = float(self.longitude)
            loc.altitude = 0.0
            loc.frame_id = self.name
            self.pub_location_fix.put(enclose(loc.SerializeToString()))

            # course_over_ground_deg (TimestampedFloat)
            cog_msg = TimestampedFloat()
            cog_msg.timestamp.FromNanoseconds(now_ns)
            cog_msg.value = float(self.cog_deg)
            self.pub_cog.put(enclose(cog_msg.SerializeToString()))

            # speed_over_ground_knots (TimestampedFloat)
            sog_msg = TimestampedFloat()
            sog_msg.timestamp.FromNanoseconds(now_ns)
            sog_msg.value = float(self.sog_knots)
            self.pub_sog.put(enclose(sog_msg.SerializeToString()))

            # name (TimestampedString)
            name_msg = TimestampedString()
            name_msg.timestamp.FromNanoseconds(now_ns)
            name_msg.value = self.name
            self.pub_name.put(enclose(name_msg.SerializeToString()))

            # mmsi_number (TimestampedInt)
            mmsi_msg = TimestampedInt()
            mmsi_msg.timestamp.FromNanoseconds(now_ns)
            mmsi_msg.value = int(self.mmsi_number)
            self.pub_mmsi.put(enclose(mmsi_msg.SerializeToString()))

            # imo_number (TimestampedInt)
            imo_msg = TimestampedInt()
            imo_msg.timestamp.FromNanoseconds(now_ns)
            imo_msg.value = int(self.imo_number)
            self.pub_imo.put(enclose(imo_msg.SerializeToString()))

            # nav_status (VesselNavStatus)
            nav_msg = VesselNavStatus()
            nav_msg.timestamp.FromNanoseconds(now_ns)
            nav_msg.navigation_status = self.nav_status_value
            self.pub_nav_status.put(enclose(nav_msg.SerializeToString()))

            # roc_status (ROCStatus)
            roc_msg = ROCStatus()
            roc_msg.timestamp.FromNanoseconds(now_ns)
            for entity_id, state_enum in self.roc_entities:
                e = roc_msg.entities.add()
                e.entity_id = entity_id
                e.state = state_enum
            self.pub_roc_status.put(enclose(roc_msg.SerializeToString()))

            # Remote status & remote time (plain Zenoh strings)
            tt = self.time_to_circle()
            remote_status = None
            remote_time_value = None

            if tt is None:
                remote_status = "Normal. No gates on path"
            else:
                gate_id, t_sec = tt
                if t_sec <= 0:
                    # Inside a safety gate
                    if self.state == "no-go":
                        remote_status = "Inside safety gate - Stopping"
                    else:
                        remote_status = "Inside safety gate - Passing"
                    # No remote_time for this case
                else:
                    remote_status = (
                        f"Normal. Time to next safety gate: {t_sec:.1f}"
                    )
                    remote_time_value = t_sec

            if remote_status is not None:
                self.pub_remote_status.put(remote_status)

            if remote_time_value is not None:
                self.pub_remote_time.put(f"{remote_time_value}")

        except Exception as e:
            print("Failure to push:", e)

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    def step(self, dt):
        """Perform a full update step: noise, speed, position, safety-gate logic, publish."""
        # Add small noise to SOG and COG
        self.sog_knots += random.gauss(0, 0.05)  # ~0.05 knot std dev
        self.cog_deg = (self.cog_deg + random.gauss(0, 0.2)) % 360  # ~0.2° std dev

        # Update dynamics
        self.update_speed(dt)
        self.update_position(dt)

        # Log history
        self.history_lat.append(self.latitude)
        self.history_lon.append(self.longitude)

        # Safety check
        self.is_in_circle()

        # Publish all data
        self._publish_state()

    def simulate(self, frequency_hz=1.0, duration_sec=10.0, live_plot=False):
        dt = 1.0 / frequency_hz
        steps = int(duration_sec / dt)

        if live_plot:
            plt.ion()
            fig, ax = plt.subplots(figsize=(8, 8))

        for _ in range(steps):
            self.step(dt)

            if live_plot:
                ax.clear()

                # Plot course history
                ax.plot(self.history_lon, self.history_lat, marker="o", linewidth=1, color="black")

                # Plot safety gates
                for gate in self.safety_gates:
                    color = "red" if gate.active else "blue"

                    # Convert radius (meters) to degrees latitude (approx)
                    radius_deg = gate.radius / 111_320.0  # meters per degree lat

                    circle = plt.Circle(
                        (gate.longitude, gate.latitude),
                        radius_deg,
                        fill=False,
                        color=color,
                        linewidth=2,
                        alpha=0.7,
                    )
                    ax.add_patch(circle)
                    ax.text(
                        gate.longitude,
                        gate.latitude,
                        f"Gate {gate.ident}",
                        fontsize=8,
                        ha="center",
                        va="center",
                        color=color,
                    )

                # Build status text box
                tt = self.time_to_circle()
                if tt is None:
                    remote_status = "Normal. No gates on path"
                else:
                    gid, tsec = tt
                    if tsec <= 0:
                        if self.state == "no-go":
                            remote_status = "Inside safety gate - Stopping"
                        else:
                            remote_status = "Inside safety gate - Passing"
                    else:
                        remote_status = f"Time to next safety gate: {tsec:.1f} sec"

                text = (
                    f"Vessel: {self.name}\n"
                    f"COG: {self.cog_deg:.1f}°\n"
                    f"SOG: {self.sog_knots:.2f} kn\n"
                    f"State: {self.state}\n"
                    f"Nav Status: {self.nav_status_value}\n"
                    f"{remote_status}"
                )

                ax.text(
                    0.02,
                    0.02,
                    text,
                    transform=ax.transAxes,
                    fontsize=10,
                    color="black",
                    bbox=dict(
                        facecolor="white",
                        alpha=0.6,
                        edgecolor="gray",
                        boxstyle="round,pad=0.3",
                    ),
                    verticalalignment="bottom",
                )

                ax.set_xlabel("Longitude")
                ax.set_ylabel("Latitude")
                ax.set_title(f"Live Course Plot for {self.name}")
                ax.grid(True)
                ax.set_aspect("equal", "box")

                # Autoscale plot so ship & gates stay visible
                all_lats = self.history_lat + [g.latitude for g in self.safety_gates]
                all_lons = self.history_lon + [g.longitude for g in self.safety_gates]

                if all_lats:
                    lat_min = min(all_lats)
                    lat_max = max(all_lats)
                    lon_min = min(all_lons)
                    lon_max = max(all_lons)
                    pad = 0.001
                    ax.set_xlim(lon_min - pad, lon_max + pad)
                    ax.set_ylim(lat_min - pad, lat_max + pad)

                plt.pause(0.001)

            time.sleep(dt)

        if live_plot:
            plt.close()


    def plot_course(self):
        plt.figure(figsize=(8, 8))
        plt.plot(self.history_lon, self.history_lat, marker="o", linewidth=1, color="black")

        # Plot safety gates
        for gate in self.safety_gates:
            color = "red" if gate.active else "blue"
            radius_deg = gate.radius / 111_320.0

            circle = plt.Circle(
                (gate.longitude, gate.latitude),
                radius_deg,
                fill=False,
                color=color,
                linewidth=2,
                alpha=0.7,
            )
            plt.gca().add_patch(circle)
            plt.text(
                gate.longitude,
                gate.latitude,
                f"Gate {gate.ident}",
                fontsize=8,
                ha="center",
                va="center",
                color=color,
            )

        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title(f"Course Plot for {self.name}")
        plt.grid(True)
        plt.gca().set_aspect("equal", "box")
        plt.show()

    def __repr__(self):
        return (
            f"Ship(name={self.name}, lat={self.latitude:.6f}, lon={self.longitude:.6f}, "
            f"COG={self.cog_deg:.2f}, SOG={self.sog_knots:.2f}, state={self.state})"
        )


# ---------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------
if __name__ == "__main__":
    ship = Ship(name="MASS_0")

    # Example initial kinematics
    ship.cog_deg = 45.0
    ship.sog_knots = 15.0

    # Example: add a safety gate ~1 km ahead
    gate = SafteyGate(latitude=ship.latitude + 0.001, longitude=ship.longitude + 0.001, radius=100.0, ident=1)
    ship.add_s_gate(gate)

    try:
        print("Starting simulation...")
        ship.simulate(frequency_hz=5, duration_sec=30, live_plot=True)

        print("Final ship state:")
        print(ship)

        if Ship._zenoh is not None:
            Ship._zenoh.close()
        sys.exit(0)

    except KeyboardInterrupt:
        print("Interrupted, shutting down...")
        if Ship._zenoh is not None:
            Ship._zenoh.close()
        sys.exit(0)
