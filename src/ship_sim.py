#!/usr/bin/env python3
import math
import random
import sys
import time
import tokenize

import matplotlib.pyplot as plt
import zenoh as Zenoh
from zenoh import Session, Config

# Keelson imports
import keelson
from keelson import uncover, enclose, construct_pubsub_key

from keelson.payloads.Primitives_pb2 import (
    TimestampedFloat,
    TimestampedInt,
    TimestampedString,
)

from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.payloads.VesselNavStatus_pb2 import VesselNavStatus
from keelson.payloads.ROCStatus_pb2 import ROCStatus


# ---------------------------------------------------------------------
# Safety Gate
# ---------------------------------------------------------------------
class SafteyGate:
    def __init__(self, latitude, longitude, radius, ident=0, active=True):
        self.latitude = latitude
        self.longitude = longitude
        self.radius = radius
        self.active = active
        self.ident = ident


# ---------------------------------------------------------------------
# Ship class with ROC handover support
# ---------------------------------------------------------------------
class Ship:
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
        # Basic identity
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.cog_deg = cog_deg
        self.sog_knots = sog_knots
        self.tonnage = tonnage
        self.length_m = length_m
        self.controller_id = controller_id

        self.mmsi_number = mmsi_number
        self.imo_number = imo_number

        # Navigation status
        self.nav_status_value = VesselNavStatus.NavigationStatus.UNDER_WAY

        # ======================================================
        # ROC CONTROL + PRIORITY + HANDOVER STATE
        # ======================================================
        self.priority_roc_id = f"ROC_{controller_id}"
        self.current_roc_id = self.priority_roc_id

        self.roc_entities = [
            (self.current_roc_id, ROCStatus.ROCEntity.State.MONITORING, True)
        ]

        self.handover_active = False
        self.handover_gate_id = None
        self.handover_broadcast_sent = False
        self.handover_relinquish_from = None
        self.handover_takeover_from = None

        self.state = "go"

        self.history_lat = [latitude]
        self.history_lon = [longitude]

        self.decel_rate = max(0.01, min(0.5, 5000 / (self.tonnage + 1)))

        # Zenoh init
        self.zenoh = Ship.init_zenoh()
        self._start_zenoh_subscribers()

        # Safety gates
        self.safety_gates = []

        self._base_path = "rise/@v0"

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pub_location_fix = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "location_fix", "gnss/0")
        )
        self.pub_cog = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "course_over_ground_deg", "gnss/0")
        )
        self.pub_sog = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "speed_over_ground_knots", "gnss/0")
        )
        self.pub_name = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "name", "registrar/0")
        )
        self.pub_mmsi = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "mmsi_number", "registrar/0")
        )
        self.pub_imo = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "imo_number", "registrar/0")
        )
        self.pub_nav_status = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "nav_status", "bridge/0")
        )
        self.pub_roc_status = self.zenoh.declare_publisher(
            construct_pubsub_key(self._base_path, self.name, "roc_status", "bridge/0")
        )

        # Remote status
        self.pub_remote_status = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/pubsub/remote_status/bridge/0"
        )
        self.pub_remote_time = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/pubsub/remote_time/bridge/1"
        )

        # NEW HANDOVER PUBLISHERS
        self.pub_handover_request = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/handover/request"
        )
        self.pub_handover_state = self.zenoh.declare_publisher(
            f"{self._base_path}/{self.name}/handover/state"
        )


    # ------------------------------------------------------------------
    # Safety gate tools
    # ------------------------------------------------------------------
    def add_s_gate(self, gate: SafteyGate):
        ids = {g.ident for g in self.safety_gates}
        while gate.ident in ids:
            gate.ident += 1
        self.safety_gates.append(gate)

    def clear_gate(self, ident):
        for g in self.safety_gates:
            if g.ident == ident:
                g.active = False


    # ------------------------------------------------------------------
    # Zenoh Subscriber Setup
    # ------------------------------------------------------------------
    def _start_zenoh_subscribers(self):
        # Legacy control
        self.zenoh.declare_subscriber(
            f"{self.name}/COG",
            lambda s: self._update_cog_legacy(s.payload)
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/SOG",
            lambda s: self._update_sog_legacy(s.payload)
        )

        # ROC-aware control
        self.zenoh.declare_subscriber(
            f"{self.name}/control/roc/*/COG",
            self._update_cog
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/control/roc/*/SOG",
            self._update_sog
        )

        # Handover events
        self.zenoh.declare_subscriber(
            f"{self.name}/handover/relinquish",
            self._on_roc_relinquish
        )
        self.zenoh.declare_subscriber(
            f"{self.name}/handover/takeover",
            self._on_roc_takeover
        )


    # ------------------------------------------------------------------
    # Decode helper
    # ------------------------------------------------------------------
    def _decode(self, sample, cls):
        try:
            _, _, payload = uncover(sample.to_bytes())
            return cls.FromString(payload)
        except Exception:
            try:
                _, _, payload = uncover(sample)
                return cls.FromString(payload)
            except:
                return None


    # ============================================================
    # LEGACY CONTROL (treated as priority ROC)
    # ============================================================
    def _update_cog_legacy(self, payload):
        msg = self._decode(payload, TimestampedFloat)
        if msg:
            self.cog_deg = msg.value

    def _update_sog_legacy(self, payload):
        if self.state == "no-go":
            return
        msg = self._decode(payload, TimestampedFloat)
        if msg:
            self.sog_knots = msg.value


    # ============================================================
    # ROC-AWARE CONTROL
    # ============================================================
    def _extract_roc_id_from_key(self, key_expr, suffix):
        key = str(key_expr)
        parts = key.split("/")
        if len(parts) >= 2 and parts[-1] == suffix:
            return parts[-2]
        return None


    def _update_cog(self, sample):
        roc_id = self._extract_roc_id_from_key(sample.key_expr, "COG")
        if roc_id != self.priority_roc_id:
            print(f"[IGNORE] COG from non-priority ROC {roc_id}")
            return
        msg = self._decode(sample.payload, TimestampedFloat)
        if msg:
            self.cog_deg = msg.value


    def _update_sog(self, sample):
        roc_id = self._extract_roc_id_from_key(sample.key_expr, "SOG")
        if roc_id != self.priority_roc_id:
            print(f"[IGNORE] SOG from non-priority ROC {roc_id}")
            return
        if self.state == "no-go":
            return
        msg = self._decode(sample.payload, TimestampedFloat)
        if msg:
            self.sog_knots = msg.value


    # ============================================================
    # HANDOVER PROTOCOL IMPLEMENTATION
    # ============================================================
    def _on_roc_relinquish(self, sample):
        """ROC currently in control announces readiness to relinquish."""
        try:
            roc_id = sample.payload.to_string().strip()
        except:
            print("[ERROR] Could not parse relinquish payload")
            return

        print(f"[HANDOVER] Relinquish received from {roc_id}")

        if not self.handover_active:
            print("[HANDOVER] Ignored: no active handover")
            return

        if roc_id != self.current_roc_id:
            print(f"[HANDOVER] Ignored relinquish from non-current ROC {roc_id}")
            return

        self.handover_relinquish_from = roc_id
        self._maybe_complete_handover()


    def _on_roc_takeover(self, sample):
        """ROC wanting to assume control."""
        try:
            roc_id = sample.payload.to_string().strip()
        except:
            print("[ERROR] Could not parse takeover payload")
            return

        print(f"[HANDOVER] Takeover received from {roc_id}")

        if not self.handover_active:
            print("[HANDOVER] Ignored: no active handover")
            return

        if roc_id == self.current_roc_id:
            print("[HANDOVER] Can't takeover: ROC is already in control")
            return

        self.handover_takeover_from = roc_id

        # Ensure ROC exists in registry
        if not any(eid == roc_id for eid, _, _ in self.roc_entities):
            self.roc_entities.append((roc_id, ROCStatus.ROCEntity.State.MONITORING, False))

        self._maybe_complete_handover()


    def _maybe_start_handover(self, tt):
        """Trigger handover request when ship is within 15 minutes of gate."""
        if tt is None:
            return

        gate_id, t_sec = tt
        if t_sec <= 0:
            return

        if t_sec <= 900 and not self.handover_broadcast_sent:
            # Activate handover state
            self.handover_active = True
            self.handover_gate_id = gate_id
            self.handover_broadcast_sent = True

            msg = (
                f"READY_FOR_HANDOVER gate_id={gate_id} "
                f"time_to_gate={t_sec:.1f} "
                f"current_roc={self.current_roc_id}"
            )

            print(f"[HANDOVER] Broadcasting request: {msg}")
            self.pub_handover_request.put(msg)


    def _maybe_complete_handover(self):
        """If relinquish + takeover have both been received, complete handover."""
        if not self.handover_active:
            return
        if not self.handover_relinquish_from:
            return
        if not self.handover_takeover_from:
            return

        new_roc = self.handover_takeover_from
        old_roc = self.current_roc_id
        gate_id = self.handover_gate_id

        print(f"[HANDOVER] Completing handover {old_roc} → {new_roc}")

        # Flip ROC priority
        self.priority_roc_id = new_roc
        self.current_roc_id = new_roc

        updated = []
        for eid, st, _prio in self.roc_entities:
            updated.append((eid, st, eid == new_roc))
        self.roc_entities = updated

        # Deactivate safety gate
        if gate_id is not None:
            self.clear_gate(gate_id)
            print(f"[HANDOVER] Safety gate {gate_id} deactivated")

        # Publish success
        self.pub_handover_state.put(
            f"HANDOVER_COMPLETED new_priority={new_roc} gate={gate_id}"
        )

        # Reset state
        self.handover_active = False
        self.handover_broadcast_sent = False
        self.handover_relinquish_from = None
        self.handover_takeover_from = None
        self.handover_gate_id = None


    # ------------------------------------------------------------------
    # Kinematics & Navigation Updates
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        if new_state not in ["go", "no-go"]:
            raise ValueError("Invalid state. Must be 'go' or 'no-go'.")
        self.state = new_state


    def update_speed(self, dt):
        if self.state == "go":
            return
        self.sog_knots = max(0.0, self.sog_knots - self.decel_rate * dt)


    def update_position(self, dt):
        speed_mps = self.sog_knots * 0.514444
        distance = speed_mps * dt
        angle = math.radians(self.cog_deg)

        R = 6371000
        dlat = (distance * math.cos(angle)) / R
        dlon = (distance * math.sin(angle)) / (R * math.cos(math.radians(self.latitude)))

        self.latitude += math.degrees(dlat)
        self.longitude += math.degrees(dlon)


    # ------------------------------------------------------------------
    # Safety Gate Logic
    # ------------------------------------------------------------------
    def is_in_circle(self):
        R = 6371000
        for gate in self.safety_gates:
            if not gate.active:
                continue

            phi1 = math.radians(self.latitude)
            phi2 = math.radians(gate.latitude)
            dphi = phi2 - phi1
            dlambda = math.radians(gate.longitude - self.longitude)

            a = (
                math.sin(dphi / 2)**2
                + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
            )
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            dist = R * c

            if dist <= gate.radius:
                self.state = "no-go"


    def time_to_circle(self):
        R = 6371000
        best_gate = None
        best_t = None

        for gate in self.safety_gates:
            if not gate.active:
                continue

            lat1 = math.radians(self.latitude)
            lon1 = math.radians(self.longitude)
            lat2 = math.radians(gate.latitude)
            lon2 = math.radians(gate.longitude)

            dx = (lon2 - lon1) * R * math.cos(lat1)
            dy = (lat2 - lat1) * R
            dist = math.hypot(dx, dy)

            if dist <= gate.radius:
                return gate.ident, 0

            speed = self.sog_knots * 0.514444
            if speed <= 0:
                continue

            theta = math.radians(self.cog_deg)
            vx = speed * math.sin(theta)
            vy = speed * math.cos(theta)

            A = vx*vx + vy*vy
            B = 2*(dx*vx + dy*vy)
            C = dx*dx + dy*dy - gate.radius*gate.radius

            D = B*B - 4*A*C
            if D < 0:
                continue

            t1 = (B - math.sqrt(D)) / (2*A)
            t2 = (B + math.sqrt(D)) / (2*A)

            times = [t for t in (t1, t2) if t > 0]
            if not times:
                continue

            t_hit = min(times)
            if best_t is None or t_hit < best_t:
                best_t = t_hit
                best_gate = gate.ident

        if best_gate is None:
            return None
        return best_gate, best_t

    # ------------------------------------------------------------------
    # Publishing Ship State (Keelson message outputs)
    # ------------------------------------------------------------------
    def _publish_state(self):
        try:
            now_ns = time.time_ns()

            # ------------------------------------------------------------------
            # LocationFix (GNSS)
            # ------------------------------------------------------------------
            loc = LocationFix()
            loc.timestamp.FromNanoseconds(now_ns)
            loc.latitude = float(self.latitude)
            loc.longitude = float(self.longitude)
            loc.altitude = 0.0
            loc.frame_id = self.name
            self.pub_location_fix.put(enclose(loc.SerializeToString()))

            # ------------------------------------------------------------------
            # Course over ground
            # ------------------------------------------------------------------
            cog_msg = TimestampedFloat()
            cog_msg.timestamp.FromNanoseconds(now_ns)
            cog_msg.value = float(self.cog_deg)
            self.pub_cog.put(enclose(cog_msg.SerializeToString()))

            # ------------------------------------------------------------------
            # Speed over ground
            # ------------------------------------------------------------------
            sog_msg = TimestampedFloat()
            sog_msg.timestamp.FromNanoseconds(now_ns)
            sog_msg.value = float(self.sog_knots)
            self.pub_sog.put(enclose(sog_msg.SerializeToString()))

            # ------------------------------------------------------------------
            # Registrar info
            # ------------------------------------------------------------------
            name_msg = TimestampedString()
            name_msg.timestamp.FromNanoseconds(now_ns)
            name_msg.value = self.name
            self.pub_name.put(enclose(name_msg.SerializeToString()))

            mmsi_msg = TimestampedInt()
            mmsi_msg.timestamp.FromNanoseconds(now_ns)
            mmsi_msg.value = int(self.mmsi_number)
            self.pub_mmsi.put(enclose(mmsi_msg.SerializeToString()))

            imo_msg = TimestampedInt()
            imo_msg.timestamp.FromNanoseconds(now_ns)
            imo_msg.value = int(self.imo_number)
            self.pub_imo.put(enclose(imo_msg.SerializeToString()))

            # ------------------------------------------------------------------
            # Navigation status
            # ------------------------------------------------------------------
            nav_msg = VesselNavStatus()
            nav_msg.timestamp.FromNanoseconds(now_ns)
            nav_msg.navigation_status = self.nav_status_value
            self.pub_nav_status.put(enclose(nav_msg.SerializeToString()))

            # ------------------------------------------------------------------
            # ROC status
            # ------------------------------------------------------------------
            roc_msg = ROCStatus()
            roc_msg.timestamp.FromNanoseconds(now_ns)
            for entity_id, state_enum, priority in self.roc_entities:
                e = roc_msg.entities.add()
                e.entity_id = entity_id
                e.state = state_enum
                # If you later add a has_priority field, it can be set here.
            self.pub_roc_status.put(enclose(roc_msg.SerializeToString()))

            # ------------------------------------------------------------------
            # Safety gate timing + Handover start check
            # ------------------------------------------------------------------
            tt = self.time_to_circle()
            self._maybe_start_handover(tt)

            # Human-readable remote status messages
            if tt is None:
                self.pub_remote_status.put("Normal. No gates on path")
            else:
                gate, t = tt
                if t <= 0:
                    msg = f"Inside safety gate {gate}"
                else:
                    msg = f"Time to gate {gate}: {t:.1f}s"
                    self.pub_remote_time.put(f"{t:.1f}")
                self.pub_remote_status.put(msg)

        except Exception as e:
            print("[ERROR] Failed to publish:", e)


    # ------------------------------------------------------------------
    # SIMULATION STEP
    # ------------------------------------------------------------------
    def step(self, dt):
        # Add small noise
        self.sog_knots += random.gauss(0, 0.05)
        self.cog_deg = (self.cog_deg + random.gauss(0, 0.2)) % 360

        # Dynamics
        self.update_speed(dt)
        self.update_position(dt)

        # Append history
        self.history_lat.append(self.latitude)
        self.history_lon.append(self.longitude)

        # Safety gate logic
        self.is_in_circle()

        # Publish state
        self._publish_state()


    # ------------------------------------------------------------------
    # High-level simulation loop
    # ------------------------------------------------------------------
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

                ax.plot(self.history_lon, self.history_lat, marker="o", linewidth=1, color="black")

                # Safety gates
                for gate in self.safety_gates:
                    color = "red" if gate.active else "blue"
                    rad_deg = gate.radius / 111320.0
                    circle = plt.Circle((gate.longitude, gate.latitude), rad_deg,
                                        fill=False, color=color, linewidth=2)
                    ax.add_patch(circle)
                    ax.text(gate.longitude, gate.latitude, f"Gate {gate.ident}",
                            color=color, ha="center")

                ax.set_xlabel("Longitude")
                ax.set_ylabel("Latitude")
                ax.set_title(f"Live Course for {self.name}")
                ax.grid(True)
                ax.set_aspect("equal")

                plt.pause(0.001)

            time.sleep(dt)

        if live_plot:
            plt.close()


    # ------------------------------------------------------------------
    # Plot result post-simulation
    # ------------------------------------------------------------------
    def plot_course(self):
        plt.figure(figsize=(8, 8))
        plt.plot(self.history_lon, self.history_lat, marker="o", linewidth=1)

        for gate in self.safety_gates:
            color = "red" if gate.active else "blue"
            rad_deg = gate.radius / 111320.0
            circle = plt.Circle((gate.longitude, gate.latitude), rad_deg,
                                fill=False, color=color, linewidth=2)
            plt.gca().add_patch(circle)
            plt.text(gate.longitude, gate.latitude, f"Gate {gate.ident}",
                     color=color, ha="center")

        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title(f"Course for {self.name}")
        plt.grid(True)
        plt.gca().set_aspect("equal")
        plt.show()


    # ------------------------------------------------------------------
    # String representation
    # ------------------------------------------------------------------
    def __repr__(self):
        return (f"Ship(name={self.name}, lat={self.latitude:.6f}, "
                f"lon={self.longitude:.6f}, COG={self.cog_deg:.2f}, "
                f"SOG={self.sog_knots:.2f}, state={self.state}, "
                f"priority_roc={self.priority_roc_id})")


# ---------------------------------------------------------------------
# MAIN ENTRY POINT
# ---------------------------------------------------------------------
if __name__ == "__main__":
    ship = Ship(name="MASS_0", controller_id=1)

    ship.cog_deg = 45.0
    ship.sog_knots = 15.0

    # Add a safety gate ~1 km ahead
    gate = SafteyGate(
        latitude=ship.latitude + 0.001,
        longitude=ship.longitude + 0.001,
        radius=100.0,
        ident=1
    )
    ship.add_s_gate(gate)

    try:
        print("Starting simulation...")
        ship.simulate(frequency_hz=5, duration_sec=30, live_plot=True)
        print(ship)

        if Ship._zenoh is not None:
            Ship._zenoh.close()

    except KeyboardInterrupt:
        print("Interrupted.")
        if Ship._zenoh is not None:
            Ship._zenoh.close()

