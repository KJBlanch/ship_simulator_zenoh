import math
import asyncio
import zenoh as Zenoh
from zenoh import Session, Config
import time
import random
import sys

import matplotlib.pyplot as plt

class Ship:
    # Zenoh session shared among all ships
    _zenoh = None

    @staticmethod
    def init_zenoh():
        if Ship._zenoh is None:
            cfg = Config()
            Ship._zenoh = Zenoh.open(cfg)
        return Ship._zenoh
    def __init__(self, name, latitude=30.0, longitude=-40.0, cog_deg=0.0, sog_knots=10.0, tonnage=5000, length_m=120):  # Defaults in mid-Atlantic
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.cog_deg = cog_deg  # Course over ground in degrees
        self.sog_knots = sog_knots  # Speed over ground in knots
        self.tonnage = tonnage
        self.length_m = length_m
        
        # Track course history
        self.history_lat = [latitude]
        self.history_lon = [longitude]
        self.state = "go"  # 'go' or 'no-go' 

        # Derived deceleration factor (simple model)
        # Larger ships reduce speed slower
        self.decel_rate = max(0.01, min(0.5, 5000 / (self.tonnage + 1)))

        # Zenoh initialization
        self.zenoh = Ship.init_zenoh()
        self._start_zenoh_subscribers()

    def set_state(self, new_state):
        if new_state not in ["go", "no-go"]:
            raise ValueError("State must be 'go' or 'no-go'")
        self.state = new_state

    def update_speed(self, dt):
        """
        If in no-go state, reduce SOG slowly toward 0
        dt: time delta in seconds
        """
        if self.state == "go":
            return  # keep current SOG

        # Apply rate-limited deceleration
        decel_amount = self.decel_rate * dt
        self.sog_knots = max(0.0, self.sog_knots - decel_amount)

    def update_position(self, dt):
        """
        Update ship position based on COG/SOG with Earth curvature approximation.
        dt: time delta in seconds
        """
        # Convert SOG from knots to m/s
        speed_mps = self.sog_knots * 0.514444

        # Distance traveled in this step (meters)
        distance = speed_mps * dt

        # Convert course to radians
        cog_rad = math.radians(self.cog_deg)

        # Earth radius in meters
        R = 6371000

        # Compute new lat/lon
        dlat = (distance * math.cos(cog_rad)) / R
        dlon = (distance * math.sin(cog_rad)) / (R * math.cos(math.radians(self.latitude)))

        # Update state
        self.latitude += math.degrees(dlat)
        self.longitude += math.degrees(dlon)

    def _start_zenoh_subscribers(self):
        # Subscribe to COG
        self.zenoh.declare_subscriber(f"testship/COG", lambda sample: self._update_cog(sample.payload))
        # Subscribe to SOG
        self.zenoh.declare_subscriber(f"testship/SOG", lambda sample: self._update_sog(sample.payload))
        # Subscribe to state (go / no-go)
        self.zenoh.declare_subscriber(f"testship/state", lambda sample: self._update_state(sample.payload))
     
    def _update_cog(self, payload):
        print("Caught COG update")
        print(payload)
        try:
            self.cog_deg = float(payload.to_string())
            print("Updating COG to:", self.cog_deg)
        except:
            pass

    def _update_sog(self, payload):
        print("Caught SOG update")
        # Ignore SOG updates if in no-go state
        if self.state == "no-go":
            return
        try:
            self.sog_knots = max(0.0, float(payload.to_string()))
            print("Updating SOG to:", self.sog_knots)
        except:
            pass

    def _update_state(self, payload):
        print("Caught go/no-go")
        try:
            new_state = payload.to_string().strip().lower()
            if new_state in ["go", "no-go"]:
                self.state = new_state
        except:
            pass

    def _publish_state(self):
        try:
            self.zenoh.put(f"testship/state_out", self.state)
            self.zenoh.put(f"testship/lat", str(self.latitude))
            self.zenoh.put(f"testship/lon", str(self.longitude))
            self.zenoh.put(f"testship/COG_out", str(self.cog_deg))
            self.zenoh.put(f"testship/SOG_out", str(self.sog_knots))
        except:
            print("Failure to push")
            pass

    def step(self, dt):
        # Main update loop: noise, speed, position, publish
        self.sog_knots += random.gauss(0, 0.05)
        self.cog_deg = (self.cog_deg + random.gauss(0, 0.2)) % 360

        self.update_speed(dt)
        self.update_position(dt)
        self._publish_state()
        """Perform a full update step: speed + position."""
        # Add small noise to SOG and COG
        self.sog_knots += random.gauss(0, 0.05)  # ~0.05 knot std dev
        self.cog_deg = (self.cog_deg + random.gauss(0, 0.2)) % 360  # ~0.2° std dev

        self.update_speed(dt)
        # Log history
        self.history_lat.append(self.latitude)
        self.history_lon.append(self.longitude)

        self.update_position(dt)

    def simulate(self, frequency_hz=1.0, duration_sec=10.0, live_plot=False):
        """
        Run a simulation loop at the given update frequency.
        frequency_hz: update rate (0.1 to 100 Hz)
        duration_sec: simulation run time
        """
        dt = 1.0 / frequency_hz
        steps = int(duration_sec / dt)

        if live_plot:
            plt.ion()
            fig, ax = plt.subplots()

        for _ in range(steps):
                self.step(dt)
                if live_plot:
                    ax.clear()
                    ax.plot(self.history_lon, self.history_lat, marker='o', linewidth=1)
                    ax.set_xlabel('Longitude')
                    ax.set_ylabel('Latitude')
                    ax.set_title(f'Live Course Plot for {self.name}')
                    ax.grid(True)
                    ax.set_aspect('equal', 'box')
                    ax.ticklabel_format(useOffset=False)
                    plt.pause(0.001)

                time.sleep(dt)


        plt.close()

    def plot_course(self):
        """Plot the ship's course using matplotlib."""
        plt.figure()
        plt.plot(self.history_lon, self.history_lat, marker='o', linewidth=1)
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title(f'Course Plot for {self.name}')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def __repr__(self):
        return (f"Ship(name={self.name}, lat={self.latitude:.6f}, lon={self.longitude:.6f}, "
                f"COG={self.cog_deg}, SOG={self.sog_knots:.2f}, state={self.state})")

if __name__ == "__main__":
    # Create a ship with defaults (mid-Atlantic)
    ship = Ship(name="TestVessel")

    # Example: Set course east, speed 15 knots
    ship.cog_deg = 45
    ship.sog_knots = 15

    # Run simulation for 30 seconds at 5 Hz with live plotting
    try:
        print("Starting simulation...")
        ship.simulate(frequency_hz=5, duration_sec=30, live_plot=True)

        print("Final ship state:")
        print(ship)

        # Show final path
        # ship.plot_course() - TODO, fix this. 
        ship._zenoh.close()
        sys.exit()
    except KeyboardInterrupt:
        print("Breaking...")
        ship._zenoh.close()
        sys.exit()