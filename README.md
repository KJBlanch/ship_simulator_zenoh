# MASS Vessel Simulator

A lightweight Python simulation of a Maritime Autonomous Surface Ship (MASS) that publishes navigation data using **Zenoh** and **Keelson** payload schemas. You can also control the simulated vessel in real time by publishing control messages to specific Zenoh topics.

---

## Features

* Publishes vessel GNSS, COG, SOG, navigation status, and ROC status using **Keelson datatypes**.
* Accepts live control commands:

  * Set course over ground (COG)
  * Set speed over ground (SOG)
  * Set vessel state (`go` / `no-go`)
  * Update safety gate completion
* Supports real-time simulation and optional live plotting of vessel trajectory.
* Includes safety-gate logic for stopping or slowing the vessel.
* Includes docker and docker-compose files for container deployment.

---

# One-line demo

_Requires docker and docker-compose_

```bash
docker-compose up
```

---

# 1. Installation

### Install dependencies

```bash
pip install eclipse-zenoh protobuf matplotlib keelson
```

---

# 2. Running the Simulation

```bash
python3 ship_sim.py
```

This will:

1. Start a Zenoh session
2. Create a vessel named **MASS_0**
3. Run a simulation loop at 5 Hz
4. Publish navigation + status messages
5. Accept incoming control messages

Enable live plotting:

```python
ship.simulate(frequency_hz=5, duration_sec=30, live_plot=True)
```

---

# 3. Published Topics (Receiving Data)

The simulator publishes data using **Keelson pub/sub keys** under:

```
rise/@v0/<VESSEL_NAME>/
```

## GNSS / Navigation

| Field                     | Zenoh Key                                        | Type                   |
| ------------------------- | ------------------------------------------------ | ---------------------- |
| Location Fix              | `rise/@v0/MASS_0/location_fix/gnss/0`            | `foxglove.LocationFix` |
| Course over ground (deg)  | `rise/@v0/MASS_0/course_over_ground_deg/gnss/0`  | `TimestampedFloat`     |
| Speed over ground (knots) | `rise/@v0/MASS_0/speed_over_ground_knots/gnss/0` | `TimestampedFloat`     |

## Registrar

| Field       | Key                                       |
| ----------- | ----------------------------------------- |
| Vessel Name | `rise/@v0/MASS_0/name/registrar/0`        |
| MMSI        | `rise/@v0/MASS_0/mmsi_number/registrar/0` |
| IMO         | `rise/@v0/MASS_0/imo_number/registrar/0`  |

## Bridge Status

| Field             | Key                                   | Message Type      |
| ----------------- | ------------------------------------- | ----------------- |
| Navigation Status | `rise/@v0/MASS_0/nav_status/bridge/0` | `VesselNavStatus` |
| ROC Entity Status | `rise/@v0/MASS_0/roc_status/bridge/0` | `ROCStatus`       |

## Remote Status (plain strings)

| Field               | Key                                             |
| ------------------- | ----------------------------------------------- |
| Remote Status       | `rise/@v0/MASS_0/pubsub/remote_status/bridge/0` |
| Time to safety gate | `rise/@v0/MASS_0/pubsub/remote_time/bridge/1`   |

---

# 4. Controlling the Ship (Sending Commands)

The simulator subscribes to these **simple Zenoh topics**:

| Action             | Topic          | Payload                      |
| ------------------ | -------------- | ---------------------------- |
| Set COG (deg)      | `MASS_0/COG`   | `TimestampedFloat`           |
| Set SOG (knots)    | `MASS_0/SOG`   | `TimestampedFloat`           |
| Set vessel state   | `MASS_0/state` | plain string `go` or `no-go` |
| Update safety gate | `MASS_0/gates` | plain string: `IDENT STATUS` |

---

# 5. Sending Commands

## 5.1 Set COG

```python
import zenoh
from keelson import enclose
from keelson.payloads.Primitives_pb2 import TimestampedFloat
import time

z = zenoh.open()
pub = z.declare_publisher("MASS_0/COG")

msg = TimestampedFloat()
msg.timestamp.FromNanoseconds(time.time_ns())
msg.value = 270.0  # turn west

pub.put(enclose(msg.SerializeToString()))
```

## 5.2 Set SOG

```python
pub = z.declare_publisher("MASS_0/SOG")
msg = TimestampedFloat()
msg.timestamp.FromNanoseconds(time.time_ns())
msg.value = 5.0
pub.put(enclose(msg.SerializeToString()))
```

## 5.3 Set State

```python
pub = z.declare_publisher("MASS_0/state")
pub.put("no-go")
```

## 5.4 Update Safety Gate Status

Format: `ident active_flag`

```python
pub = z.declare_publisher("MASS_0/gates")
pub.put("1 0")   # disable gate 1
```

---

# 6. Receiving Messages

Example subscriber:

```python
import zenoh

z = zenoh.open()

def callback(sample):
    print(sample.key_expr, sample.payload)

z.declare_subscriber("rise/@v0/MASS_0/location_fix/gnss/0", callback)

input("Press ENTER to exit\n")
```

---

# 7. Safety Gate Behavior

* If vessel enters a safety gate → state becomes `no-go`.
* Publishes human-readable status strings:

  * `Normal. Time to next safety gate: 8.4`
  * `Inside safety gate - Stopping`

---

# 8. Live Plotting

Displays:

* vessel trajectory
* active + inactive gates
* status info
* real-time kinematics

---

# 9. Interactive Workflow

1. Run simulator:

```bash
python3 ship_sim.py
```

2. Send commands in a separate terminal:

```bash
python3 ship_controller.py <command> <value>
```

---

# 10. Zenoh Shutdown

The simulator cleans up automatically:

```python
if Ship._zenoh is not None:
    Ship._zenoh.close()
```

---
