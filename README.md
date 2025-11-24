# ship_simulator_zenoh

A very basic ship simulator that contains lat/lon, cog and sog, as well as a state 'go' and 'no-go'. Interfaces with zenoh. 

- To change COG, a string containing a number (0-360) can be sent to testship/COG. 
- To change SOG, a string containing a number can be sent to testship/SOG. 
- To set go/no-go, a string containing the string "go" or "no-go" can be sent to testship/state

Setting no-go will tell the ship to slow down and stop. Setting go will tell it to accelerate up to prior SOG. 

Vessel state is constantly published to the following topics, using the following section:

 - self.zenoh.put(f"testship/state_out", self.state)
 - self.zenoh.put(f"testship/lat", str(self.latitude)) 
 - self.zenoh.put(f"testship/lon", str(self.longitude))
 - self.zenoh.put(f"testship/COG_out", str(self.cog_deg))
 - self.zenoh.put(f"testship/SOG_out", str(self.sog_knots))
