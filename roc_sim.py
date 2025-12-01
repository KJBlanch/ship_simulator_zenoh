import zenoh

cog_out = ""
sog_out = ""
lat = ""
lon = ""
state_out = ""

def listen_cog_out(sample):
    global cog_out
    cog_out = sample.payload

def listen_sog_out(sample):
    global sog_out
    sog_out = sample.payload

def listen_lat(sample):
    global lat
    lat = sample.payload

def listen_lon(sample):
    global lon
    lon = sample.payload

def listen_state_out(sample):
    global state_out
    state_out = sample.payload

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        session.declare_subscriber("testship/COG_out", listen_cog_out)
        session.declare_subscriber("testship/SOG_out", listen_sog_out)
        session.declare_subscriber("testship/lat", listen_lat)
        session.declare_subscriber("testship/lon", listen_lon)
        session.declare_subscriber("testship/state_out", listen_state_out)

        cog = session.declare_publisher("testship/COG")
        sog = session.declare_publisher("testship/SOG")
        state = session.declare_publisher("testship/state")

        while True:
            print("read/write: ", end="")
            match input():
                case "read":
                    print("COG_out/SOG_out/lat/lon/state_out: ", end="")
                    match input():
                        case "COG_out":
                            print(f"COG_out: {cog_out}")
                        case "SOG_out":
                            print(f"SOG_out: {sog_out}")
                        case "lat":
                            print(f"lat: {lat}")
                        case "lon":
                            print(f"lon: {lon}")
                        case "state_out":
                            print(f"state_out: {state_out}")
                case "write":
                    print("COG/SOG/state: ", end="")
                    match input():
                        case "COG":
                            print("COG: ", end="")
                            cog.put(input())
                        case "SOG":
                            print("SOG: ", end="")
                            sog.put(input())
                        case "state":
                            print("state: ", end="")
                            state.put(input())
