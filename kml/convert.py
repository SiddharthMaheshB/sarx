import json
from lxml import etree

KML_FILE = "generated_polygons/map.kml"
OUTPUT_PLAN = "mission.plan"
DEFAULT_ALTITUDE = 20.0


# ==========================
# KML COORDINATE EXTRACTION
# ==========================
def extract_coordinates(kml_file):
    """
    Robust KML coordinate extraction.
    Supports:
    - LineString
    - Point
    - gx:Track
    Returns list of (lat, lon, alt)
    """

    with open(kml_file, "rb") as f:
        tree = etree.parse(f)

    root = tree.getroot()
    coords = []

    # ---- LineString & Point (no namespace assumption) ----
    for elem in root.findall(".//{*}coordinates"):
        if elem.text is None:
            continue

        raw = elem.text.strip().replace("\n", " ").split()
        for item in raw:
            parts = item.split(",")
            if len(parts) < 2:
                continue

            lon = float(parts[0])
            lat = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 else DEFAULT_ALTITUDE

            coords.append((lat, lon, alt))

    # ---- gx:Track support ----
    for elem in root.findall(".//{*}coord"):
        parts = elem.text.strip().split()
        if len(parts) < 2:
            continue

        lon = float(parts[0])
        lat = float(parts[1])
        alt = float(parts[2]) if len(parts) > 2 else DEFAULT_ALTITUDE

        coords.append((lat, lon, alt))

    if not coords:
        raise RuntimeError("❌ No coordinates found in KML file")

    return coords


# ==========================
# QGROUNDCONTROL PLAN BUILDER
# ==========================
def build_qgc_plan(coords):
    items = []

    for i, (lat, lon, alt) in enumerate(coords):
        items.append({
            "autoContinue": True,
            "command": 16,          # MAV_CMD_NAV_WAYPOINT
            "doJumpId": i + 1,
            "frame": 3,             # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [
                0, 0, 0, float("nan"),
                lat, lon, alt
            ],
            "type": "SimpleItem"
        })
    
    polygon = []
    for i, (lat,lon,alt) in enumerate(coords):
        if i == 0  or i == len(coords)-1:
            continue
        polygon.append([
            lat,lon
        ])
    
    plan = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "mission": {
            "version": 2,
            "firmwareType": 3,
            "vehicleType":13,
            "cruiseSpeed": 15,
            "hoverSpeed": 5,
            "plannedHomePosition": [
                coords[0][0],
                coords[0][1],
                coords[0][2]
            ],
            "globalPlanAltitudeMode": 1,
            "items": items
        },
        "geoFence": {
            "version": 2,
            "circles": [],
            "polygons": [
                {
                "inclusion" : True , 
                "polygon" : polygon
                }
            ],
            "version" : 1
        },
        "rallyPoints": {
            "version": 2,
            "points": []
        }
    }

    return plan


# ==========================
# MAIN
# ==========================
def main():
    coords = extract_coordinates(KML_FILE)
    for coord in coords:
        print(coord)
    plan = build_qgc_plan(coords)

    with open(OUTPUT_PLAN, "w") as f:
        json.dump(plan, f, indent=4)

    print(f"✔ Extracted {len(coords)} coordinates")
    print(f"✔ QGroundControl plan saved as {OUTPUT_PLAN}")


if __name__ == "__main__":
    main()
