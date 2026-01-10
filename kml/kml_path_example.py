#!/usr/bin/env python3
"""
kml_path_example.py

Example usage of kml_path.py for different scenarios
"""

from kml_path import (
    WaypointGenerator,
    DronePathController,
    PLAN_FILE,
    DEFAULT_SEPARATION_M,
)
import custom_survey as cs


def example_1_basic_mission():
    """
    Example 1: Basic survey mission with default parameters
    """
    print("\n" + "="*60)
    print("EXAMPLE 1: Basic Survey Mission")
    print("="*60)
    
    # Load polygon
    print("\nLoading survey area from mission.plan...")
    poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
        PLAN_FILE
    )
    
    # Split and generate paths
    print("Generating survey paths...")
    poly1_m, poly2_m, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
    
    angle1, path1, _, _, _ = cs.find_best_angle_for_region(
        poly1_m, DEFAULT_SEPARATION_M, (0, 0)
    )
    angle2, path2, _, _, _ = cs.find_best_angle_for_region(
        poly2_m, DEFAULT_SEPARATION_M, (0, 0)
    )
    
    if path1 is None or path2 is None:
        print("ERROR: Could not generate paths")
        return
    
    # Generate waypoints
    print("Creating waypoints...")
    gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    
    waypoints1 = gen1.generate_waypoints(spacing_m=5.0)
    waypoints2 = gen2.generate_waypoints(spacing_m=5.0)
    
    all_waypoints = waypoints1 + waypoints2
    
    print(f"\nGenerated {len(all_waypoints)} waypoints:")
    print(f"  Path 1: {len(waypoints1)} waypoints")
    print(f"  Path 2: {len(waypoints2)} waypoints")
    
    # Show first 3 and last 3 waypoints
    print("\nFirst 3 waypoints:")
    for i, (lat, lon, alt) in enumerate(all_waypoints[:3]):
        print(f"  {i+1}. {lat:.6f}, {lon:.6f} @ {alt:.1f}m")
    
    print("\nLast 3 waypoints:")
    for i, (lat, lon, alt) in enumerate(all_waypoints[-3:], start=len(all_waypoints)-2):
        print(f"  {i}. {lat:.6f}, {lon:.6f} @ {alt:.1f}m")


def example_2_custom_separation():
    """
    Example 2: Mission with custom line spacing
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: Custom Line Spacing")
    print("="*60)
    
    custom_spacing = 10.0  # 10 meter spacing instead of default 15m
    
    # Load polygon
    print(f"\nUsing {custom_spacing}m line spacing...")
    poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
        PLAN_FILE
    )
    
    minx, miny, maxx, maxy = poly_m.bounds
    min_dim = min(maxx - minx, maxy - miny)
    
    if custom_spacing >= min_dim:
        print(f"ERROR: Spacing {custom_spacing}m is larger than polygon dimension {min_dim:.1f}m")
        return
    
    # Generate paths with custom spacing
    poly1_m, poly2_m, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
    
    angle1, path1, survey_len1, _, _ = cs.find_best_angle_for_region(
        poly1_m, custom_spacing, (0, 0)
    )
    angle2, path2, survey_len2, _, _ = cs.find_best_angle_for_region(
        poly2_m, custom_spacing, (0, 0)
    )
    
    if path1 is None or path2 is None:
        print("ERROR: Could not generate paths")
        return
    
    # Generate waypoints with different spacing
    gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    
    waypoint_spacing = 3.0  # Denser waypoint spacing for finer control
    waypoints1 = gen1.generate_waypoints(spacing_m=waypoint_spacing)
    waypoints2 = gen2.generate_waypoints(spacing_m=waypoint_spacing)
    
    total_distance = survey_len1 + survey_len2
    total_waypoints = len(waypoints1) + len(waypoints2)
    
    print(f"\nSurvey parameters:")
    print(f"  Line spacing: {custom_spacing}m")
    print(f"  Waypoint spacing: {waypoint_spacing}m")
    print(f"  Total survey distance: {total_distance:.1f}m")
    print(f"  Total waypoints: {total_waypoints}")
    print(f"  Estimated flight time @ 8 m/s: {total_distance/8:.1f} minutes")


def example_3_waypoint_analysis():
    """
    Example 3: Detailed waypoint analysis
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: Waypoint Analysis")
    print("="*60)
    
    # Load polygon
    poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
        PLAN_FILE
    )
    
    # Generate paths
    poly1_m, poly2_m, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
    
    angle1, path1, _, _, _ = cs.find_best_angle_for_region(
        poly1_m, DEFAULT_SEPARATION_M, (0, 0)
    )
    angle2, path2, _, _, _ = cs.find_best_angle_for_region(
        poly2_m, DEFAULT_SEPARATION_M, (0, 0)
    )
    
    # Generate waypoints
    gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    waypoints = gen1.generate_waypoints(spacing_m=5.0)
    
    print(f"\nWaypoint analysis for path 1 ({len(waypoints)} waypoints):")
    
    # Calculate distances between consecutive waypoints
    print("\nDistance analysis:")
    distances = []
    for i in range(1, len(waypoints)):
        lat1, lon1, _ = waypoints[i-1]
        lat2, lon2, _ = waypoints[i]
        
        # Rough distance calculation (meters)
        dlat = (lat2 - lat1) * 111000  # ~111km per degree latitude
        dlon = (lon2 - lon1) * 111000 * (math.cos(math.radians(lat1)))  # Adjusted for latitude
        dist = (dlat**2 + dlon**2)**0.5
        distances.append(dist)
    
    if distances:
        print(f"  Min distance: {min(distances):.1f}m")
        print(f"  Max distance: {max(distances):.1f}m")
        print(f"  Mean distance: {sum(distances)/len(distances):.1f}m")
        print(f"  Total distance: {sum(distances):.1f}m")
    
    # Show coordinate bounds
    lats = [w[0] for w in waypoints]
    lons = [w[1] for w in waypoints]
    
    print(f"\nCoordinate bounds:")
    print(f"  Lat range: {min(lats):.6f} to {max(lats):.6f}")
    print(f"  Lon range: {min(lons):.6f} to {max(lons):.6f}")
    print(f"  Lat span: {(max(lats)-min(lats))*111:.1f}m")
    print(f"  Lon span: {(max(lons)-min(lons))*111*math.cos(math.radians(lat0)):.1f}m")


def example_4_export_waypoints():
    """
    Example 4: Export waypoints to JSON/CSV
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: Export Waypoints")
    print("="*60)
    
    import json
    import csv
    
    # Load polygon and generate waypoints
    poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
        PLAN_FILE
    )
    
    poly1_m, poly2_m, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
    
    angle1, path1, _, _, _ = cs.find_best_angle_for_region(
        poly1_m, DEFAULT_SEPARATION_M, (0, 0)
    )
    
    gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    waypoints = gen1.generate_waypoints(spacing_m=5.0)
    
    # Export to JSON
    waypoints_json = [
        {
            "index": i,
            "latitude": lat,
            "longitude": lon,
            "altitude_m": alt,
            "altitude_ft": alt * 3.28084
        }
        for i, (lat, lon, alt) in enumerate(waypoints)
    ]
    
    with open("waypoints.json", "w") as f:
        json.dump(waypoints_json, f, indent=2)
    
    print(f"\nExported {len(waypoints)} waypoints to waypoints.json")
    
    # Export to CSV
    with open("waypoints.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Index", "Latitude", "Longitude", "Altitude (m)", "Altitude (ft)"])
        for i, (lat, lon, alt) in enumerate(waypoints):
            writer.writerow([i, f"{lat:.8f}", f"{lon:.8f}", f"{alt:.2f}", f"{alt*3.28084:.2f}"])
    
    print(f"Exported {len(waypoints)} waypoints to waypoints.csv")


# Run examples
if __name__ == "__main__":
    import math
    
    print("\n" + "="*60)
    print("KML PATH EXAMPLES")
    print("="*60)
    
    try:
        example_1_basic_mission()
        example_2_custom_separation()
        example_3_waypoint_analysis()
        example_4_export_waypoints()
        
        print("\n" + "="*60)
        print("ALL EXAMPLES COMPLETED")
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"\nERROR in examples: {e}")
        import traceback
        traceback.print_exc()
