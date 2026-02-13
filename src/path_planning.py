#!/usr/bin/env python3
"""
Path planning module for SARX system.
Handles survey polygon loading and path generation.
"""

import sys
import traceback
sys.path.insert(0, "/home/drone/Desktop/kml")
import completesurvey as cs

from waypoints import WaypointGenerator
from config import (
    PLAN_FILE, DEFAULT_SEPARATION_M, WAYPOINT_RADIUS, 
    WAYPOINT_SPACING, SURVEY_ALT, RTL_ALTITUDE_M
)


def load_survey_polygon():
    """
    Load polygon from mission.plan file.
    
    Returns:
        tuple: (poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon) or (None, None, None, None, None) on error
    """
    print("\n[SURVEY] Loading polygon from mission.plan...")
    try:
        poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
            PLAN_FILE
        )
        print(f"  Polygon loaded: {poly_m.area:.1f} m²")
        print(f"  Reference: lat0={lat0:.6f}, lon0={lon0:.6f}")
        return poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon
    except Exception as e:
        print(f"ERROR loading polygon: {e}")
        return None, None, None, None, None


async def rtl_at_height(drone):
    """
    Commands the drone to first go to a specific altitude,
    then initiate RTL.
    
    Args:
        drone: MAVSDK drone object
    """
    from mavsdk.offboard import PositionNedYaw
    import asyncio
    
    print(f"[RTL] Adjusting altitude to {RTL_ALTITUDE_M}m before RTL...")

    # NED frame: Down is positive, Up is negative
    ned_down = -RTL_ALTITUDE_M

    # Hold current X/Y, only change altitude
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, ned_down, 0.0)
    )

    # Wait until altitude is reached
    await asyncio.sleep(4)

    print("[RTL] Initiating Return to Launch")
    await drone.action.return_to_launch()


def generate_survey_paths(poly_m, separation_m):
    """
    Generate survey paths for two polygon halves.
    
    Args:
        poly_m: Shapely Polygon in meters
        separation_m: Separation distance between survey lines
        
    Returns:
        tuple: (path1, path2, angle_half1, angle_half2) or (None, None, None, None) on error
    """
    print("\n[SURVEY] Generating survey paths...")
    try:
        # Split polygon into two halves
        poly1_m, poly2_m, cut_line = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
        
        if poly1_m is None or poly2_m is None:
            print("ERROR: Failed to split polygon into two regions")
            print("  Polygon may be too small or irregularly shaped")
            return None, None, None, None
        
        # Find best angles for each half
        print("  Finding optimal angles for each region...")
        angle_half1, path1, _, _, _ = cs.find_best_angle_for_region(
            poly1_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        angle_half2, path2, _, _, _ = cs.find_best_angle_for_region(
            poly2_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        
        # Validate path generation
        if angle_half1 is None or path1 is None:
            print("ERROR: Could not generate survey path for first half")
            print(f"  Separation {separation_m}m may be too large for region size")
            minx, miny, maxx, maxy = poly_m.bounds
            min_dim = min(maxx - minx, maxy - miny)
            print(f"  Try reducing separation to <{min_dim/2:.1f}m")
            return None, None, None, None
        
        if angle_half2 is None or path2 is None:
            print("ERROR: Could not generate survey path for second half")
            print(f"  Separation {separation_m}m may be too large for region size")
            minx, miny, maxx, maxy = poly_m.bounds
            min_dim = min(maxx - minx, maxy - miny)
            print(f"  Try reducing separation to <{min_dim/2:.1f}m")
            return None, None, None, None
        
        if path1.is_empty or path2.is_empty:
            print("ERROR: Generated paths are empty")
            print("  Check polygon size and separation distance")
            return None, None, None, None
        
        print(f"  ✓ Half 1: angle={angle_half1:.1f} deg, path length={path1.length:.1f}m")
        print(f"  ✓ Half 2: angle={angle_half2:.1f} deg, path length={path2.length:.1f}m")
        
        return path1, path2, angle_half1, angle_half2
        
    except Exception as e:
        print(f"ERROR generating survey paths: {e}")
        print("Traceback:")
        traceback.print_exc()
        return None, None, None, None


def generate_waypoints_from_paths(path1, path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon):
    """
    Generate GPS waypoints from survey paths.
    
    Args:
        path1, path2: Shapely LineString paths in meters
        lat0, lon0: Reference GPS coordinates
        m_per_deg_lat, m_per_deg_lon: Meter-per-degree conversion factors
        
    Returns:
        tuple: (waypoints1, waypoints2, all_waypoints, total_distance, est_time_min) or None on error
    """
    from config import SEARCH_SPEED
    
    print("\n[SURVEY] Converting paths to waypoints...")
    
    try:
        # Create waypoint generators for both paths
        gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        
        # Use relative altitude for now - will be converted to absolute later
        survey_altitude_relative = abs(SURVEY_ALT)  # 12.19m relative altitude
        
        # Generate waypoints for both paths
        waypoints1 = gen1.generate_waypoints(spacing_m=WAYPOINT_SPACING, altitude_m=survey_altitude_relative)
        waypoints2 = gen2.generate_waypoints(spacing_m=WAYPOINT_SPACING, altitude_m=survey_altitude_relative)
        
        # Validate waypoint generation
        if not waypoints1 or not waypoints2:
            print("ERROR: Failed to generate waypoints from paths")
            return None
        
        print(f"  ✓ Path 1: {len(waypoints1)} waypoints")
        print(f"  ✓ Path 2: {len(waypoints2)} waypoints")
        
        # Combine waypoints (first half, then second half)
        all_waypoints = waypoints1 + waypoints2
        print(f"  ✓ Total waypoints: {len(all_waypoints)}")
        
        # Estimate mission time
        total_distance = path1.length + path2.length
        est_time_min = (total_distance / SEARCH_SPEED) / 60
        print(f"  Estimated mission time: {est_time_min:.1f} minutes")
        print(f"  Survey altitude: {abs(SURVEY_ALT):.1f}m")
        
        return waypoints1, waypoints2, all_waypoints, total_distance, est_time_min
        
    except Exception as e:
        print(f"ERROR generating waypoints: {e}")
        print("Traceback:")
        traceback.print_exc()
        return None


def generate_survey_waypoints(separation_m=DEFAULT_SEPARATION_M):
    """
    Generate complete survey path waypoints (wrapper function).
    
    Args:
        separation_m: Distance between survey lines
        
    Returns:
        list: GPS waypoints [(lat, lon, alt), ...] or None on error
    """
    # Load polygon
    poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon = load_survey_polygon()
    if poly_m is None:
        return None
    
    # Generate paths
    path1, path2, angle_half1, angle_half2 = generate_survey_paths(poly_m, separation_m)
    if path1 is None or path2 is None:
        return None
    
    # Generate waypoints
    result = generate_waypoints_from_paths(path1, path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    if result is None:
        return None
    
    waypoints1, waypoints2, all_waypoints, total_distance, est_time_min = result
    return all_waypoints


def main():
    """Test path planning module"""
    print("="*60)
    print("PATH PLANNING MODULE TEST")
    print("="*60)
    
    # Test polygon loading
    print("\n[TEST] Loading polygon...")
    poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon = load_survey_polygon()
    
    if poly_m is None:
        print("✗ Failed to load polygon")
        print(f"  Check that {PLAN_FILE} exists")
        return
    
    print("✓ Polygon loaded successfully")
    
    # Test path generation
    print(f"\n[TEST] Generating survey paths with {DEFAULT_SEPARATION_M}m separation...")
    path1, path2, angle1, angle2 = generate_survey_paths(poly_m, DEFAULT_SEPARATION_M)
    
    if path1 is None:
        print("✗ Failed to generate paths")
        return
    
    print("✓ Paths generated successfully")
    
    # Test waypoint generation
    print("\n[TEST] Generating waypoints...")
    waypoints = generate_survey_waypoints(separation_m=DEFAULT_SEPARATION_M)
    
    if waypoints is None:
        print("✗ Failed to generate waypoints")
        return
    
    print(f"✓ Generated {len(waypoints)} waypoints")
    print(f"\nFirst 3 waypoints:")
    for i, (lat, lon, alt) in enumerate(waypoints[:3]):
        print(f"  {i+1}. ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
