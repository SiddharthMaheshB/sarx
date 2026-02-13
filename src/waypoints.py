#!/usr/bin/env python3
"""
Waypoint generation module for SARX system.
Converts Shapely paths to navigable GPS waypoints.
"""

from config import SURVEY_ALT, WAYPOINT_SPACING


class WaypointGenerator:
    """Converts Shapely LineString paths to navigable GPS waypoints"""
    
    def __init__(self, path_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon):
        """
        Initialize waypoint generator with path in local meters.
        
        Args:
            path_m: Shapely LineString in local meters (East, North)
            lat0, lon0: Reference GPS point for coordinate conversion
            m_per_deg_lat, m_per_deg_lon: Meter-per-degree conversion factors
        """
        self.path_m = path_m
        self.lat0 = lat0
        self.lon0 = lon0
        self.m_per_deg_lat = m_per_deg_lat
        self.m_per_deg_lon = m_per_deg_lon
        self.waypoints_gps = []
        self.waypoints_local = []
    
    def generate_waypoints(self, spacing_m=5.0, altitude_m=None):
        """
        Generate waypoints along path at specified spacing.
        
        Args:
            spacing_m: Distance between waypoints in meters
            altitude_m: Altitude for waypoints (defaults to survey altitude)
            
        Returns:
            List of (lat, lon, alt_m) tuples in GPS coordinates
        """
        if self.path_m is None or self.path_m.is_empty:
            print("ERROR: No valid path to generate waypoints")
            return []
        
        if altitude_m is None:
            altitude_m = abs(SURVEY_ALT)
        
        total_length = self.path_m.length
        num_points = max(2, int(total_length / spacing_m) + 1)
        
        print(f"  Generating {num_points} waypoints along {total_length:.1f}m path at {altitude_m:.1f}m altitude")
        
        for i in range(num_points):
            # Interpolate point along path (normalized = 0.0 to 1.0)
            fraction = i / (num_points - 1) if num_points > 1 else 0.0
            point = self.path_m.interpolate(fraction, normalized=True)
            
            x_m = point.x  # East in meters
            y_m = point.y  # North in meters
            
            # Convert local meters to lat/lon offset
            dlat_deg = y_m / self.m_per_deg_lat
            dlon_deg = x_m / self.m_per_deg_lon
            
            lat = self.lat0 + dlat_deg
            lon = self.lon0 + dlon_deg
            
            self.waypoints_local.append((x_m, y_m))
            self.waypoints_gps.append((lat, lon, altitude_m))
        
        return self.waypoints_gps
    
    def get_local_waypoints(self):
        """Return waypoints in local meters (for visualization)"""
        return self.waypoints_local
    
    def get_gps_waypoints(self):
        """Return waypoints in GPS coordinates"""
        return self.waypoints_gps


def main():
    """Test waypoint generator with a simple path"""
    from shapely.geometry import LineString
    
    print("="*60)
    print("WAYPOINT GENERATOR TEST")
    print("="*60)
    
    # Create a simple test path (100m north, then 100m east)
    path = LineString([(0, 0), (0, 100), (100, 100)])
    
    # Test coordinates (approximate Delhi area)
    lat0 = 28.4159
    lon0 = 77.5253
    m_per_deg_lat = 111320.0  # Approximate
    m_per_deg_lon = 111320.0 * 0.88  # Adjust for latitude
    
    print(f"\n[TEST] Creating waypoint generator")
    print(f"  Path length: {path.length:.1f}m")
    print(f"  Reference: ({lat0:.6f}, {lon0:.6f})")
    
    gen = WaypointGenerator(path, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    
    print(f"\n[TEST] Generating waypoints with {WAYPOINT_SPACING}m spacing...")
    waypoints = gen.generate_waypoints(spacing_m=WAYPOINT_SPACING, altitude_m=10.0)
    
    print(f"\n✓ Generated {len(waypoints)} waypoints:")
    for i, (lat, lon, alt) in enumerate(waypoints[:5]):  # Show first 5
        print(f"  {i+1}. ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
    
    if len(waypoints) > 5:
        print(f"  ... and {len(waypoints)-5} more")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
