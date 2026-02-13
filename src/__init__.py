"""
SARX - Search and Rescue eXtended
Modular drone system for autonomous human detection and delivery.
"""

__version__ = "2.0.0"
__author__ = "SARX Team"

# Import key classes and functions for convenience
from .config import *
from .cv import load_model, infer_pytorch
from .waypoints import WaypointGenerator
from .path_planning import generate_survey_waypoints
from .DroneController import DroneController
from .servo import init_servo, drop_payload
from .detection import (
    init_cameras, 
    is_in_exclusion_zone, 
    tag_location
)

__all__ = [
    # Config
    'CAM_PREVIEW_SIZE', 'IMG_SIZE', 'SYSTEM_ADDRESS',
    'TAKEOFF_ALT', 'SURVEY_ALT', 'DELIVERY_ALT',
    
    # CV
    'load_model', 'infer_pytorch',
    
    # Waypoints
    'WaypointGenerator',
    
    # Path Planning
    'generate_survey_waypoints',
    
    # Drone
    'DroneController',
    
    # Servo
    'init_servo', 'drop_payload',
    
    # Detection
    'init_cameras', 'is_in_exclusion_zone', 'tag_location',
]
