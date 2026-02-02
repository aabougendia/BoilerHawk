"""
Unit tests for control node.
"""

import unittest
from geometry_msgs.msg import Point
import math


class TestControlUtils(unittest.TestCase):
    """Test utility functions for control node."""
    
    def test_distance_calculation(self):
        """Test Euclidean distance calculation."""
        point1 = Point()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0
        
        point2 = Point()
        point2.x = 3.0
        point2.y = 4.0
        point2.z = 0.0
        
        # Calculate distance
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Should be 5.0 (3-4-5 triangle)
        self.assertAlmostEqual(distance, 5.0, places=5)
    
    def test_distance_3d(self):
        """Test 3D distance calculation."""
        point1 = Point()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0
        
        point2 = Point()
        point2.x = 1.0
        point2.y = 1.0
        point2.z = 1.0
        
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Should be sqrt(3)
        self.assertAlmostEqual(distance, math.sqrt(3), places=5)
    
    def test_waypoint_threshold(self):
        """Test waypoint reached logic."""
        threshold = 0.5
        
        # Point within threshold
        point1 = Point()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0
        
        point2 = Point()
        point2.x = 0.3
        point2.y = 0.3
        point2.z = 0.0
        
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        self.assertTrue(distance < threshold)
        
        # Point outside threshold
        point3 = Point()
        point3.x = 1.0
        point3.y = 1.0
        point3.z = 0.0
        
        dx = point1.x - point3.x
        dy = point1.y - point3.y
        dz = point1.z - point3.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        self.assertFalse(distance < threshold)


if __name__ == '__main__':
    unittest.main()
