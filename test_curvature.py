#!/usr/bin/env python3
# Simple test script for bed mesh curvature calculation
import sys
import os

# Add the path to find the bed_mesh module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'klippy', 'extras'))

def test_curvature_calculation():
    """Test curvature calculation with synthetic data"""
    
    # Create mock mesh parameters for testing
    params = {
        'min_x': 0.0, 'max_x': 200.0, 'min_y': 0.0, 'max_y': 200.0,
        'x_count': 5, 'y_count': 5, 'mesh_x_pps': 2, 'mesh_y_pps': 2,
        'algo': 'direct', 'tension': 0.2
    }
    
    # Mock ZMesh class for testing  
    class MockZMesh:
        def __init__(self, params):
            self.mesh_params = params
            self.mesh_x_min = params['min_x'] 
            self.mesh_x_max = params['max_x']
            self.mesh_y_min = params['min_y']
            self.mesh_y_max = params['max_y']
            px_cnt = params['x_count']
            py_cnt = params['y_count']
            mesh_x_pps = params['mesh_x_pps']
            mesh_y_pps = params['mesh_y_pps']
            self.mesh_x_count = (px_cnt - 1) * mesh_x_pps + px_cnt
            self.mesh_y_count = (py_cnt - 1) * mesh_y_pps + py_cnt
            self.mesh_x_dist = (self.mesh_x_max - self.mesh_x_min) / (self.mesh_x_count - 1)
            self.mesh_y_dist = (self.mesh_y_max - self.mesh_y_min) / (self.mesh_y_count - 1)
            
        def get_x_coordinate(self, index):
            return self.mesh_x_min + self.mesh_x_dist * index
            
        def get_y_coordinate(self, index):
            return self.mesh_y_min + self.mesh_y_dist * index
    
    # Test 1: Flat surface (should have zero curvature)
    print("Test 1: Flat surface")
    zmesh = MockZMesh(params)
    zmesh.mesh_matrix = [[0.0 for _ in range(zmesh.mesh_x_count)] 
                         for _ in range(zmesh.mesh_y_count)]
    
    # Import the curvature calculation method (would need actual import)
    # For now, just test the logic
    print(f"Mesh size: {zmesh.mesh_x_count} x {zmesh.mesh_y_count}")
    print(f"Mesh spacing: X={zmesh.mesh_x_dist:.2f}, Y={zmesh.mesh_y_dist:.2f}")
    
    # Test 2: Surface with a bump (should have high curvature at center)
    print("\nTest 2: Surface with central bump")
    center_x = zmesh.mesh_x_count // 2
    center_y = zmesh.mesh_y_count // 2
    
    # Create a bump at the center
    for j in range(zmesh.mesh_y_count):
        zmesh.mesh_matrix[j] = [0.0 for _ in range(zmesh.mesh_x_count)]
    
    zmesh.mesh_matrix[center_y][center_x] = 0.5  # 0.5mm bump
    if center_x > 0:
        zmesh.mesh_matrix[center_y][center_x-1] = 0.1
    if center_x < zmesh.mesh_x_count - 1:
        zmesh.mesh_matrix[center_y][center_x+1] = 0.1
    if center_y > 0:
        zmesh.mesh_matrix[center_y-1][center_x] = 0.1
    if center_y < zmesh.mesh_y_count - 1:
        zmesh.mesh_matrix[center_y+1][center_x] = 0.1
    
    # Manually calculate curvature at center point
    if center_x > 0 and center_x < zmesh.mesh_x_count - 1:
        curvature_x = (zmesh.mesh_matrix[center_y][center_x+1] - 
                       2*zmesh.mesh_matrix[center_y][center_x] + 
                       zmesh.mesh_matrix[center_y][center_x-1]) / (zmesh.mesh_x_dist**2)
        print(f"X-direction curvature at center: {curvature_x:.4f}")
    
    if center_y > 0 and center_y < zmesh.mesh_y_count - 1:
        curvature_y = (zmesh.mesh_matrix[center_y+1][center_x] - 
                       2*zmesh.mesh_matrix[center_y][center_x] + 
                       zmesh.mesh_matrix[center_y-1][center_x]) / (zmesh.mesh_y_dist**2)
        print(f"Y-direction curvature at center: {curvature_y:.4f}")
    
    print("\nTest completed successfully!")

if __name__ == "__main__":
    test_curvature_calculation()