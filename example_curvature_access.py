#!/usr/bin/env python3
"""
Example script showing how external applications can access bed mesh curvature data.
This demonstrates the API that Moonraker, Fluidd, KlipperScreen, etc. can use.
"""

def example_curvature_access():
    """Example of how to access curvature data from external applications"""
    
    # This is how external applications would access the data through Klipper's API
    print("=== Bed Mesh Curvature Data Access Example ===")
    
    # Simulate getting status from bed_mesh object
    # In real usage: status = printer.lookup_object('bed_mesh').get_status()
    example_status = {
        'profile_name': 'default',
        'mesh_min': (0.0, 0.0),
        'mesh_max': (200.0, 200.0),
        'probed_matrix': [[0.1, 0.05, 0.0], [0.08, 0.02, -0.05], [0.0, -0.1, -0.15]],
        'mesh_matrix': [
            [0.1, 0.075, 0.05, 0.025, 0.0],
            [0.09, 0.065, 0.04, 0.015, -0.01], 
            [0.08, 0.055, 0.03, 0.005, -0.02],
            [0.04, 0.015, -0.01, -0.035, -0.06],
            [0.0, -0.025, -0.05, -0.075, -0.1]
        ],
        'curvature_x_matrix': [
            [0.0, -0.001, -0.001, -0.001, 0.0],
            [0.0, -0.001, -0.001, -0.001, 0.0],
            [0.0, -0.001, -0.001, -0.001, 0.0],
            [0.0, -0.002, -0.002, -0.002, 0.0],
            [0.0, -0.001, -0.001, -0.001, 0.0]
        ],
        'curvature_y_matrix': [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [-0.001, -0.001, -0.001, -0.001, -0.001],
            [-0.001, -0.001, -0.001, -0.001, -0.001],
            [-0.002, -0.002, -0.002, -0.002, -0.002],
            [0.0, 0.0, 0.0, 0.0, 0.0]
        ],
        'curvature_warnings': [
            {'x': 100.0, 'y': 75.0, 'curvature_x': -0.002, 'curvature_y': -0.002, 'total_curvature': 0.004}
        ],
        'profiles': {}
    }
    
    print("1. Basic curvature data access:")
    print(f"   Profile: {example_status['profile_name']}")
    print(f"   Mesh area: {example_status['mesh_min']} to {example_status['mesh_max']}")
    
    print("\n2. Curvature matrices:")
    curvature_x = example_status['curvature_x_matrix']
    curvature_y = example_status['curvature_y_matrix']
    print(f"   X-curvature matrix size: {len(curvature_x)}x{len(curvature_x[0])}")
    print(f"   Y-curvature matrix size: {len(curvature_y)}x{len(curvature_y[0])}")
    
    print("\n3. High curvature warnings:")
    warnings = example_status['curvature_warnings']
    if warnings:
        for i, warning in enumerate(warnings):
            print(f"   Warning {i+1}:")
            print(f"     Position: ({warning['x']:.1f}, {warning['y']:.1f})")
            print(f"     X-curvature: {warning['curvature_x']:.4f}")
            print(f"     Y-curvature: {warning['curvature_y']:.4f}")
            print(f"     Total curvature: {warning['total_curvature']:.4f}")
    else:
        print("   No high curvature areas detected")
    
    print("\n4. Practical usage for UI applications:")
    print("   - Display curvature as heatmap overlay on mesh visualization")
    print("   - Show warnings as markers on bed visualization")
    print("   - Alert users to potential bed issues during calibration")
    print("   - Export curvature data for analysis or bed adjustment")
    
    print("\n5. API endpoints for web interfaces:")
    print("   GET /printer/objects/query?bed_mesh=curvature_x_matrix")
    print("   GET /printer/objects/query?bed_mesh=curvature_y_matrix") 
    print("   GET /printer/objects/query?bed_mesh=curvature_warnings")
    
    return example_status

if __name__ == "__main__":
    example_curvature_access()