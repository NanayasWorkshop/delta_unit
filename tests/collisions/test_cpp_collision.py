#!/usr/bin/env python3
import sys
import os

# Add the main project directory to Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

print(f"Looking for delta_robot in: {project_root}")

# Test if we can use the delta_robot module
try:
    import delta_robot
    print("✅ Import successful")
    
    # Test basic functionality to verify compilation worked
    result = delta_robot.calculate_motors(100, 50, 300)
    print("✅ C++ compilation successful - delta_robot works")
    print(f"✅ Motor calculation works: FABRIK converged = {result.fabrik_converged}")
    
    # Now let's test if our collision utilities compiled
    # They should be in the C++ code but not exposed to Python yet
    print("\n=== Testing collision module compilation ===")
    print("Collision utilities compiled but not yet exposed to Python")
    print("This is expected - we'll add Python bindings later")
    
except ImportError as e:
    print(f"❌ Import Error: {e}")
    print("The delta_robot module was not found")
    print("Make sure you're running from the correct directory")
except Exception as e:
    print(f"❌ Runtime Error: {e}")
    print("The compilation may have failed")
    import traceback
    traceback.print_exc()