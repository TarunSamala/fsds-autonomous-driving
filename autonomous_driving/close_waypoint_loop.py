#!/usr/bin/env python3
"""
Waypoint Loop Closer - Closes open tracks by connecting end to start.

Detects the gap between last and first waypoint and interpolates
intermediate points to create a perfect closed loop.
"""

import json
import math


def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def interpolate_points(p1, p2, num_points):
    """Create interpolated points between p1 and p2."""
    points = []
    for i in range(1, num_points + 1):
        t = i / (num_points + 1)
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        points.append([x, y])
    return points


def close_waypoint_loop(input_file, output_file, gap_threshold=2.0):
    """
    Close the waypoint loop.
    
    Args:
        input_file: Path to input waypoints.json
        output_file: Path to output closed_waypoints.json
        gap_threshold: Max distance to consider as gap (meters)
    """
    print("=" * 80)
    print("🔄 WAYPOINT LOOP CLOSER")
    print("=" * 80)
    
    # Load waypoints
    try:
        with open(input_file, 'r') as f:
            waypoints = json.load(f)
        print(f"✅ Loaded {len(waypoints)} waypoints")
    except Exception as e:
        print(f"❌ Error loading: {e}")
        return
    
    if len(waypoints) < 2:
        print("❌ Need at least 2 waypoints")
        return
    
    # Get first and last waypoints
    first_wp = waypoints[0]
    last_wp = waypoints[-1]
    
    gap_dist = distance(last_wp, first_wp)
    print(f"📍 First waypoint: ({first_wp[0]:.2f}, {first_wp[1]:.2f})")
    print(f"📍 Last waypoint:  ({last_wp[0]:.2f}, {last_wp[1]:.2f})")
    print(f"📏 Gap distance: {gap_dist:.2f}m")
    
    # If gap is small, just connect them
    if gap_dist < gap_threshold:
        print(f"✅ Gap is small ({gap_dist:.2f}m < {gap_threshold}m), connecting...")
        
        # Calculate number of interpolation points (1 point per ~0.2m)
        num_interp = max(1, int(gap_dist / 0.2))
        print(f"   Adding {num_interp} interpolation points...")
        
        # Create interpolation points
        closing_points = interpolate_points(last_wp, first_wp, num_interp)
        
        # Append to waypoints
        closed_waypoints = waypoints + closing_points
        
        print(f"✅ Closed loop: {len(waypoints)} → {len(closed_waypoints)} waypoints")
        
    else:
        print(f"⚠️  Gap is large ({gap_dist:.2f}m > {gap_threshold}m)")
        print("   No closing performed - gap too large for safety")
        closed_waypoints = waypoints
    
    # Save closed waypoints
    try:
        with open(output_file, 'w') as f:
            json.dump(closed_waypoints, f, indent=2)
        print(f"💾 Saved to {output_file}")
        print("=" * 80)
        
        # Verify
        print("\n✅ VERIFICATION:")
        print(f"   Original: {len(waypoints)} waypoints")
        print(f"   Closed:   {len(closed_waypoints)} waypoints")
        
        new_gap = distance(closed_waypoints[-1], closed_waypoints[0])
        print(f"   New gap distance: {new_gap:.4f}m")
        
        if new_gap < 0.5:
            print("   ✅ Loop successfully closed!")
        else:
            print("   ⚠️  Loop still has gap - may need more interpolation")
        
    except Exception as e:
        print(f"❌ Error saving: {e}")


if __name__ == '__main__':
    # Close the loop
    close_waypoint_loop(
        input_file='/workspace/ros2_ws/waypoints.json',
        output_file='/workspace/ros2_ws/waypoints_closed.json',
        gap_threshold=2.0
    )

