import carla
import math, random
import numpy as np

# Get stop waypoints for stop signs

def rotate_point(point, angle):
    x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
    y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y
    return carla.Vector3D(x_, y_, point.z)

def get_stop_sign_waypoints(town_map, stop_sign, world):
    base_transform = stop_sign.get_transform()
    base_rot = base_transform.rotation.yaw
    area_loc = base_transform.transform(stop_sign.trigger_volume.location)

    # Discretize the trigger box into points
    area_ext = stop_sign.trigger_volume.extent
    x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes

    area = []
    for x in x_values:
        point = rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
        point_location = area_loc + carla.Location(x=point.x, y=point.y)
        area.append(point_location)

    # Get the waypoints of these points, removing duplicates
    ini_wps = []
    for pt in area:
        wpx = town_map.get_waypoint(pt)
        # As x_values are arranged in order, only the last one has to be checked
        if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
            ini_wps.append(wpx)

    # Advance them until the intersection
    wps = []
    for wpx in ini_wps:
        while not wpx.is_intersection:
            next_wp = wpx.next(0.5)[0]
            if next_wp and not next_wp.is_intersection:
                wpx = next_wp
            else:
                break
        wps.append(wpx)

    for wp in wps:
        world.debug.draw_point(wp.transform.location + carla.Location(z=1), life_time=60)

    return wps
