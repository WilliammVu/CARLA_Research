# distance from start idea?

import glob, sys, random, time, math, numpy

import carla
from global_route_planner import GlobalRoutePlanner, RoadOption

class TrafficElement:
    def __init__(self):
        self.waypoint:carla.Waypoint = None
        self.stop_sign = False
        self.yield_sign = False
        self.traffic_light = False
        self.junction = False
        self.direction = RoadOption.VOID

    def is_significant(self):
        if self.direction == RoadOption.LANEFOLLOW or self.direction == RoadOption.VOID:
            return False
        if self.stop_sign:
            return True
        if self.traffic_light:
            return True
        if self.junction and self.direction != RoadOption.STRAIGHT:
            return True
        return False

class RouteInspection:
    def __init__(self, *, world:carla.World, map:carla.Map = None, resolution = 2.0):
        # Carla Simulator Essentials
        self.world = world

        if map == None:
            self.map = self.world.get_map()
        else:
            self.map = map

        # GlobalRoutePlanner for A* shortest path finding
        self.resolution = resolution
        self.global_route_planner = GlobalRoutePlanner(self.map, self.resolution)

        self._stop_sign_waypoints = []

        self._initialize_stop_sign_waypoints()
    
    def get_route(self, origin:carla.Location, destination:carla.Location):
        # Returns list of relevant TrafficElement, describing the route
        # There are two checks to avoid duplicate waypoints:
        # - significant_waypoints and junction_id_stack
        waypoint_route = self.global_route_planner.trace_route(origin, destination)

        # Draw a visual route in the Sim for clarity/debugging
        for waypoint, _ in waypoint_route:
            self.world.debug.draw_point(waypoint.transform.location + carla.Location(z=1), life_time=120)

        significant_waypoints = []
        junction_id_stack = []

        route = []

        for waypoint, direction in waypoint_route:

            location = waypoint.transform.location

            # Avoid adding duplicate waypoints
            avoid = False
            for wp in significant_waypoints:
                distance = location.distance(wp.transform.location)
                if distance <= self.resolution:
                    avoid = True
                    break
            if waypoint.is_junction and junction_id_stack and waypoint.junction_id == junction_id_stack[-1]:
                avoid = True
            if avoid:
                continue

            te = TrafficElement()

            # Determine self.waypoint
            te.waypoint = waypoint
            # Determine self.direction
            te.direction = direction

            # Determine self.junction and self.traffic_light
            if waypoint.is_junction:
                te.junction = True

                traffic_lights = self.world.get_traffic_lights_in_junction(waypoint.junction_id)
                
                if len(traffic_lights) != 0:
                    te.traffic_light = True
                
            # Determine self.stop_sign
            for wp in self._stop_sign_waypoints:
                distance = wp.transform.location.distance(location)
                if distance < 1.0:
                    te.stop_sign = True
                    break

            if te.is_significant():
                route.append(te)
                significant_waypoints.append(waypoint)
                junction_id_stack.append(waypoint.junction_id)

        return route

    # Helper functions

    def _initialize_stop_sign_waypoints(self):
        actors = self.world.get_actors()

        for actor in actors:
            if 'stop' in actor.type_id:
                waypoints = self._get_stop_sign_waypoints(actor)
                self._stop_sign_waypoints.append(waypoints[0])

    def _rotate_point(self, point, angle):
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)

    def _get_stop_sign_waypoints(self, stop_sign):
        # Determine area of stop_sign
        base_transform = stop_sign.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(stop_sign.trigger_volume.location)

        # Discretize the trigger box into points
        area_ext = stop_sign.trigger_volume.extent
        x_values = numpy.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes

        area = []
        for x in x_values:
            point = self._rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self.map.get_waypoint(pt)
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

        return wps

