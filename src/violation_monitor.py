# William Vu
# San Jose State University
# 2025

import carla
import time, random, sys, math
from datetime import datetime
from enum import IntEnum

# ViolationMonitor for CARLA Simulator
# Follows a vehicle for a specified duration of time
# If the vehicle violates a traffic law, keeps track of that data

# The ViolationMonitor class is a fork/superset of carla.Vehicle

# Types of violations:
#    RedLightViolation
#    IllegalStopViolation
#    RunStopSign
#    CollisionViolation
#    SpeedingViolation
#    SlowViolation

class Violation:
    def __init__(self):
        self.type_id:str = None

# If the vehicle runs a red light
class RedLightViolation (Violation):
    def __init__(self):
        super().__init__()
        self.type_id = 'RedLightViolation'
        self.junction_id = None

# If the vehicle makes an illegal stop 
class IllegalStopViolation (Violation): 
    def __init__(self, transform = None):
        super().__init__()
        self.type_id = 'StopViolation'
        self.transform:carla.Transform = transform

# If the vehicle drives faster than the speed limit
class SpeedingViolation (Violation): # Done
    def __init__(self, speed = None, limit = None):
        super().__init__()
        self.type_id = 'SpeedingViolation'
        self.speed = speed
        self.limit = limit

# If the vehicle drives too slowly without obstacles
class SlowViolation (Violation):
    def __init__(self, speed = None, limit = None):
        super().__init__()
        self.type_id = 'SlowViolation'
        self.speed = speed
        self.limit = limit

class RunStopSignViolation (Violation):
    def __init__(self, stop_sign = None):
        super().__init__()
        self.type_id = 'RunStopSignViolation'
        self.stop_sign = stop_sign

# If the vehicle collides with another actor or landmark
class CollisionViolation (Violation): # Done
    def __init__(self, other_actor = None, impulse = None):
        super().__init__()
        self.type_id = 'CollisionViolation'
        self.other_actor = other_actor
        self.severity = None
        if impulse < 1000:
            self.severity = 'Minor'
        elif impulse < 5000:
            self.severity = 'Moderate'
        else:
            self.severity = 'Severe'

class ViolationMonitor: 
    def __init__(self, *, host = None, port = None):
        # Carla Simulator essentials
        if host is None:
            host = 'localhost'
        if port is None:
            port = 2000
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()
        self.traffic_manager = self.client.get_trafficmanager()
        self.spectator = self.world.get_spectator()

        # Object Mnagaement
        self.actor_list = []
        self.vehicle = None
        self.collision_sensor = None
        self.obstacle_sensor = None
        self.previous_obstacle_detection = None # Timestamp
        self.start_time = 0.0
        self.last_junction_id = None
        self.last_illegal_stop_location = None
        self.stop_signs = []; self._initialize_stop_signs()

        self.violations = []

    def monitor(self, *, vehicle = None, duration = 300.0):
        # Setup self.vehicle
        if vehicle is None:
            self._spawn_vehicle()
        else:
            self.vehicle = vehicle
            if self.vehicle is None:
                sys.stderr.write('ViolationMonitor.monitor(): pass a valid vehicle object')
                sys.exit(1)
        self._spawn_sensors()
        self._activate_sensors()
        self._enable_autopilot(True)

        start = time.time()

        while time.time() - start < duration:
            if self.vehicle.is_at_traffic_light():
                self._monitor_light_violation()

            sign_bounding_box = self._get_stop_sign()
            if sign_bounding_box is not None:
                self._monitor_stop_sign_violation(sign_bounding_box)

            self._monitor_illegal_stop_violation()

            self._monitor_speed_violation()
        
        self.collision_sensor.stop()

    def _initialize_stop_signs(self):
        actors = self.world.get_actors()
        for actor in actors:
            if 'stop' in actor.type_id:
                self.stop_signs.append(actor)

    def _spawn_vehicle(self):
        bp = random.choice(self.blueprint_library.filter('vehicle.tesla.model3'))
        sp = random.choice(self.map.get_spawn_points())
        vehicle = None
        while vehicle is None:
            vehicle = self.world.try_spawn_actor(bp, sp)
        
        self.actor_list.append(vehicle)
        self.vehicle = vehicle

    def _spawn_sensors(self):
        # Collision sensor
        col_bp = self.blueprint_library.find('sensor.other.collision')
        self.collision_sensor = self.world.try_spawn_actor(
            col_bp, 
            carla.Transform(carla.Location(0, 0, 0)), 
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid
        )
        self.actor_list.append(self.collision_sensor)

        # Obstacle sensor
        sens_bp = self.blueprint_library.find('sensor.other.obstacle')
        self.obstacle_sensor = self.world.try_spawn_actor(
            sens_bp,
            carla.Transform(carla.Location(0, 0, 0)),
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid,
            sensor_tick = 0.25 # To reduce computational costs
        )
        self.actor_list.append(self.obstacle_sensor)

    def _activate_sensors(self):
        # This will actively start detecting collision violations,
        # and log them as they occur
        self.collision_sensor.listen(self._add_collision_violation)
        self.obstacle_sensor.listen(self._detect_obstacle)

    def _add_collision_violation(self, event): 
        """
        Callback function for the collision sensor
        """
        other_actor = event.other_actor
        impulse = event.impulse
        vi = CollisionViolation(other_actor, impulse)
        self.violations.append(vi)

    def _detect_obstacle(self, event):
        """
        Callback function for the obstacle sensor
        """
        self.previous_obstacle_detection = event.timestamp

    def _enable_autopilot(self, foo:bool):
        if foo:
            self.traffic_manager.ignore_lights_percentage(self.vehicle, 100)
            self.vehicle.set_autopilot(True)
        else:
            self.traffic_manager.ignore_lights_percentage(self.vehicle, 0)
            self.vehicle.set_autopilot(False)

    def _monitor_light_violation(self):
        # The vehicle is in a red light affected area at this point
        # If the vehicle's last traffic light state before leaving the area was red,
        # log a Violtation.RunRedLight
        # Furthermore, if the vehicle experiences an entire green light cycle before leaving the area,
        # and there is no obstruction directly in front of it,
        # log a Violation.FailToGoAtGreen

        currState = None
        while self.vehicle.get_traffic_light() is not None:
            currState = self.vehicle.get_traffic_light_state()
            # Check for unnecessary stop
            if self._monitor_stop_violation():
                return False
            
        if currState == carla.TrafficLightState.Red:
            vi = RedLightViolation()
        
    def _monitor_illegal_stop_violation(self):
        # If a vehicle is stopped illegally for 3 seconds, log an IllegalStopViolation
        v = self.vehicle.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

        stopped = False
        if speed <= 0.05:
            stopped = True

        obstacle = False
        if abs(time.time() - self.previous_obstacle_detection) < 1.0:
            obstacle = True

        green = False
        if self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Green:
            green = True

        if stopped and not obstacle and green and not self._get_stop_sign():
            time.sleep(3.0)
            if stopped and not obstacle and green and not self._get_stop_sign():
                vi = IllegalStopViolation(self.vehicle.get_transform())
                self.violations.append(vi)
                return True
        return False
    
    def _monitor_speed_violation(self):
        # If the vehicle is going > 2 km/hr above the speed limit,
        # log a speeding violation
        # If the vehicle is going < half the speed limit with no obstructions in front of it,
        # log a slow violation

        limit = self.vehicle.get_speed_limit()
        # speed = magnitude of the velocity vector
        v = self.vehicle.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

        if speed > limit + 2.0:
            vi = SpeedingViolation(speed,limit)
            self.violations.append(vi)
            return
        if speed < limit / 2:
            if abs(time.time() - self.previous_obstacle_detection) < 1.0:
                vi = SlowViolation(speed,limit)
                self.violations.append(vi)

    def _get_stop_sign(self):
        """
        Return the stop sign affecting the vehicle
        """
        for stop_sign in self.stop_signs:
            bounding_box = stop_sign.trigger_volume
            if bounding_box.contains(self.vehicle.get_location(), self.vehicle.get_transform()):
                return bounding_box
        return None

    def _monitor_stop_sign_violation(self, sign_bounding_box):
        # Right now, the vehicle should be in the bounding box of a stop sign
        # If the vehicle leaves the bounding box before its velocity turns 0, log a RunStopSignViolation
        stopped = False
        while sign_bounding_box.contains(self.vehicle.get_location(), self.vehicle.get_transform()):
            v = self.vehicle.get_velocity()
            spd = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            if spd == 0:
                stopped = True
                break
        if not stopped:
            vi = RunStopSignViolation()

    def _destroy_actors(self):
        for actor in self.actor_list:
            # For vehicles
            if hasattr(actor, 'set_autopilot'):
                actor.set_autopilot(False)
            # For sensors
            if hasattr(actor, 'stop'):
                actor.stop()
            actor.destroy()

    def _reset(self):
        self._destroy_actors()
        
        self.actor_list = []
        self.vehicle = None
        self.collision_sensor = None
        self.obstacle_sensor = None
        self.previous_obstacle_detection = None # Timestamp
        self.start_time = 0.0
        self.last_junction_id = None # To avoid certain duplicate violations
        self.last_illegal_stop_location = None
        self.stop_signs = []; self._initialize_stop_signs()

        self.violations = []

if __name__ == '__main__':
    pass
    # Not sure how I want this to look yet
    # I want it to be a fork/superset of carla.Vehicle
    # So a vehicle but with the monitor() function
