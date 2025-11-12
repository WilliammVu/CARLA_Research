# William Vu
# San Jose State University
# 2025

import carla
import time, random, math
from datetime import datetime

# ViolationMonitor for CARLA Simulator
# Follows a vehicle for a specified duration of time
# If the vehicle violates a traffic law, keeps track of that data

# The ViolationMonitor class is a manager of carla.Vehicle
# You need to run it separately from your AV script

# Types of violations:
#    RedLightViolation
#    IllegalStopViolation
#    RunStopSignViolation
#    CollisionViolation
#    SpeedingViolation
#    SlowViolation

class Violation:
    def __init__(self):
        self.type_id:str = None

# If the vehicle runs a red light
class RedLightViolation (Violation):
    def __init__(self, light = None):
        super().__init__()
        self.type_id = 'RedLightViolation'
        self.light:carla.TrafficLight = light

# If the vehicle makes an illegal stop 
class IllegalStopViolation (Violation): 
    def __init__(self, transform = None, time_stopped = None):
        super().__init__()
        self.type_id = 'StopViolation'
        self.transform:carla.Transform = transform
        self.time_stopped:float = time_stopped

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
class CollisionViolation (Violation):
    def __init__(self, other_actor = None, impulse = None):
        super().__init__()
        self.type_id = 'CollisionViolation'
        self.other_actor = other_actor
        self.severity = None
        impulse = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        if impulse < 1000:
            self.severity = 'Minor'
        elif impulse < 5000:
            self.severity = 'Moderate'
        else:
            self.severity = 'Severe'

class ViolationMonitor: 
    def __init__(self, *, host = 'localhost', port = 2000, file = None):
        # Carla Simulator essentials
        self.host = host
        self.port = port
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()
        self.traffic_manager = self.client.get_trafficmanager()
        self.spectator = self.world.get_spectator()

        # Object Management
        self.actor_list = []
        self.vehicle = None
        self.collision_sensor = None
        self.obstacle_sensor = None
        self.stop_signs = []; self._initialize_stop_signs()
        self.traffic_lights = []; self._initialize_traffic_lights()

        # Violations
        self.violations = []

        # Duplicate violation prevention
        self.previous_obstacle_detection = 0.0 
        self.last_light_run = 0.0 
        self.last_illegal_stop_transform = None 
        self.last_slow_time = 0.0
        self.last_speed_time = 0.0
        
        self.file = file
        if file is not None:
            with open(self.file, 'w') as file:
                pass

    def monitor(self, *, vehicle = None, duration = 300.0):
        # Setup self.vehicle
        if vehicle is None:
            self._spawn_vehicle()
            self._spawn_sensors()
            self._activate_sensors()
            self._enable_autopilot()
        else:
            self.vehicle = vehicle
            self._spawn_sensors()
            self._activate_sensors()

        # Start monitoring when the vehicle starts moving
        start = time.time()
        while self._get_speed() < 0.05 and time.time() - start < 2.0:
            time.sleep(0.1)
        
        print('started to monitor. Ctrl+C to stop.')
        while time.time() - start < duration:
            try:
                # track light violation while preventing duplicate violation logs
                if time.time() - self.last_light_run > 4.0 and self.vehicle.is_at_traffic_light():
                    self._monitor_light_violation()

                sign = self._get_stop_sign()
                if sign is not None:
                    self._monitor_stop_sign_violation(sign)

                self._monitor_illegal_stop_violation()

                self._monitor_speed_violation()
            except KeyboardInterrupt as e:
                break

        self.collision_sensor.stop()
        self.obstacle_sensor.stop()
        self._reset()

    def _log_violation(self, violation):
        """
        Log a violation to file and/or terminal.
        """
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if isinstance(violation, RedLightViolation):
            message = f"[{timestamp}] RED LIGHT VIOLATION"
            
        elif isinstance(violation, IllegalStopViolation):
            if violation.transform:
                loc = violation.transform.location
                message = f"[{timestamp}] ILLEGAL STOP VIOLATION - Location: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})"
            else:
                message = f"[{timestamp}] ILLEGAL STOP VIOLATION - Location: Unknown"
                
        elif isinstance(violation, SpeedingViolation):
            message = f"[{timestamp}] SPEEDING VIOLATION - Speed: {violation.speed:.1f} km/h (Limit: {violation.limit:.1f} km/h)"
            
        elif isinstance(violation, SlowViolation):
            message = f"[{timestamp}] SLOW DRIVING VIOLATION - Speed: {violation.speed:.1f} km/h (Limit: {violation.limit:.1f} km/h)"
            
        elif isinstance(violation, RunStopSignViolation):
            message = f"[{timestamp}] STOP SIGN VIOLATION"
            
        elif isinstance(violation, CollisionViolation):
            actor_info = violation.other_actor.type_id if violation.other_actor else "Unknown"
            message = f"[{timestamp}] COLLISION VIOLATION - Other Actor: {actor_info}, Severity: {violation.severity}"
            
        else:
            message = f"[{timestamp}] UNKNOWN VIOLATION - Type: {violation.type_id}"
        
        print(message)
        
        if self.file is not None:
            with open(self.file, 'a') as file:
                file.write(message + '\n')

    def _initialize_stop_signs(self):
        actors = self.world.get_actors()
        for actor in actors:
            if 'stop' in actor.type_id:
                self.stop_signs.append(actor)

    def _initialize_traffic_lights(self):
        actors = self.world.get_actors()
        for actor in actors:
            if actor.type_id == 'traffic.traffic_light':
                self.traffic_lights.append(actor)

    def _spawn_vehicle(self):
        bp = random.choice(self.blueprint_library.filter('vehicle.tesla.model3'))
        sp = random.choice(self.map.get_spawn_points())
        self.spectator.set_transform(sp)
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
        impulse = event.normal_impulse
        vi = CollisionViolation(other_actor, impulse)
        self.violations.append(vi)
        self._log_violation(vi)

    def _detect_obstacle(self, event):
        """
        Callback function for the obstacle sensor
        """
        if 'static' not in event.other_actor.type_id:
            self.previous_obstacle_detection = time.time()

    def _enable_autopilot(self, perc = 100):
        self.traffic_manager.ignore_signs_percentage(self.vehicle, perc)
        self.traffic_manager.ignore_lights_percentage(self.vehicle, perc)
        self.traffic_manager.ignore_vehicles_percentage(self.vehicle, perc)
        self.traffic_manager.ignore_walkers_percentage(self.vehicle, perc)
        self.traffic_manager.global_percentage_speed_difference(-50)
        self.vehicle.set_autopilot(True)

    def _monitor_light_violation(self):
        # The vehicle is in a red light affected area at this point

        if time.time() - self.last_light_run < 7.0:
            return False
        
        light = self.vehicle.get_traffic_light()
        last_red_time = None

        currState = None
        while self.vehicle.get_traffic_light() is not None:
            currState = self.vehicle.get_traffic_light_state()

            if self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Red:
                last_red_time = time.time()

            # Check for unnecessary stop
            if self._monitor_illegal_stop_violation():
                return False

        exit_time = time.time()
        if last_red_time is not None and exit_time - last_red_time < 0.3:
            vi = RedLightViolation(light)
            self.violations.append(vi)
            self._log_violation(vi)
            self.last_light_run = time.time()
            return True

        if currState == carla.TrafficLightState.Red:
            vi = RedLightViolation(light)
            self.violations.append(vi)
            self._log_violation(vi)
            self.last_light_run = time.time()
            return True
        
        return False
    
    def _get_speed(self) -> float:
        v = self.vehicle.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        return speed

    def _get_light(self) -> carla.TrafficLight:
        # This process is used in _monitor_illegal_stop_violation()
        # to avoid the unpredictable nature of carla.Vehicle.get_traffic_light()

        vehicle_trans = self.vehicle.get_transform()
        vehicle_loc = vehicle_trans.location
        vehicle_vec = self.vehicle.get_transform().get_forward_vector()

        # Normalize vector
        vmag = (vehicle_vec.x**2 + vehicle_vec.y**2 + vehicle_vec.z**2) ** 0.5
        vehicle_vec = carla.Vector3D(vehicle_vec.x/vmag, vehicle_vec.y/vmag, vehicle_vec.z/vmag)

        for traffic_light in self.traffic_lights:
            if vehicle_loc.distance(traffic_light.get_transform().location) > 20.0:
                continue

            light_vec = traffic_light.get_transform().get_forward_vector()
            lmag = (light_vec.x**2 + light_vec.y**2 + light_vec.z**2) ** 0.5
            light_vec = carla.Vector3D(light_vec.x/lmag, light_vec.y/lmag, light_vec.z/lmag)

            dot = light_vec.x*vehicle_vec.x + light_vec.y*vehicle_vec.y + light_vec.z*vehicle_vec.z

            tol_deg = 10.0
            tol = -math.cos(math.radians(tol_deg))

            if dot <= tol:
                return traffic_light

        return None

    def _monitor_illegal_stop_violation(self):
        if self._get_speed() < 0.05:
            start = time.time()
            trans = self.vehicle.get_transform()

            if self.last_illegal_stop_transform is not None and trans.location.distance(self.last_illegal_stop_transform.location) < 1.0:
                return
            if self._get_stop_sign() is not None:
                return
            if self.vehicle.get_traffic_light_state() == carla.TrafficLightState.Red:
                return
            if self.vehicle.get_traffic_light() is not None:
                if self.vehicle.get_traffic_light().get_state() != carla.TrafficLightState.Green:
                    return
            light = self._get_light()
            if light is not None and light.state != carla.TrafficLightState.Green:
                return
            if time.time() - self.previous_obstacle_detection < 2.0:
                return

            while self._get_speed() < 0.05:
                time.sleep(0.1)
            
            vi = IllegalStopViolation(trans, time.time() - start)
            self._log_violation(vi)
            self.last_illegal_stop_transform = trans
    
    def _monitor_speed_violation(self):
        # If the vehicle is going > 5 km/hr above the speed limit,
        # log a speeding violation
        # If the vehicle is going < half the speed limit with no obstructions in front of it,
        # log a slow violation

        limit = self.vehicle.get_speed_limit()
        # speed = magnitude of the velocity vector
        speed = self._get_speed()
        speed = speed * 3.6 # m/s --> km/h

        if time.time() - self.last_speed_time > 10.0:
            if speed > limit + 5.0:
                vi = SpeedingViolation(speed,limit)
                self.violations.append(vi)
                self._log_violation(vi)
                self.last_speed_time = time.time()
                return
        
        if time.time() - self.last_slow_time > 10.0:
            # Check SlowViolation, not counting stops or red lights
            notRed = self.vehicle.get_traffic_light_state() != carla.TrafficLightState.Red
            notStopped = speed > 0.05
            noObstacle = time.time() - self.previous_obstacle_detection > 1.0
            noStopSign = self._get_stop_sign() == False
            if speed <= (limit / 2.0) and notStopped and notRed and noObstacle and noStopSign:
                vi = SlowViolation(speed,limit)
                self.violations.append(vi)
                self._log_violation(vi)
                self.last_slow_time = time.time()

    def _get_stop_sign(self) -> carla.TrafficSign:
        vehicle_location = self.vehicle.get_location()
        vehicle_forward = self.vehicle.get_transform().get_forward_vector()
        
        for stop_sign in self.stop_signs:
            distance = vehicle_location.distance(stop_sign.get_location())
            
            if distance < 3:
                return stop_sign

            if distance < 7:
                to_sign = stop_sign.get_location() - vehicle_location
                to_sign_normalized = to_sign / (to_sign.length() + 1e-6)
                
                dot_product = vehicle_forward.x * to_sign_normalized.x + \
                            vehicle_forward.y * to_sign_normalized.y + \
                            vehicle_forward.z * to_sign_normalized.z
                
                if dot_product > 0.5:
                    return stop_sign
        
        return None

    def _monitor_stop_sign_violation(self, sign):
        # Right now, the vehicle should be coming closer to a stop sign or at one
        # If the distance between the vehicle and the stop sign starts to increase,
        # but the vehicle never stopped before that point,
        # log a RunStopSignViolation

        stopped = False
        prev_distance = self.vehicle.get_location().distance(sign.get_location())
        time.sleep(0.1)
        curr_distance = self.vehicle.get_location().distance(sign.get_location())
        while curr_distance <= prev_distance or curr_distance < 5.0:
            spd = self._get_speed()
            if spd <= 0.05:
                stopped = True
                break

            prev_distance = curr_distance
            time.sleep(0.1)
            curr_distance = self.vehicle.get_location().distance(sign.get_location())

        if not stopped:
            vi = RunStopSignViolation(sign)
            self.violations.append(vi)
            self._log_violation(vi)

    def _destroy_actors(self):
        for actor in self.actor_list:
            # For vehicles
            if hasattr(actor, 'set_autopilot'):
                actor.set_autopilot(False)
            actor.destroy()

    def _reset(self):
        self._destroy_actors()
        
        # Object Management
        self.actor_list = []
        self.vehicle = None
        self.collision_sensor = None
        self.obstacle_sensor = None

        # Violations
        self.violations = []

        # Duplicate violation prevention
        self.previous_obstacle_detection = 0.0 
        self.last_light_run = 0.0 
        self.last_illegal_stop_transform = None 
        self.last_slow_time = 0.0
        self.last_speed_time = 0.0

if __name__ == '__main__':
    pass
