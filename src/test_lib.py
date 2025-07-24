
import carla
import time
import random
from datetime import datetime

class ViolationMonitor:
    def __init__(self, host='localhost', port=2000):
        """
        Initialize the CARLA client and violation monitor.
        
        Args:
            host (str): CARLA server host
            port (int): CARLA server port
        """
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        
        # Get the world and spawn points
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.spectator = self.world.get_spectator()
        self.trafficmanager = self.client.get_trafficmanager()

        # Vehicle and monitoring variables
        self.vehicle = None
        self.violation_list = []
        self.last_traffic_light_state = carla.TrafficLightState.Green
        self.monitoring = False
        self.junction_id_stack = []  # Stack to track recent junction violations
        
        print(f"Connected to CARLA server at {host}:{port}")
        print(f"Available spawn points: {len(self.spawn_points)}")

    def turn_all_lights_red(self, turn:bool):
        actors = self.world.get_actors()
        lights = []

        for actor in actors:
            if actor.type_id == 'traffic.traffic_light':
                lights.append(actor)
        
        if turn:
            for light in lights:
                light.set_state(carla.TrafficLightState.Red)
                light.freeze(True)
        else:
            for light in lights:
                light.reset_group()
                light.freeze(False)

    def spawn_vehicle(self, vehicle_filter='vehicle.tesla.model3'):
        """
        Spawn a vehicle at a random spawn point.
        
        Args:
            vehicle_filter (str): Blueprint filter for vehicle selection
        """
        try:
            # Get a random vehicle blueprint
            vehicle_bp = random.choice(self.blueprint_library.filter(vehicle_filter))
            
            # Choose a random spawn point
            spawn_point = random.choice(self.spawn_points)
            self.spectator.set_transform(spawn_point)
            
            # Spawn the vehicle
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
            
            if self.vehicle:
                print(f"Vehicle spawned successfully at location: {spawn_point.location}")
                print(f"Vehicle ID: {self.vehicle.id}")
                return True
            else:
                print("Failed to spawn vehicle")
                return False
                
        except Exception as e:
            print(f"Error spawning vehicle: {e}")
            return False

    def enable_autopilot(self):
        """
        Enable autopilot for the spawned vehicle.
        """
        if self.vehicle:
            try:
                self.trafficmanager.ignore_lights_percentage(self.vehicle, 100)
                self.vehicle.set_autopilot(True)
                print("Autopilot enabled")
                return True
            except Exception as e:
                print(f"Error enabling autopilot: {e}")
                return False
        else:
            print("No vehicle available to enable autopilot")
            return False

    def check_red_light_violation(self):
        """
        Check if the vehicle is violating a red light.
        Uses junction ID stack to prevent duplicate consecutive violations.
        
        Returns:
            bool: True if a violation is detected, False otherwise
        """
        if not self.vehicle:
            return False
            
        try:
            # Get current traffic light state
            current_state = self.vehicle.get_traffic_light_state()
            traffic_light = self.vehicle.get_traffic_light()
            
            # Check if vehicle is running a red light
            if current_state == carla.TrafficLightState.Red and traffic_light is not None:
                # Get vehicle's current speed to confirm it's moving
                velocity = self.vehicle.get_velocity()
                speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                
                # Consider it a violation if moving faster than 1 m/s through red light
                if speed > 1.0:
                    # Get junction ID to prevent duplicate violations
                    junction_id = None
                    if hasattr(traffic_light, 'get_transform'):
                        # Try to get junction ID from traffic light location
                        traffic_light_location = traffic_light.get_transform().location
                        waypoint = self.world.get_map().get_waypoint(traffic_light_location)
                        if waypoint and waypoint.is_junction:
                            junction_id = waypoint.junction_id
                    
                    # If we can't get junction ID, use traffic light ID as fallback
                    if junction_id is None:
                        junction_id = traffic_light.id
                    
                    # Check if this is the same violation as the last one
                    if not self.junction_id_stack or self.junction_id_stack[-1] != junction_id:
                        # Add to stack and limit stack size to prevent memory issues
                        self.junction_id_stack.append(junction_id)
                        if len(self.junction_id_stack) > 10:  # Keep only last 10 violations
                            self.junction_id_stack.pop(0)
                        
                        violation_data = {
                            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                            'vehicle_id': self.vehicle.id,
                            'location': self.vehicle.get_location(),
                            'speed': speed,
                            'traffic_light_id': traffic_light.id if traffic_light else None,
                            'junction_id': junction_id,
                            'violation_type': 'red_light_violation'
                        }
                        
                        self.violation_list.append(violation_data)
                        print(f"RED LIGHT VIOLATION DETECTED")
                        print(f"Time: {violation_data['timestamp']}")
                        print(f"Vehicle ID: {violation_data['vehicle_id']}")
                        print(f"Location: {violation_data['location']}")
                        print(f"Speed: {violation_data['speed']:.2f} m/s")
                        print(f"Traffic Light ID: {violation_data['traffic_light_id']}")
                        print(f"Junction ID: {violation_data['junction_id']}")
                        print("-" * 50)
                        
                        return True
            
            return False
            
        except Exception as e:
            print(f"Error checking red light violation: {e}")
            return False

    def monitor_violations(self, duration=60):
        """
        Monitor the vehicle for violations for a specified duration.
        
        Args:
            duration (int): Monitoring duration in seconds
        """
        if not self.vehicle:
            print("No vehicle available for monitoring")
            return
            
        print(f"Starting violation monitoring for {duration} seconds...")
        print("Press Ctrl+C to stop monitoring early")
        
        self.monitoring = True
        start_time = time.time()
        
        try:
            while self.monitoring and (time.time() - start_time) < duration:
                # Check for red light violations
                self.check_red_light_violation()
                
                # Sleep to avoid excessive CPU usage
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user")
        finally:
            self.monitoring = False
            print(f"\nMonitoring completed. Total violations detected: {len(self.violation_list)}")

    def get_violation_summary(self):
        """
        Print a summary of all detected violations.
        """
        if not self.violation_list:
            print("No violations detected.")
            return
            
        print(f"\n=== VIOLATION SUMMARY ===")
        print(f"Total violations: {len(self.violation_list)}")
        print("=" * 30)
        
        for i, violation in enumerate(self.violation_list, 1):
            print(f"Violation #{i}:")
            print(f"  Time: {violation['timestamp']}")
            print(f"  Type: {violation['violation_type']}")
            print(f"  Vehicle ID: {violation['vehicle_id']}")
            print(f"  Speed: {violation['speed']:.2f} m/s")
            print(f"  Location: {violation['location']}")
            print(f"  Traffic Light ID: {violation['traffic_light_id']}")
            print(f"  Junction ID: {violation['junction_id']}")
            print("-" * 20)

    def cleanup(self):
        """
        Clean up resources and destroy the vehicle.
        """
        if self.vehicle:
            try:
                self.vehicle.destroy()
                print("Vehicle destroyed successfully")
            except Exception as e:
                print(f"Error destroying vehicle: {e}")

def main():
    """
    Main function to run the red light violation monitor.
    """
    monitor = ViolationMonitor()
    
    try:
        # Spawn a vehicle
        if not monitor.spawn_vehicle():
            print("Failed to spawn vehicle. Exiting.")
            return
        
        # Enable autopilot
        if not monitor.enable_autopilot():
            print("Failed to enable autopilot. Exiting.")
            return
        
        # Monitor for violations (60 seconds by default)
        monitor.turn_all_lights_red(True)
        monitor.monitor_violations(duration=300)
        monitor.turn_all_lights_red(False)
        
        # Print violation summary
        monitor.get_violation_summary()
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean up
        monitor.cleanup()

if __name__ == "__main__":
    main()