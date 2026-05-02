import math
# Constants from Arduino code
BASE_SPEED = 6 
Kp = 0.3 
Kd = 15
BASE_POSITION = 2000
RED_HERRING_THRESHOLD = 800 
SENSOR_COUNT = 5
SENSOR_SPACING = 10  # Distance between sensors (left-right)
SENSOR_FRONT_DIST = 30  # How far ahead of robot center to place sensors

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.last_error = 0
        self.no_line_duration = 0
        self.left_speed = 0
        self.right_speed = 0
        self.sensor_values = [0] * SENSOR_COUNT
        self.on_finish_line = False
        self.finish_line_start_time = None

    def get_sensor_pos(self, index):
        """Calculates the world position of a specific virtual sensor."""
        # Sensors are arranged perpendicular to robot heading
        # Sensor 0 on far left, Sensor 2 center, Sensor 4 far right
        offset = (2 - index) * SENSOR_SPACING
        
        sx = self.x + SENSOR_FRONT_DIST * math.cos(self.angle) - offset * math.sin(self.angle)
        sy = self.y + SENSOR_FRONT_DIST * math.sin(self.angle) + offset * math.cos(self.angle)
        return sx, sy

    def read_sensors(self, surface, surface_width, surface_height):
        """Simulates QTR sensors by reading pixel brightness in a small area."""
        on_line = False
        detection_radius = 5  # Larger detection area for each sensor
        
        for i in range(SENSOR_COUNT):
            sx, sy = self.get_sensor_pos(i)
            brightness_sum = 0
            sample_count = 0
            
            # Sample a small circle around the sensor position
            for dx in range(-detection_radius, detection_radius + 1):
                for dy in range(-detection_radius, detection_radius + 1):
                    if dx*dx + dy*dy > detection_radius*detection_radius:
                        continue
                    
                    px = int(sx + dx)
                    py = int(sy + dy)
                    
                    # Check bounds
                    if 0 <= px < surface_width and 0 <= py < surface_height:
                        try:
                            color = surface.get_at((px, py))
                            brightness = (color.r + color.g + color.b) / 3
                            brightness_sum += brightness
                            sample_count += 1
                        except:
                            pass
            
            # Calculate average brightness for this sensor
            if sample_count > 0:
                avg_brightness = brightness_sum / sample_count
                # Convert to 0-1000 scale (0=black line, 1000=white)
                val = (avg_brightness / 255) * 1000
                self.sensor_values[i] = val
                if val < 750:  # Dark/gray = on line (lowered from 500)
                    on_line = True
            else:
                self.sensor_values[i] = 1000  # White if out of bounds
        
        return on_line

    def calculate_position(self):
        """Calculate line position like QTR's readLineWhite() function.
        Returns a value 0-4000 where 2000 is the center (over middle sensor)."""
        weighted_sum = 0
        sum_value = 0
        
        for i in range(SENSOR_COUNT):
            # Invert the sensor value (dark line = low reading = high inverted value)
            inverted = 1000 - self.sensor_values[i]
            # Weight by position: sensor 0 -> 0, sensor 1 -> 1000, sensor 2 -> 2000, etc.
            weight = i * 1000
            weighted_sum += weight * inverted
            sum_value += inverted
        
        # Calculate position (0-4000)
        if sum_value > 0:
            position = weighted_sum / sum_value
        else:
            position = BASE_POSITION  # Default to center if no line detected
        
        return position

    def check_finish_line(self):
        """Check if robot is on finish line (4+ sensors detecting dark)."""
        on_line_count = sum(1 for v in self.sensor_values if v < 750)
        return on_line_count >= 4

    def update_logic(self, on_line, dt):
        """Update motor speeds based on line detection and sensor feedback."""
        if not on_line:
            self.no_line_duration += dt
            
            # If line lost for too long, back up to find it
            if self.no_line_duration > RED_HERRING_THRESHOLD:
                # Back up slowly to find the line again
                self.left_speed = -BASE_SPEED * 0.6
                self.right_speed = -BASE_SPEED * 0.6
            else:
                # Line just lost - keep moving forward slowly to find it
                # Don't go full speed to avoid overshooting
                self.left_speed = BASE_SPEED * 0.7
                self.right_speed = BASE_SPEED * 0.7
        else:
            self.no_line_duration = 0
            
            # Calculate position from sensors
            position = self.calculate_position()
            
            # PD controller
            error = position - BASE_POSITION
            correction = Kp * error + Kd * (error - self.last_error)
            
            # Constrain correction to allow responsive steering
            # Allow larger corrections for better line tracking (3x BASE_SPEED)
            max_correction = BASE_SPEED * 3
            correction = max(-max_correction, min(max_correction, correction))
            
            self.last_error = error
            
            # Apply correction to motor speeds
            self.left_speed = BASE_SPEED + correction
            self.right_speed = BASE_SPEED - correction

    def move(self):
        """Update robot position and angle based on motor speeds."""
        speed_scale = 0.15  # Speed scale factor
        v_left = self.left_speed * speed_scale
        v_right = self.right_speed * speed_scale
        
        # Update position FIRST
        velocity = (v_left + v_right) / 2
        self.x += velocity * math.cos(self.angle)
        self.y += velocity * math.sin(self.angle)
        
        # Update angle based on differential speed (differential drive model)
        # The turning rate should be proportional to speed difference and inversely proportional to wheel separation
        # Much reduced sensitivity to prevent oscillation
        wheel_separation = SENSOR_SPACING  # Distance between left and right wheels
        
        # Only allow turning if robot is actually moving
        if abs(velocity) > 0.01:
            # Reduce turning coefficient significantly (was dividing by 30, now by 150)
            turning_coefficient = 0.05  # Reduced turning sensitivity
            self.angle += (v_right - v_left) * turning_coefficient / wheel_separation
        
        # Normalize angle to 0-2π to prevent overflow
        self.angle = self.angle % (2 * math.pi)