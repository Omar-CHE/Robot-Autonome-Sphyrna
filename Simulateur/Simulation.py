import pygame
import math
from Robot import Robot, SENSOR_COUNT

CIRCUIT_NAME = ["circuit.png","circuit1.png"]
CIRCUIT_NUM = 0

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    
    # Load the circuit track
    track = pygame.image.load(CIRCUIT_NAME[CIRCUIT_NUM]).convert()
    track_width, track_height = track.get_size()

    robot = Robot(80, 80)
    robot.angle = math.pi / 2
    running = True
    dragging = False
    finished = False

    while running:
        mouse_pos = pygame.mouse.get_pos()
        dt = clock.get_time()  # Delta time in milliseconds

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                dist = math.hypot(mouse_pos[0] - robot.x, mouse_pos[1] - robot.y)
                if dist < 30: 
                    dragging = True
                    finished = False
            
            if event.type == pygame.MOUSEBUTTONUP:
                dragging = False
                robot.last_error = 0
                robot.no_line_duration = 0

        if dragging:
            robot.x, robot.y = mouse_pos
            robot.left_speed = 0
            robot.right_speed = 0

        elif not finished:
            # Read sensors and update robot logic
            on_line = robot.read_sensors(track, track_width, track_height)
            robot.update_logic(on_line, dt)
            
            # Check for finish line
            if robot.check_finish_line():
                if robot.finish_line_start_time is None:
                    robot.finish_line_start_time = pygame.time.get_ticks()
                else:
                    elapsed = pygame.time.get_ticks() - robot.finish_line_start_time
                    # Finish line must be held for 200+ ms to count
                    if elapsed >= 200:
                        finished = True
            else:
                robot.finish_line_start_time = None
            
            # Move the robot
            robot.move()

        # Render the track
        screen.blit(track, (0, 0))
        
        # Update cursor to hand when hovering over robot
        dist = math.hypot(mouse_pos[0] - robot.x, mouse_pos[1] - robot.y)
        if dist < 30:
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_HAND)
        else:
            pygame.mouse.set_cursor(pygame.SYSTEM_CURSOR_ARROW)

        # Draw the robot
        robot_color = (100, 100, 255) if dragging else (0, 0, 255)
        pygame.draw.circle(screen, robot_color, (int(robot.x), int(robot.y)), 12)
        
        # Draw direction indicator (robot heading)
        head_x = robot.x + 15 * math.cos(robot.angle)
        head_y = robot.y + 15 * math.sin(robot.angle)
        pygame.draw.line(screen, (255, 255, 0), (robot.x, robot.y), (head_x, head_y), 2)
        
        # Draw sensors with larger radius
        for i in range(SENSOR_COUNT):
            sx, sy = robot.get_sensor_pos(i)
            s_color = (0, 255, 0) if robot.sensor_values[i] < 750 else (200, 200, 200)
            pygame.draw.circle(screen, s_color, (int(sx), int(sy)), 3)
            # Also draw a circle showing detection radius
            pygame.draw.circle(screen, (100, 100, 100), (int(sx), int(sy)), 3, 1)

        # Draw debug info
        font_small = pygame.font.Font(None, 20)
        font_large = pygame.font.Font(None, 28)
        
        pos_text = font_small.render(f"Pos: {robot.calculate_position():.0f}", True, (100, 255, 100))
        error_text = font_small.render(f"Error: {robot.last_error:.0f}", True, (100, 255, 100))
        speed_text = font_small.render(f"L:{robot.left_speed:.1f} R:{robot.right_speed:.1f}", True, (100, 255, 100))
        
        # # Sensor readings
        sensor_str = f"Sensors: {[int(v) for v in robot.sensor_values]}"
        sensor_text = font_small.render(sensor_str, True, (255, 200, 0))
        
        # Finish line status
        finish_count = sum(1 for v in robot.sensor_values if v < 500)
        finish_text = font_small.render(f"On line: {finish_count}/5 sensors", True, (100, 255, 100))
        
        screen.blit(pos_text, (10, 10))
        screen.blit(error_text, (10, 32))
        screen.blit(speed_text, (10, 54))
        screen.blit(sensor_text, (10, 76))
        screen.blit(finish_text, (10, 98))
        
        # Finish line message
        if finished:
            finish_msg = font_large.render(f"FINISHED!", True, (0, 255, 0))
            screen.blit(finish_msg, (250, 280))

        pygame.display.flip()
        
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    main()