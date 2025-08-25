#!/usr/bin/env python3

from PIL import Image, ImageDraw
import os

def create_three_lane_course():
    # Load the original course image
    original_path = "/home/rokey1/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace_2020/course/materials/textures/course.png"
    
    # Create backup
    backup_path = original_path.replace(".png", "_backup.png")
    if not os.path.exists(backup_path):
        os.system(f"cp '{original_path}' '{backup_path}'")
        print(f"Created backup: {backup_path}")
    
    # Load image
    img = Image.open(original_path)
    img = img.convert("RGBA")
    width, height = img.size
    
    # Create drawing context
    draw = ImageDraw.Draw(img)
    
    # Define colors
    yellow = (255, 255, 0, 255)  # Lane lines
    white = (255, 255, 255, 255)  # Track borders
    
    # Function to draw a thick line
    def draw_thick_line(draw, start, end, color, width=3):
        for i in range(width):
            offset = i - width//2
            draw.line([(start[0]+offset, start[1]), (end[0]+offset, end[1])], fill=color, width=1)
            draw.line([(start[0], start[1]+offset), (end[0], end[1]+offset)], fill=color, width=1)
    
    # Trace the original yellow line and create parallel lines
    # This is a simplified approach - we'll add lane dividers
    
    # Main track sections - adding lane dividers
    # Top straight section
    draw_thick_line(draw, (270, 40), (470, 40), yellow, 2)  # Upper lane divider
    draw_thick_line(draw, (270, 60), (470, 60), yellow, 2)  # Lower lane divider
    
    # Right vertical section  
    draw_thick_line(draw, (470, 40), (470, 260), yellow, 2)  # Left lane divider
    draw_thick_line(draw, (490, 40), (490, 260), yellow, 2)  # Right lane divider
    
    # Bottom horizontal section
    draw_thick_line(draw, (280, 440), (400, 440), yellow, 2)  # Upper lane divider
    draw_thick_line(draw, (280, 460), (400, 460), yellow, 2)  # Lower lane divider
    
    # Left vertical section
    draw_thick_line(draw, (50, 100), (50, 400), yellow, 2)   # Left lane divider
    draw_thick_line(draw, (70, 100), (70, 400), yellow, 2)   # Right lane divider
    
    # Curves - approximate positions for lane dividers
    # Top-left curve
    for angle in range(0, 90, 5):
        import math
        x1 = int(100 + 30 * math.cos(math.radians(angle)))
        y1 = int(80 + 30 * math.sin(math.radians(angle)))
        x2 = int(100 + 50 * math.cos(math.radians(angle)))
        y2 = int(80 + 50 * math.sin(math.radians(angle)))
        draw.point((x1, y1), fill=yellow)
        draw.point((x2, y2), fill=yellow)
    
    # Top-right curve  
    for angle in range(90, 180, 5):
        import math
        x1 = int(420 + 30 * math.cos(math.radians(angle)))
        y1 = int(80 + 30 * math.sin(math.radians(angle)))
        x2 = int(420 + 50 * math.cos(math.radians(angle)))
        y2 = int(80 + 50 * math.sin(math.radians(angle)))
        draw.point((x1, y1), fill=yellow)
        draw.point((x2, y2), fill=yellow)
    
    # Save the modified image
    img.save(original_path)
    print(f"Created 3-lane course texture: {original_path}")
    print("The course now has 3 parallel lanes with yellow lane dividers")

if __name__ == "__main__":
    create_three_lane_course()