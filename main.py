import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from openal import *
import time
import math
import os
from pynput import keyboard
from threading import Thread
import pygame
import random
import math
import sys
from openal.al import alSourcei, AL_TRUE, AL_SOURCE_RELATIVE
from std_msgs.msg import Float32, Int32

imu_subscriber = None
pygame.mixer.init()
breathing_sound = pygame.mixer.Sound("sounds/breathing.wav")
shoot_sound = pygame.mixer.Sound("sounds/gunshot.wav")
breathing_sound.set_volume(0.3)  # Optional: adjust volume

bite_sound = pygame.mixer.Sound("sounds/bite.wav")
bite_sound.set_volume(0.5)  # Optional volume

os.environ["ALSOFT_LOGLEVEL"] = "3"
os.environ["ALSOFT_HRTF_MODE"] = "true"

# angle_deg = 0
player_angle = 0
enemies = []
enemy_sources = [] 
running = True
last_spawn_time = 0

WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
PLAYER_SIZE = 40
PLAYER_HEALTH = 100
ENEMY_SIZE = 50
SHOT_ANGLE_RANGE = 6  # Degrees
ENEMY_SPAWN_INTERVAL = 10000  # milliseconds
FPS = 60

def on_press(key):
    global player_angle, running
    try:
        if key == keyboard.Key.left:
             player_angle = (player_angle - 5) % 360
        elif key == keyboard.Key.right:
             player_angle = (player_angle + 5) % 360
        elif key == keyboard.Key.esc:
             running = False
    except:
        pass


def input_thread():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

def spawn_enemy():
    if len(enemies) >= 5:
        return  # Limit to 5 enemies on screen

    angle = random.uniform(0, 360)
    rad = math.radians(angle)
    distance = 200  # Radius of the spawn circle
    x = CENTER[0] + distance * math.cos(rad)
    y = CENTER[1] - distance * math.sin(rad)

    # Append enemy and create sound
    enemies.append({
    'x': x,
    'y': y,
    'angle': angle,
    'last_bite_time': pygame.time.get_ticks()
})

    # Create and configure OpenAL sound source
    source = oalOpen("sounds/zombie_mono.wav")
    source.set_looping(True)
    source.set_gain(1.0)
    z = -5  # z-axis to simulate depth in audio
    source.set_position(((x - CENTER[0]) / 100, 0, (CENTER[1] - y) / 100))
    source.play()
    enemy_sources.append(source)

def update_enemies():
    global PLAYER_HEALTH
    BITE_INTERVAL = 4000  # ms
    STOP_DISTANCE = 60  # px
    SPEED = 0.1  # pixels per frame

    current_time = pygame.time.get_ticks()

    for i, enemy in enumerate(enemies):
        dx = CENTER[0] - enemy['x']
        dy = CENTER[1] - enemy['y']
        distance = math.hypot(dx, dy)

        if distance > STOP_DISTANCE:
            enemy['x'] += SPEED * dx / distance
            enemy['y'] += SPEED * dy / distance
            enemy_sources[i].set_position(((enemy['x'] - CENTER[0]) / 100, 0, (CENTER[1] - enemy['y']) / 100))
        else:
            # Close enough to bite
            if current_time - enemy['last_bite_time'] >= BITE_INTERVAL:
                bite_sound.play()
                PLAYER_HEALTH -= 10
                enemy['last_bite_time'] = current_time


def draw_spawn_circle(screen):
    pygame.draw.circle(screen, (50, 50, 255), CENTER, 200, 1)

# --- Pygame Drawing & Logic ---
def draw_quadrants(screen):
    pygame.draw.line(screen, (100, 100, 100), (WIDTH//2, 0), (WIDTH//2, HEIGHT), 2)
    pygame.draw.line(screen, (100, 100, 100), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 2)

def draw_player(screen, angle):
    rad = math.radians(angle)
    tip = (CENTER[0] + PLAYER_SIZE * math.cos(rad), CENTER[1] - PLAYER_SIZE * math.sin(rad))
    left = (CENTER[0] + PLAYER_SIZE * math.cos(rad + 2.5), CENTER[1] - PLAYER_SIZE * math.sin(rad + 2.5))
    right = (CENTER[0] + PLAYER_SIZE * math.cos(rad - 2.5), CENTER[1] - PLAYER_SIZE * math.sin(rad - 2.5))
    pygame.draw.polygon(screen, (0, 255, 0), [tip, left, right])

def draw_enemies(screen):
    for enemy in enemies:
        pygame.draw.rect(screen, (255, 0, 0), pygame.Rect(enemy['x'] - ENEMY_SIZE//2, enemy['y'] - ENEMY_SIZE//2, ENEMY_SIZE, ENEMY_SIZE))

def draw_shooting_cone(screen, angle):
    start_angle = angle - SHOT_ANGLE_RANGE
    end_angle = angle + SHOT_ANGLE_RANGE
    radius = 300  # Length of the shooting guide lines

    for a in [start_angle, end_angle]:
        rad = math.radians(a)
        end_x = CENTER[0] + radius * math.cos(rad)
        end_y = CENTER[1] - radius * math.sin(rad)
        pygame.draw.line(screen, (255, 255, 0), CENTER, (end_x, end_y), 2)


def display_imu_data(screen, angle, PLAYER_HEALTH, font):
    text = font.render(f"IMU Angle: {angle:.2f}°", True, (255, 255, 255))
    screen.blit(text, (20, 20))

    textHealth = font.render(f"Health: {PLAYER_HEALTH}", True, (255, 255, 255))
    screen.blit(textHealth, (60, 60))

def shoot(angle):
    global enemies, enemy_sources
    new_enemies = []
    new_sources = []

    for i, enemy in enumerate(enemies):
        dx = enemy['x'] - CENTER[0]
        dy = CENTER[1] - enemy['y']  # Inverted y-axis
        distance = math.hypot(dx, dy)

        # Calculate enemy's angle relative to the player
        enemy_angle = math.degrees(math.atan2(dy, dx))
        if enemy_angle < 0:
            enemy_angle += 360
        diff = abs(enemy_angle - angle) % 360
        if diff > 180:
            diff = 360 - diff

        if diff <= SHOT_ANGLE_RANGE or (diff <= 45):
            print("Enemy hit at angle:", enemy_angle, "Distance:", distance)
            enemy_sources[i].stop()
            continue 
        
        new_enemies.append(enemy)
        new_sources.append(enemy_sources[i])

    enemies = new_enemies
    enemy_sources = new_sources


# --- Pygame Thread ---
def pygame_thread_fn():
    global player_angle, running, last_spawn_time, PLAYER_HEALTH

    pygame.init()
    breathing_sound.play(-1)  # -1 means loop indefinitely

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("IMU Visualization & Enemy Quadrants")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 36)
    game_over_font = pygame.font.SysFont(None, 72)

    game_over = False

    while running:
        screen.fill((0, 0, 0))

        if PLAYER_HEALTH <= 0:
            game_over = True

        if not game_over:
            draw_quadrants(screen)
            draw_shooting_cone(screen, player_angle)
            draw_player(screen, player_angle)
            draw_spawn_circle(screen)
            update_enemies()
            draw_enemies(screen)
            display_imu_data(screen, player_angle, PLAYER_HEALTH, font)

            current_time = pygame.time.get_ticks()
            if current_time - last_spawn_time >= ENEMY_SPAWN_INTERVAL:
                spawn_enemy()
                last_spawn_time = current_time
        else:
            # Display Game Over
            text = game_over_font.render("GAME OVER", True, (255, 0, 0))
            text_rect = text.get_rect(center=(WIDTH // 2, HEIGHT // 2 - 40))
            screen.blit(text, text_rect)
            global imu_subscriber
            imu_subscriber = ImuSubscriber()
            rclpy.shutdown()
            oalQuit()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif game_over and event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.KEYDOWN:
                 if event.key == pygame.K_SPACE:
                     shoot_sound.play()
                     shoot(player_angle)

        keys = pygame.key.get_pressed()
        # if keys[pygame.K_LEFT]:
        #     player_angle = (player_angle + 3) % 360
        # if keys[pygame.K_RIGHT]:
        #     player_angle = (player_angle - 3) % 360

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    for src in enemy_sources:
            src.stop()

# --- ROS2 Node ---
class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.motor_pub = self.create_publisher(Int32, '/motor', 10)

        self.motor_timer = self.create_timer(0.1, self.check_aiming_cone)  # 10 Hz
        
        self.subscription = self.create_subscription(
            Float32,               # Change this to Int32 if needed
            '/yaw_angle',
            self.listener_callback,
            10
        )

        self.switch_subscription = self.create_subscription(
            Int32,               # Change this to Int32 if needed
            '/limit_switch',
            self.switch_callback,
            10
        )

    def listener_callback(self, msg):
        global player_angle
        player_angle = msg.data

    def switch_callback(self, msg):
         if msg.data == 0:
            shoot_sound.play()
            shoot(player_angle)
    
    def check_aiming_cone(self):
        global enemies, player_angle
        enemy_in_cone = False

        for enemy in enemies:
            dx = enemy['x'] - CENTER[0]
            dy = CENTER[1] - enemy['y']
            enemy_angle = math.degrees(math.atan2(dy, dx))
            if enemy_angle < 0:
                enemy_angle += 360

            diff = abs(enemy_angle - player_angle) % 360
            if diff > 180:
                diff = 360 - diff

            if diff <= SHOT_ANGLE_RANGE:
                enemy_in_cone = True
                break

        msg = Int32()
        msg.data = 1 if enemy_in_cone else 0   # 1 = vibrate, 0 = stop
        self.motor_pub.publish(msg)

# --- Main Execution ---
def main(args=None):
    global running
    rclpy.init(args=args)

    # Start keyboard input thread
    Thread(target=input_thread, daemon=True).start()

    # Start pygame thread
    Thread(target=pygame_thread_fn, daemon=True).start()

    # Start OpenAL source
    listener = Listener()
    listener.set_position((0, 0, 0))

    # ROS2 node
    imu_subscriber = ImuSubscriber()
    ros_thread = Thread(target=rclpy.spin, args=(imu_subscriber,), daemon=True)
    ros_thread.start()

    # Orientation updater
    try:
        while running:
            angle_rad = math.radians(player_angle - 90)
            forward = (math.sin(angle_rad), 0, -math.cos(angle_rad))
            up = (0, 1, 0)
            listener.set_orientation(forward + up)
            time.sleep(0.05)
    finally:
        for src in enemy_sources:
            src.stop()
        imu_subscriber.destroy_node()
        rclpy.shutdown()
        oalQuit()
        sys.exit()

if __name__ == '__main__':
    main()
