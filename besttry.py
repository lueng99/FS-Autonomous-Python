import sys
import os
import time
import math
import numpy
import matplotlib.pyplot as plt

# --- CONFIGURACIÓN DE LA RUTA ---
sys.path.append(r"C:\Users\alvar\Music\FS\Formula-Student-Driverless-Simulator-master\python")
import fsds

# --- CONEXIÓN AL SIMULADOR ---
client = fsds.FSDSClient()
client.confirmConnection()
print("Conectado. Ajuste v1.1: Más velocidad, frenos más suaves...")

client.enableApiControl(True)

# --- CONSTANTES DEL COCHE ---
max_throttle = 0.5      
target_speed = 7.0      # [AJUSTE] Subimos velocidad objetivo
max_steering = 0.4      
cones_range_cutoff = 8  

# --- FUNCIONES AUXILIARES ---

def pointgroup_to_cone(group):
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))

def find_cones():
    lidardata = client.getLidarData(lidar_name = 'Lidar')
    if len(lidardata.point_cloud) < 3: return []

    points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))

    current_group = []; cones = []
    for i in range(1, len(points)):
        distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])
        if distance_to_last_point < 0.1:
            current_group.append({'x': points[i][0], 'y': points[i][1]})
        else:
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff:
                    cones.append(cone)
                current_group = []
    return cones

def calculate_steering(cones):
    average_y = 0
    for cone in cones:
        average_y += cone['y']
    
    if len(cones) > 0:
        average_y = average_y / len(cones)

    kp = 0.25 
    
    steering_angle = -(average_y * kp)
    steering_angle = max(min(steering_angle, 1.0), -1.0)
    
    return steering_angle

def get_velocity_and_throttle():
    """Devuelve tanto la velocidad real como el comando de acelerador"""
    gps = client.getGpsData()
    velocity = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))
    
    throttle = max_throttle * max(1 - velocity / target_speed, 0)
    return velocity, throttle

# --- BUCLE PRINCIPAL ---
try:
    while True:
        plt.pause(0.05)
        plt.clf()
        plt.axis([-cones_range_cutoff, cones_range_cutoff, -2, cones_range_cutoff])
        
        cones = find_cones()
        if len(cones) == 0:
            continue

        # 1. Calcular valores base
        steering_val = calculate_steering(cones)
        velocity, throttle_val = get_velocity_and_throttle() # Obtenemos velocidad real
        brake_val = 0.0 

        # 2. Lógica de Frenado MEJORADA (Más suave)
        abs_steer = abs(steering_val)
        
        if abs_steer > 0.3:
            throttle_val = 0.0  # Soltar gas
            
            # [AJUSTE CLAVE] Solo frenamos si vamos "rápido" (> 4 m/s)
            # Si vamos lentos, dejamos que el coche fluya (coasting)
            if velocity > 4.0:
                brake_val = 0.05 # Freno mínimo (apenas tocarlo)
                
                # Si la curva es MUY cerrada
                if abs_steer > 0.5:
                    # [AJUSTE] Bajamos la intensidad máxima de 0.5 a 0.25
                    brake_val = 0.25 

        # 3. Enviar órdenes
        car_controls = fsds.CarControls()
        car_controls.steering = float(steering_val)
        car_controls.throttle = float(throttle_val)
        car_controls.brake = float(brake_val)
        
        client.setCarControls(car_controls)

        # 4. Dibujar
        for cone in cones:
            plt.scatter(x=-1*cone['y'], y=cone['x'])

except KeyboardInterrupt:
    print("Deteniendo el coche...")
    client.enableApiControl(False)