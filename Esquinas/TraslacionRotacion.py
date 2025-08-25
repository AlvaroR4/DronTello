import numpy as np

def punto_cuerpo_a_mundo(roll_deg, pitch_deg, yaw_deg, pos_dron_mundo, punto_cuerpo):
    """
    Transforma un punto expresado en ejes cuerpo FRD (Forward, Right, Down)
    al sistema de coordenadas mundo usado por ORB-SLAM (X adelante, Y izquierda, Z arriba).

    Parámetros:
    -----------
    roll_deg, pitch_deg, yaw_deg : float
        Ángulos de orientación del dron en grados (roll, pitch, yaw).
        - roll: rotación sobre eje X cuerpo (forward)
        - pitch: rotación sobre eje Y cuerpo (right)
        - yaw: rotación sobre eje Z cuerpo (down)
    pos_dron_mundo : array-like (3,)
        Posición del dron en coordenadas mundo [x, y, z] (proporcionada por ORB-SLAM).
    punto_cuerpo : array-like (3,)
        Coordenadas del objeto detectado en ejes cuerpo FRD [x_forward, y_right, z_down].

    Retorna:
    --------
    punto_mundo : ndarray (3,)
        Coordenadas del punto en el sistema mundo de ORB-SLAM.
    """

    # 1) Conversión de grados a radianes
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    # 2) Matrices de rotación elementales
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    R_y = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # 3) Matriz de corrección FRD -> FLU (porque ORB usa Y izquierda, Z arriba)
    S = np.diag([1, -1, -1])

    # 4) Matriz de rotación cuerpo -> mundo
    R = R_z @ R_y @ R_x @ S

    # 5) Transformar el punto: primero rotación, luego traslación
    punto_cuerpo = np.array(punto_cuerpo).reshape(3,)
    pos_dron_mundo = np.array(pos_dron_mundo).reshape(3,)
    punto_mundo = pos_dron_mundo + R @ punto_cuerpo

    return punto_mundo


# ==========================
# EJEMPLO DE USO
# ==========================

# Supongamos:
# - El dron tiene una orientación roll=10°, pitch=5°, yaw=45°
# - El dron está en la posición (2, 3, 1) en el mundo ORB-SLAM
# - La cámara detecta un objeto en (x=1m adelante, y=0.5m derecha, z=-0.2m abajo) en ejes cuerpo FRD
roll, pitch, yaw = 10, 5, 45
pos_dron_mundo = [2, 3, 1]
punto_cuerpo = [1.0, 0.5, -0.2]

punto_mundo = punto_cuerpo_a_mundo(roll, pitch, yaw, pos_dron_mundo, punto_cuerpo)

print("Coordenadas del punto en mundo (ORB-SLAM):", punto_mundo)
