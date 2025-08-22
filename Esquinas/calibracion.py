import numpy as np

def R_from_euler_and_FRD_to_world(roll, pitch, yaw):
    # roll, pitch, yaw en radianes
    cr = np.cos(roll); sr = np.sin(roll)
    cp = np.cos(pitch); sp = np.sin(pitch)
    cy = np.cos(yaw);   sy = np.sin(yaw)

    R = np.array([
      [ cy*cp,            -sr*sp*cy + sy*cr,    -sr*sy - sp*cr*cy ],
      [ sy*cp,            -sr*sy*sp - cr*cy,     sr*cy - sy*sp*cr ],
      [ -sp,              -sr*cp,                -cr*cp          ]
    ])
    return R

# Uso:
# p_b = np.array([x_forward, y_right, z_down])  # vector FRD
# t_w = drone_position_from_orb (3-vector)
# R = R_from_euler_and_FRD_to_world(roll, pitch, yaw)
# p_w = t_w + R.dot(p_b)
