import numpy as np
import cv2
import math

# -------------------------
# Util: rotación world <- body (igual que antes)
# -------------------------
def rot_from_euler_deg(roll_deg, pitch_deg, yaw_deg):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    Rx = np.array([[1,0,0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll),  np.cos(roll)]])
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [0, 0, 1]])
    # corrección FRD -> FLU si la venías usando (la mantenemos para coherencia)
    S = np.diag([1, -1, -1])
    R = Rz @ Ry @ Rx @ S
    return R

# -------------------------
# Opción A: backproject Z=D_center y ajustar plano (SVD)
# -------------------------
def option_A_plane_from_4corners(pts_img, K, D_center,
                                 drone_pos_world, roll_deg, pitch_deg, yaw_deg):
    """
    pts_img: np.array (4,2) pixel coords (any order ok for SVD)
    K: intrinsics 3x3
    D_center: estimated distance to center (meters)
    drone_pos_world: [x,y,z]
    roll_deg, pitch_deg, yaw_deg: orientation of drone in degrees (world<-body)
    Returns dict con normal_cam, normal_world, center_cam, center_world, pts_cam, pts_world, angle_deg
    """
    pts_img = np.asarray(pts_img).reshape(-1,2)
    fx = K[0,0]; fy = K[1,1]; cx = K[0,2]; cy = K[1,2]

    # Backproject each corner with Z = D_center
    pts_cam = []
    for (u,v) in pts_img:
        X = (u - cx) / fx * D_center
        Y = (v - cy) / fy * D_center
        Z = D_center
        pts_cam.append([X, Y, Z])
    pts_cam = np.array(pts_cam)   # (4,3)

    # Fit plane by SVD
    centroid = pts_cam.mean(axis=0)
    Xc = pts_cam - centroid
    _, _, Vt = np.linalg.svd(Xc, full_matrices=False)
    normal_cam = Vt[-1, :]
    normal_cam = normal_cam / np.linalg.norm(normal_cam)

    # Ensure normal orientation: make it point "towards" camera (camera at origin)
    # vector from plane centroid to camera = -centroid
    if np.dot(normal_cam, -centroid) < 0:
        normal_cam = -normal_cam

    # Center of rectangle: backproject image centroid at Z=D_center
    u_center = pts_img[:,0].mean()
    v_center = pts_img[:,1].mean()
    center_cam = np.array([ (u_center - cx)/fx * D_center,
                            (v_center - cy)/fy * D_center,
                            D_center ])

    # Transform to world
    R_wb = rot_from_euler_deg(roll_deg, pitch_deg, yaw_deg)
    t_w = np.asarray(drone_pos_world).reshape(3,)
    pts_world = (R_wb @ pts_cam.T).T + t_w
    center_world = (R_wb @ center_cam) + t_w
    normal_world = R_wb @ normal_cam
    normal_world = normal_world / np.linalg.norm(normal_world)

    # Angle between normal_world and vector drone->center_world
    v = center_world - t_w
    if np.linalg.norm(v) < 1e-8:
        angle_deg = 0.0
    else:
        v_u = v / np.linalg.norm(v)
        angle_rad = np.arccos(np.clip(np.dot(normal_world, v_u), -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

    return {
        'method': 'A_backproj_SVD',
        'normal_cam': normal_cam,
        'normal_world': normal_world,
        'center_cam': center_cam,
        'center_world': center_world,
        'pts_cam': pts_cam,
        'pts_world': pts_world,
        'angle_deg': angle_deg
    }

# -------------------------
# Opción B: homografía + decomposeHomographyMat
# -------------------------
def option_B_homography_decompose(pts_img, K, door_W, door_H,
                                  drone_pos_world, roll_deg, pitch_deg, yaw_deg):
    """
    pts_img: np.array (4,2) pixel coords corresponding to rectangle model points
             Order must match model order. We'll use model: TL, TR, BR, BL.
    K: intrinsics 3x3
    door_W, door_H: real dimensions (meters) of the rectangle door (use actual W,H)
    pose: drone position + orientation as before
    Returns dict with chosen solution and candidates
    """
    # Model rectangle points in plane coordinates (Z=0), in meters
    # We'll take origin at top-left (0,0), x->right, y->down
    pts_model = np.array([[0.0, 0.0],
                          [door_W, 0.0],
                          [door_W, door_H],
                          [0.0, door_H]], dtype=np.float32)

    pts_img = np.asarray(pts_img).reshape(-1,2).astype(np.float32)

    # Compute homography from model -> image (H maps model -> image)
    Hmat, mask = cv2.findHomography(pts_model, pts_img, method=0)
    if Hmat is None:
        raise RuntimeError("Homography failed (H is None)")

    # Decompose homography: returns lists of R, t, normals (all in camera frame)
    # Note: cv2.decomposeHomographyMat returns (num, Rs, Ts, Ns)
    n_solutions, Rs, Ts, Ns = cv2.decomposeHomographyMat(Hmat, K)

    # Prepare centroid in camera frame (via backprojection using D ~ unknown)
    # We can compute centroid in image and backproject a ray; but to test normal pointing,
    # we need an approximate centroid in camera coords. We'll pick Z=1 for direction only.
    # Better: compute approximate plane point by solving for scale using door real size:
    # We can pick the model centroid (W/2,H/2), map via H to image, then backproject ray.
    model_centroid = np.array([door_W/2.0, door_H/2.0, 1.0]).reshape(3,1)
    img_centroid_h = Hmat @ model_centroid
    img_centroid_h = img_centroid_h.flatten()
    img_centroid = (img_centroid_h[:2] / img_centroid_h[2]).reshape(2,)

    # Backproject image centroid to camera ray direction (Z=1)
    fx = K[0,0]; fy = K[1,1]; cx = K[0,2]; cy = K[1,2]
    u_c, v_c = img_centroid
    ray_cam = np.array([(u_c - cx)/fx, (v_c - cy)/fy, 1.0])
    ray_cam = ray_cam / np.linalg.norm(ray_cam)

    # We'll choose the candidate solution whose normal points roughly towards camera (dot with -ray?),
    # and whose translation has positive Z in camera frame (plane in front).
    chosen = None
    candidates = []
    for i in range(n_solutions):
        R_cam_model = Rs[i]      # rotation from model frame (plane) to camera frame
        t_cam = Ts[i].reshape(3,)  # translation in camera frame
        n_cam = Ns[i].reshape(3,)  # plane normal in camera frame (unit?)
        # Normalize
        n_cam = n_cam / np.linalg.norm(n_cam)
        # Evaluate heuristics:
        #  - plane should be in front: t_cam[2] > 0 (centroid distance along camera Z)
        #  - normal should point towards camera: dot(n_cam, -ray_cam) > 0
        score = 0.0
        if t_cam[2] > 0:
            score += 1.0
        if np.dot(n_cam, -ray_cam) > 0:
            score += 1.0
        candidates.append({'i': i, 'R': R_cam_model, 't': t_cam, 'n': n_cam, 'score': score})

    # Pick best candidate by score, break ties by larger t_cam[2]
    if len(candidates) == 0:
        raise RuntimeError("No homography decomposition candidates")
    candidates = sorted(candidates, key=lambda c: (c['score'], c['t'][2]), reverse=True)
    best = candidates[0]

    # Best candidate is in camera frame: normal n_cam (pointing outward from plane)
    n_cam = best['n']
    t_cam = best['t']   # position of model origin in camera frame (approx)
    # We can compute center_cam in camera frame: transform model centroid (W/2,H/2,0)
    # model centroid in homogeneous: [W/2, H/2, 1] under H -> image, but we want its 3D position
    # Using best R,t: a point in model (x,y,0) maps to camera: p_cam = R * [x,y,0]^T + t
    model_cent = np.array([door_W/2.0, door_H/2.0, 0.0])
    center_cam = best['R'].dot(model_cent) + best['t']

    # Normalize normal orientation: make it point toward camera
    if np.dot(n_cam, -center_cam) < 0:
        n_cam = -n_cam

    # Transform normal and center to world
    # Note: Rs are camera<-model (i.e., rotation from model plane to camera). The normal is already in camera frame.
    R_wb = rot_from_euler_deg(roll_deg, pitch_deg, yaw_deg)
    t_w = np.asarray(drone_pos_world).reshape(3,)

    # pts in camera frame: we can map model corner points to camera using R,t and then to world
    pts_model_h = np.array([[0.0,0.0,0.0],
                            [door_W,0.0,0.0],
                            [door_W,door_H,0.0],
                            [0.0,door_H,0.0]])
    pts_cam = np.array([best['R'].dot(p) + best['t'] for p in pts_model_h])  # (4,3)
    pts_world = (R_wb @ pts_cam.T).T + t_w
    center_world = (R_wb @ center_cam) + t_w
    normal_world = R_wb @ n_cam
    normal_world = normal_world / np.linalg.norm(normal_world)

    v = center_world - t_w
    if np.linalg.norm(v) < 1e-8:
        angle_deg = 0.0
    else:
        v_u = v / np.linalg.norm(v)
        angle_rad = np.arccos(np.clip(np.dot(normal_world, v_u), -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

    return {
        'method': 'B_homography_decompose',
        'H': Hmat,
        'candidates': candidates,
        'chosen_idx': best['i'],
        'normal_cam': n_cam,
        'normal_world': normal_world,
        'center_cam': center_cam,
        'center_world': center_world,
        'pts_cam': pts_cam,
        'pts_world': pts_world,
        'angle_deg': angle_deg
    }

# -------------------------
# Ejemplo comparativo
# -------------------------
if __name__ == "__main__":
    # Intrínsecos ejemplo (sustituye por los tuyos)
    K = np.array([[617.0, 0.0, 320.0],
                  [0.0, 617.0, 240.0],
                  [0.0,   0.0,   1.0]])
    # Ejemplo de 4 esquinas detectadas en pixels (orden TL,TR,BR,BL)
    pts_img_ordered = np.array([[200,150],[440,150],[440,330],[200,330]])
    # Distancia estimada al centro (m) que ya calculas con tu método
    D_center = 2.5
    # Medidas reales de la puerta (W,H) en metros
    door_W, door_H = 0.90, 2.0
    # Pose del dron (ejemplo)
    drone_pos = [2.0, 3.0, 1.0]
    roll, pitch, yaw = 5.0, -2.0, 30.0

    resA = option_A_plane_from_4corners(pts_img_ordered, K, D_center,
                                        drone_pos, roll, pitch, yaw)
    resB = option_B_homography_decompose(pts_img_ordered, K, door_W, door_H,
                                         drone_pos, roll, pitch, yaw)

    print("=== Opción A (backproj+SVD) ===")
    print("normal_cam:", resA['normal_cam'])
    print("normal_world:", resA['normal_world'])
    print("center_world:", resA['center_world'])
    print("angle_deg:", resA['angle_deg'])
    print()
    print("=== Opción B (homography) ===")
    print("normal_cam:", resB['normal_cam'])
    print("normal_world:", resB['normal_world'])
    print("center_world:", resB['center_world'])
    print("angle_deg:", resB['angle_deg'])
