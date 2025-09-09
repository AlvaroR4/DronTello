#!/usr/bin/env python3
"""
calibrar_generar_yaml.py
- Busca imágenes en ./tello_calib_images/
- Calibra con un chessboard (por defecto 9x6 esquinas internas)
- Guarda imágenes de reproyección en ./reproj/
- Genera 2 YAMLs:
    - Tello_mono_calib_Camera.yaml   (keys Camera.fx, Camera.k1, ...)
    - Tello_mono_calib_Camera1.yaml  (keys Camera1.fx, Camera1.k1, ...)
"""

import cv2, glob, os, numpy as np, yaml, sys

# ---- Ajustes ----
IMAGE_FOLDER = "tello_calib_images"
INPUT_YAML = "gazebo_mono.yaml"   # opcional: base para llenar claves
OUT_YAML_CAMERA = "Tello_mono_calib_Camera.yaml"
OUT_YAML_CAMERA1 = "Tello_mono_calib_Camera1.yaml"
REPROJ_DIR = "reproj"
CHESSBOARD = (6,4)       # (cols, rows) esquinas internas: cambia si usas otro
SQUARE_SIZE = 0.034      # en metros (opcional, solo para objpoints escala)
MIN_GOOD_IMAGES = 8      # mínimo deseable es 15-40, pero se chequeará

os.makedirs(REPROJ_DIR, exist_ok=True)

# preparar puntos 3D del tablero
objp = np.zeros((CHESSBOARD[0]*CHESSBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1,2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []
img_shape = None

# buscar imágenes
imgs = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.png"))) + sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.jpg")))
if not imgs:
    print("No hay imágenes en", IMAGE_FOLDER)
    sys.exit(1)

for fname in imgs:
    img = cv2.imread(fname)
    if img is None:
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img_shape is None:
        img_shape = gray.shape[::-1]  # (width, height)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    if ret:
        # subpixel refine
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        objpoints.append(objp)
        print("Encontrado tablero en:", fname)
    else:
        print("No se encontró tablero en:", fname)

n_good = len(objpoints)
print(f"\nImágenes con tablero detectado: {n_good} / {len(imgs)}")

if n_good < MIN_GOOD_IMAGES:
    print(f"Advertencia: solo {n_good} imágenes válidas (recomiendo 20-40). Puedes continuar, pero la calibración puede ser pobre.")

# calibrar
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("\nRMS reprojection error:", ret)
print("Matriz cámara (mtx):\n", mtx)
print("Distorsión (dist):", dist.ravel())

# generar imágenes de reproyección (dibuja esquinas detectadas y proyectadas)
for i, fname in enumerate(imgs):
    img = cv2.imread(fname)
    if img is None: continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    if not ret:
        continue
    # usa el i-ésimo objpoint/imgpoint si existe, sino calcula proj
    imgpts = None
    if i < len(rvecs):
        # proyectar puntos de objp usando parámetros estimados
        rvec, tvec = rvecs[i], tvecs[i]
        proj, _ = cv2.projectPoints(objp, rvec, tvec, mtx, dist)
        imgpts = proj
    else:
        # fallback: proyectar con rvecs[0]
        proj, _ = cv2.projectPoints(objp, rvecs[0], tvecs[0], mtx, dist)
        imgpts = proj
    vis = img.copy()
    cv2.drawChessboardCorners(vis, CHESSBOARD, corners, True)
    # dibuja reproyectados (pequeños círculos)
    for p in imgpts.reshape(-1,2):
        cv2.circle(vis, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
    base = os.path.basename(fname)
    cv2.imwrite(os.path.join(REPROJ_DIR, "reproj_"+base), vis)

# --- crear dict YAML (formato Camera.*) ---
d = dist.ravel()
cam = {
    'File.version': "1.0",
    'Camera.type': "PinHole",
    'Camera.width': int(img_shape[0]),
    'Camera.height': int(img_shape[1]),
    'Camera.fps': 30,
    'Camera.RGB': 0,   # djitellopy/opencv -> BGR
    'Camera.fx': float(mtx[0,0]),
    'Camera.fy': float(mtx[1,1]),
    'Camera.cx': float(mtx[0,2]),
    'Camera.cy': float(mtx[1,2]),
    'Camera.k1': float(d[0]) if d.size>0 else 0.0,
    'Camera.k2': float(d[1]) if d.size>1 else 0.0,
    'Camera.p1': float(d[2]) if d.size>2 else 0.0,
    'Camera.p2': float(d[3]) if d.size>3 else 0.0,
    'Camera.k3': float(d[4]) if d.size>4 else 0.0,
    # parámetros ORB por defecto (puedes editar)
    'ORBextractor.nFeatures': 2000,
    'ORBextractor.scaleFactor': 1.2,
    'ORBextractor.nLevels': 8,
    'ORBextractor.iniThFAST': 20,
    'ORBextractor.minThFAST': 7
}
# --- crear dict YAML (formato Camera1.*) ---
cam1 = {'File.version': "1.0", 'Camera.type': "PinHole",
        'Camera.width': int(img_shape[0]), 'Camera.height': int(img_shape[1]),
        'Camera.fps': 30, 'Camera.RGB': 0,
        'Camera1.fx': float(mtx[0,0]), 'Camera1.fy': float(mtx[1,1]),
        'Camera1.cx': float(mtx[0,2]), 'Camera1.cy': float(mtx[1,2]),
        'Camera1.k1': float(d[0]) if d.size>0 else 0.0,
        'Camera1.k2': float(d[1]) if d.size>1 else 0.0,
        'Camera1.p1': float(d[2]) if d.size>2 else 0.0,
        'Camera1.p2': float(d[3]) if d.size>3 else 0.0,
        'Camera1.k3': float(d[4]) if d.size>4 else 0.0,
        'ORBextractor.nFeatures': 2000, 'ORBextractor.scaleFactor': 1.2,
        'ORBextractor.nLevels': 8, 'ORBextractor.iniThFAST': 20, 'ORBextractor.minThFAST': 7}

# guarda YAMLs
with open(OUT_YAML_CAMERA, 'w') as f:
    yaml.safe_dump(cam, f, sort_keys=False)
with open(OUT_YAML_CAMERA1, 'w') as f:
    yaml.safe_dump(cam1, f, sort_keys=False)

print("\nGuardado:", OUT_YAML_CAMERA, "y", OUT_YAML_CAMERA1)
print("Imágenes de reproyección en:", REPROJ_DIR)
print("Hecho.")
