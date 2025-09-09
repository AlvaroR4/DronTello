#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os

CARPETA_IMAGENES = "tello_calib_images"   
CARPETA_SALIDA   = "calib_resultados"     
PATRON           = (6, 4)                 # esquinas internas (ancho x alto)
TAM_CUADRADO     = 3.4                   

def calibrar_camara(folder, output_dir, pattern_size=(7,6), square_size=1.0):
    """Calibra la cámara usando imágenes con un tablero de ajedrez."""
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # generar puntos 3D del tablero
    objp = np.zeros((pattern_size[1]*pattern_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []  # puntos 3D en mundo real
    imgpoints = []  # puntos 2D en imagen
    
    os.makedirs(output_dir, exist_ok=True)
    images = glob.glob(os.path.join(folder, '*.png')) + glob.glob(os.path.join(folder, '*.jpg'))
    if not images:
        raise RuntimeError(f"No se encontraron imágenes en {folder}")
    
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Dibujar y guardar la imagen con esquinas detectadas
            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            out_path = os.path.join(output_dir, f"detectado_{idx:03d}.png")
            cv2.imwrite(out_path, img)
            print(f"[OK] Esquinas detectadas y guardadas en {out_path}")
        else:
            print(f"[FAIL] No se detectaron esquinas en {fname}")
    
    if not objpoints:
        raise RuntimeError("No se detectaron esquinas en ninguna imagen.")

    ret, cam_mtx, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    return ret, cam_mtx, dist_coefs, rvecs, tvecs

def mostrar_parametros(cam_mtx, dist, ret=None):
    fx = cam_mtx[0,0]
    fy = cam_mtx[1,1]
    cx = cam_mtx[0,2]
    cy = cam_mtx[1,2]
    k1, k2, p1, p2 = dist.ravel()[:4]
    k3 = dist.ravel()[4] if dist.size >= 5 else None

    print("\n=== Parámetros de calibración ===")
    if ret is not None:
        print(f" Error de reproyección: {ret:.4f}")
    print(f"Camera.fx: {fx:.4f}")
    print(f"Camera.fy: {fy:.4f}")
    print(f"Camera.cx: {cx:.4f}")
    print(f"Camera.cy: {cy:.4f}")
    print("\nCoeficientes de distorsión:")
    print(f" Camera.k1: {k1:.6f}")
    print(f" Camera.k2: {k2:.6f}")
    print(f" Camera.p1: {p1:.6f}")
    print(f" Camera.p2: {p2:.6f}")
    if k3 is not None:
        print(f" Camera.k3: {k3:.6f}")

if __name__ == "__main__":
    ret, cam_mtx, dist, rvecs, tvecs = calibrar_camara(
        CARPETA_IMAGENES, CARPETA_SALIDA, PATRON, TAM_CUADRADO
    )
    mostrar_parametros(cam_mtx, dist, ret)
