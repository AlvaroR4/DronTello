import cv2

img = cv2.imread("aruco2.png", cv2.IMREAD_GRAYSCALE)

candidates = [
    cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_4X4_100, cv2.aruco.DICT_4X4_250,
    cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_5X5_100, cv2.aruco.DICT_5X5_250,
    cv2.aruco.DICT_6X6_50, cv2.aruco.DICT_6X6_100, cv2.aruco.DICT_6X6_250,
    cv2.aruco.DICT_7X7_50, cv2.aruco.DICT_7X7_100, cv2.aruco.DICT_7X7_250
]

# compatibilidad parámetros
try:
    default_params = cv2.aruco.DetectorParameters_create()
except AttributeError:
    default_params = cv2.aruco.DetectorParameters()

for d in candidates:
    ar = cv2.aruco.getPredefinedDictionary(d)
    corners, ids, _ = cv2.aruco.detectMarkers(img, ar, parameters=default_params)
    if ids is not None and len(ids) > 0:
        print("Detectado con:", d, " IDs:", ids.flatten())
    else:
        print("No detectado con:", d)
