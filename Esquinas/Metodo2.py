import math

px, py = 232, 80
ancho_imagen, alto_imagen = 640, 480
fov_horizontal_grados = 90.0
fov_vertical_grados = 60.0
distancia_real = 1.5 #m

fov_horizontal_rad = math.radians(fov_horizontal_grados)
fov_vertical_rad = math.radians(fov_vertical_grados)
nx = (px - ancho_imagen / 2) / (ancho_imagen / 2)
ny = -(py - alto_imagen / 2) / (alto_imagen / 2)

angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
angulo_vertical_rad = ny * (fov_vertical_rad / 2)

print(f"{math.degrees(angulo_horizontal_rad):.2f}")
print(f"{math.degrees(angulo_vertical_rad):.2f}")


coordenada_Z = distancia_real * math.sin(angulo_vertical_rad)

coordenada_Y = distancia_real * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)

coordenada_X = distancia_real * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)


print(f"  X = {coordenada_X:.3f} m")
print(f"  Y = {coordenada_Y:.3f} m")
print(f"  Z = {coordenada_Z:.3f} m")

distancia_calculada = math.sqrt(coordenada_X**2 + coordenada_Y**2 + coordenada_Z**2)
print(f"Verificación: La distancia calculada es {distancia_calculada:.3f} m (debería ser {distancia_real} m)")