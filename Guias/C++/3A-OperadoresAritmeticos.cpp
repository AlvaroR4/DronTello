#include <iostream>

int main() {
    double distancia_recorrida = 15.5; // Metros
    double tiempo_transcurrido = 3.0;  // Segundos

    // Calcular velocidad
    double velocidad = distancia_recorrida / tiempo_transcurrido; // 15.5 / 3.0 = 5.166...
    std::cout << "Velocidad del dron: " << velocidad << " m/s" << std::endl;

    int puntos_mapa = 100;
    int puntos_nuevos = 25;
    int total_puntos = puntos_mapa + puntos_nuevos;
    std::cout << "Total de puntos en el mapa: " << total_puntos << std::endl;

    int frames_totales = 60;
    int frames_procesados = 17;
    int frames_restantes = frames_totales % frames_procesados; // 60 % 17 = 9 (17*3 = 51, 60-51 = 9)
    std::cout << "Frames restantes en el lote (usando modulo): " << frames_restantes << std::endl;

    return 0;
}