#include <iostream>

int main() {
    double nivel_bateria = 25.5; // Porcentaje
    double umbral_bateria_baja = 20.0; // Umbral crítico

    bool bateria_baja = (nivel_bateria < umbral_bateria_baja);
    std::cout << "¿Batería baja? " << bateria_baja << std::endl; // Salida: 0 (false)

    int puntos_actuales = 150;
    int puntos_minimos_requeridos = 100;

    bool suficientes_puntos = (puntos_actuales >= puntos_minimos_requeridos);
    std::cout << "¿Suficientes puntos detectados? " << suficientes_puntos << std::endl; // Salida: 1 (true)

    // Comparando con valores literales
    bool es_igual_a_cero = (0 == 0);
    std::cout << "¿Cero es igual a cero? " << es_igual_a_cero << std::endl; // Salida: 1 (true)

    return 0;
}