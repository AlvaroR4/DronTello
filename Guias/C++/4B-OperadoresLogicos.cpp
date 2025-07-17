#include <iostream>

int main() {
    bool obstaculo_detectado = true;
    bool ruta_alternativa_disponible = false;
    double velocidad_dron = 0.5; // m/s
    double velocidad_segura_obstaculo = 1.0; // m/s

    // Condición para detener el dron: hay obstáculo Y NO hay ruta alternativa
    bool detener_dron = obstaculo_detectado && !ruta_alternativa_disponible;
    std::cout << "¿Detener dron? " << detener_dron << std::endl; // Salida: 1 (true)

    // Condición para alerta: hay obstáculo O la velocidad es muy alta
    bool alerta_critica = obstaculo_detectado || (velocidad_dron > velocidad_segura_obstaculo);
    std::cout << "¿Alerta crítica? " << alerta_critica << std::endl; // Salida: 1 (true, porque obstaculo_detectado es true)

    // Ejemplo de negación
    bool es_dia = false;
    bool es_noche = !es_dia;
    std::cout << "¿Es de noche? " << es_noche << std::endl; // Salida: 1 (true)

    return 0;
}