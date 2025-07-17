#include <iostream> // Necesario para std::cout, std::cin, std::endl

int main() {
    std::cout << "--- Simulación de Control de Vuelo del Dron ---" << std::endl;

    double altitud_actual; // En metros
    int carga_bateria;    // En porcentaje (0-100)
    bool obstaculo_cercano; // true si hay obstáculo, false si no

    // 1. Entrada de datos por el usuario
    std::cout << "\nIntroduce la altitud actual del dron (m): ";
    std::cin >> altitud_actual;

    std::cout << "Introduce la carga de bateria (0-100%): ";
    std::cin >> carga_bateria;

    std::cout << "¿Hay un obstaculo cercano? (1 para Si / 0 para No): ";
    std::cin >> obstaculo_cercano;

    // --- Uso de 'if' simple ---
    // Si la condición es verdadera, se ejecuta el bloque.
    if (obstaculo_cercano) {
        std::cout << "\n[Alerta IF]: Obstaculo detectado. Reduciendo velocidad..." << std::endl;
    }

    // --- Uso de 'if-else' ---
    // Si la condición es verdadera se ejecuta un bloque, si es falsa se ejecuta el otro.
    if (carga_bateria < 20) {
        std::cout << "[IF-ELSE]: Bateria baja. Iniciando retorno a base." << std::endl;
    } else {
        std::cout << "[IF-ELSE]: Bateria suficiente. Continuando mision." << std::endl;
    }

    // --- Uso de 'if-else if-else' ---
    // Múltiples condiciones encadenadas. Solo se ejecuta el primer bloque cuya condición sea verdadera.
    if (altitud_actual > 100.0) {
        std::cout << "[IF-ELSE IF-ELSE]: Altitud elevada. Volando por encima de edificios." << std::endl;
    } else if (altitud_actual > 50.0) {
        std::cout << "[IF-ELSE IF-ELSE]: Altitud media. Volando sobre areas residenciales." << std::endl;
    } else if (altitud_actual > 10.0) {
        std::cout << "[IF-ELSE IF-ELSE]: Altitud baja. Realizando inspeccion detallada." << std::endl;
    } else { // Si ninguna de las condiciones anteriores es verdadera (altitud <= 10.0)
        std::cout << "[IF-ELSE IF-ELSE]: Altitud muy baja. Preparandose para aterrizar o despegue." << std::endl;
    }

    std::cout << "\n--- Fin de la simulacion de control de vuelo ---" << std::endl;

    return 0;
}