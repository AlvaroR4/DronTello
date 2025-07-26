#include <iostream> // Para std::cout, std::cin, std::endl
#include <chrono>   // Para std::chrono::seconds (simular tiempo)
#include <thread>   // Para std::this_thread::sleep_for (simular espera)

int main() {
    std::cout << "--- Simulacion de Operaciones Repetitivas del Dron ---" << std::endl;

    // --- Ejemplo de Bucle 'while' ---
    // Simular la espera de que la batería alcance un nivel mínimo para el despegue.

    double carga_bateria_actual = 0.0;
    const double carga_minima_despegue = 20.0; // Porcentaje

    std::cout << "\n[Bucle WHILE]: Esperando bateria para despegar (minimo 20%)..." << std::endl;

    while (carga_bateria_actual < carga_minima_despegue) {
        std::cout << "Bateria actual: " << carga_bateria_actual << "%. Recargando..." << std::endl;
        carga_bateria_actual += 5.0; // Simula que la batería sube un 5% cada vez
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Espera 1 segundo para simular la carga
    }
    std::cout << "Bateria cargada al " << carga_bateria_actual << "%. ¡Despegue autorizado!" << std::endl;

    // --- Ejemplo de Bucle 'for' ---
    // Simular el procesamiento de un número fijo de frames de video.

    int numero_frames_a_procesar;
    std::cout << "\n[Bucle FOR]: Introduce el numero de frames a procesar: ";
    std::cin >> numero_frames_a_procesar;

    std::cout << "Iniciando procesamiento de " << numero_frames_a_procesar << " frames..." << std::endl;
    for (int i = 0; i < numero_frames_a_procesar; ++i) { // i se inicializa en 0, se ejecuta mientras i sea menor que el numero total, i se incrementa en 1 despues de cada iteracion
        std::cout << "Procesando frame " << (i + 1) << " de " << numero_frames_a_procesar << std::endl;
        // Aquí iría el código real para procesar un frame, por ejemplo, aplicar un filtro de OpenCV
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Pequeña espera para simular trabajo
    }
    std::cout << "Procesamiento de frames completado." << std::endl;


    // --- Ejemplo Adicional: Bucle 'while' con entrada de usuario para controlar la repetición ---
    char comando_dron;
    std::cout << "\n[Bucle WHILE interactivo]: Introduce un comando ('v' para volar, 'a' para aterrizar, 'q' para salir): ";
    std::cin >> comando_dron; // Lee el primer comando

    while (comando_dron != 'q' && comando_dron != 'Q') { // El bucle continúa mientras el comando no sea 'q' o 'Q'
        if (comando_dron == 'v' || comando_dron == 'V') {
            std::cout << "Dron volando..." << std::endl;
        } else if (comando_dron == 'a' || comando_dron == 'A') {
            std::cout << "Dron aterrizando..." << std::endl;
        } else {
            std::cout << "Comando desconocido. Intenta de nuevo." << std::endl;
        }

        std::cout << "Introduce el siguiente comando ('v', 'a', 'q'): ";
        std::cin >> comando_dron; // Lee el siguiente comando para la próxima iteración
    }
    std::cout << "Saliendo del control de dron. ¡Hasta la proxima!" << std::endl;

    std::cout << "\n--- Fin de la simulacion ---" << std::endl;

    return 0;
}