#include <iostream> // Para std::cout, std::endl

// Definición de una estructura simple para representar la altitud y velocidad
struct EstadoSimpleDron {
    double altitud_m; // Altitud en metros
    double velocidad_vertical_mps; // Velocidad vertical en m/s
};

int main() {
    std::cout << "--- Demostracion de Punteros y Referencias en C++ ---" << std::endl;

    // --- Parte 1: Variables Simples (int) ---
    int vida_dron = 100; // Puntos de vida del dron
    std::cout << "\n--- Variables Simples ---" << std::endl;
    std::cout << "Vida Dron (original): " << vida_dron << std::endl;
    std::cout << "Direccion de 'vida_dron': " << &vida_dron << std::endl; // Usamos & para obtener la dirección

    // --- Uso de Puntero ---
    // Declaracion de un puntero a un entero
    int* puntero_a_vida;
    // Asignar al puntero la direccion de 'vida_dron'
    puntero_a_vida = &vida_dron;

    std::cout << "\n--- Uso de Puntero ('*') ---" << std::endl;
    std::cout << "Valor al que apunta 'puntero_a_vida' (*puntero_a_vida): " << *puntero_a_vida << std::endl;
    std::cout << "Direccion almacenada en 'puntero_a_vida': " << puntero_a_vida << std::endl; // Muestra la misma direccion que &vida_dron

    // Modificar el valor a traves del puntero
    *puntero_a_vida = 75; // Desreferenciamos para acceder y modificar el valor
    std::cout << "Vida Dron (despues de modificar con puntero): " << vida_dron << std::endl; // ¡vida_dron ha cambiado!

    // Los punteros pueden apuntar a 'nullptr' (similar a NULL en C)
    // int* otro_puntero = nullptr; // Un puntero que no apunta a nada
    // Si intentas desreferenciar un nullptr, tu programa crasheara (error de segmentacion)

    // --- Uso de Referencia ---
    // Declaracion de una referencia a un entero
    // ¡Debe inicializarse en el momento de la declaracion y no puede cambiar a que referencia!
    int& referencia_a_vida = vida_dron;

    std::cout << "\n--- Uso de Referencia ('&') ---" << std::endl;
    std::cout << "Valor a traves de 'referencia_a_vida': " << referencia_a_vida << std::endl; // Se usa directamente, sin '*'
    std::cout << "Direccion de 'referencia_a_vida': " << &referencia_a_vida << std::endl; // Misma direccion que vida_dron

    // Modificar el valor a traves de la referencia
    referencia_a_vida = 50; // Modificamos 'referencia_a_vida', lo que modifica 'vida_dron'
    std::cout << "Vida Dron (despues de modificar con referencia): " << vida_dron << std::endl; // ¡vida_dron ha cambiado de nuevo!

    // --- Parte 2: Uso con Estructuras y Funciones ---

    EstadoSimpleDron estado_actual = {10.5, 2.3}; // Altitud 10.5m, velocidad 2.3m/s
    std::cout << "\n--- Con Estructuras y Funciones ---" << std::endl;
    std::cout << "Estado Dron inicial: Altitud=" << estado_actual.altitud_m
              << "m, Velocidad=" << estado_actual.velocidad_vertical_mps << "m/s" << std::endl;

    // --- Funcion que recibe un puntero a una estructura ---
    // Se usa '->' (operador flecha) para acceder a miembros de una estructura a traves de un puntero.
    auto modificar_estado_con_puntero = [](EstadoSimpleDron* p_estado, double nueva_altitud, double nueva_velocidad) {
        if (p_estado != nullptr) { // Siempre verificar que el puntero no sea nulo antes de usarlo
            p_estado->altitud_m = nueva_altitud;
            p_estado->velocidad_vertical_mps = nueva_velocidad;
            std::cout << "  (Funcion con puntero) Estado modificado." << std::endl;
        }
    };

    // Llamada a la funcion con puntero
    modificar_estado_con_puntero(&estado_actual, 15.0, 1.0); // Le pasamos la direccion de 'estado_actual'
    std::cout << "Estado Dron (despues de puntero): Altitud=" << estado_actual.altitud_m
              << "m, Velocidad=" << estado_actual.velocidad_vertical_mps << "m/s" << std::endl;

    // --- Funcion que recibe una referencia a una estructura ---
    // Se usa '.' (operador punto) como si fuera la variable directamente.
    auto modificar_estado_con_referencia = [](EstadoSimpleDron& ref_estado, double nueva_altitud, double nueva_velocidad) {
        ref_estado.altitud_m = nueva_altitud;
        ref_estado.velocidad_vertical_mps = nueva_velocidad;
        std::cout << "  (Funcion con referencia) Estado modificado." << std::endl;
    };

    // Llamada a la funcion con referencia
    modificar_estado_con_referencia(estado_actual, 5.0, -0.5); // Le pasamos la variable directamente, no su direccion
    std::cout << "Estado Dron (despues de referencia): Altitud=" << estado_actual.altitud_m
              << "m, Velocidad=" << estado_actual.velocidad_vertical_mps << "m/s" << std::endl;

    std::cout << "\n--- Fin de la demostracion ---" << std::endl;

    return 0;
}