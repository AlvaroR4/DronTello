#include <iostream> // Para std::cout, std::endl, std::cin
#include <vector>   // ¡Necesario para std::vector!
#include <string>   // Para usar std::string
#include <numeric>  // Para std::accumulate (suma elementos de un vector)

// Clase DronTello (simplificada para este ejemplo)
class DronTello {
public:
    DronTello(std::string id) : id_dron(id) {
        std::cout << "Dron " << id_dron << " listo." << std::endl;
    }

    void imprimir_id() const {
        std::cout << "ID del Dron: " << id_dron << std::endl;
    }

private:
    std::string id_dron;
};

// Estructura Punto3D (ya la conocemos)
struct Punto3D {
    double x;
    double y;
    double z;
};

int main() {
    std::cout << "--- Uso de std::vector en la Simulacion del Dron ---" << std::endl;

    // --- 1. Vector de números (lecturas de sensores) ---
    std::vector<double> lecturas_distancia; // Vector vacío inicialmente

    std::cout << "\n--- Vector de Lecturas de Distancia ---" << std::endl;
    std::cout << "Tamaño inicial del vector: " << lecturas_distancia.size() << std::endl;

    // Añadir lecturas dinámicamente
    lecturas_distancia.push_back(5.7);
    lecturas_distancia.push_back(4.2);
    lecturas_distancia.push_back(6.1);
    lecturas_distancia.push_back(3.9);

    std::cout << "Tamaño despues de añadir 4 lecturas: " << lecturas_distancia.size() << std::endl;

    // Acceder e imprimir elementos (bucle for tradicional)
    std::cout << "Lecturas registradas (usando corchetes []):" << std::endl;
    for (int i = 0; i < lecturas_distancia.size(); ++i) {
        std::cout << "  Lectura " << i << ": " << lecturas_distancia[i] << "m" << std::endl;
    }

    // Acceder de forma segura con .at()
    std::cout << "Lectura en posicion 1 (con .at()): " << lecturas_distancia.at(1) << "m" << std::endl;
    // lecturas_distancia.at(10); // Esto lanzaría una excepción si se descomenta y ejecuta


    // --- 2. Vector de estructuras (waypoints) ---
    std::vector<Punto3D> waypoints; // Vector para almacenar puntos 3D

    std::cout << "\n--- Vector de Waypoints (Puntos de Ruta) ---" << std::endl;
    // Añadir waypoints al vector
    waypoints.push_back({10.0, 5.0, 2.0}); // Inicialización de struct en el push_back
    waypoints.push_back({25.0, 15.0, 3.0});
    Punto3D p3 = {40.0, 30.0, 4.0};
    waypoints.push_back(p3);

    std::cout << "Numero de waypoints: " << waypoints.size() << std::endl;

    // Iterar e imprimir waypoints (bucle for basado en rango, ¡muy limpio!)
    std::cout << "Lista de Waypoints:" << std::endl;
    for (const Punto3D& wp : waypoints) { // Usar 'const &' para eficiencia y evitar copias
        std::cout << "  Waypoint: (" << wp.x << ", " << wp.y << ", " << wp.z << ")" << std::endl;
    }

    // --- 3. Vector de objetos de Clase (flota de drones) ---
    std::vector<DronTello> flota; // Vector para almacenar objetos DronTello

    std::cout << "\n--- Flota de Drones (Vector de Objetos) ---" << std::endl;
    flota.push_back(DronTello("Dron_Alpha")); // Se crea el objeto DronTello directamente en el vector
    flota.push_back(DronTello("Dron_Beta"));
    flota.push_back(DronTello("Dron_Gamma"));

    std::cout << "Total de drones en la flota: " << flota.size() << std::endl;

    // Iterar sobre la flota y llamar a un metodo de cada dron
    std::cout << "Drones en la flota:" << std::endl;
    for (DronTello& dron : flota) { // Usar '&' si quieres modificar los objetos del vector
        dron.imprimir_id();
    }

    // --- 4. Operaciones adicionales con vector ---
    std::cout << "\n--- Operaciones Adicionales ---" << std::endl;
    
    // Eliminar el ultimo elemento
    if (!lecturas_distancia.empty()) {
        std::cout << "Eliminando ultima lectura (" << lecturas_distancia.back() << "m)..." << std::endl;
        lecturas_distancia.pop_back();
        std::cout << "Nuevo tamaño de lecturas_distancia: " << lecturas_distancia.size() << std::endl;
    }

    // Sumar todos los elementos de un vector (ej. total de lecturas)
    // Necesita #include <numeric>
    double suma_distancias = 0.0;
    for (double d : lecturas_distancia) {
        suma_distancias += d;
    }
    std::cout << "Suma total de distancias restantes: " << suma_distancias << "m" << std::endl;

    // Limpiar el vector (vaciarlo completamente)
    flota.clear();
    std::cout << "La flota de drones ha sido limpiada. ¿Esta vacia? "
              << (flota.empty() ? "Si" : "No") << std::endl;

    std::cout << "\n--- Fin de la demostracion de std::vector ---" << std::endl;

    return 0;
}