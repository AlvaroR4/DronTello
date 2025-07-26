#include <iostream> // Para std::cout, std::cin, std::endl
#include <string>   // Para usar std::string (cadenas de texto) en las estructuras

// --- Definición de Estructuras (fuera de main, suelen ir al principio del archivo o en un .h) ---
// Estructura para representar un punto en 3D
struct Punto3D {
    double x; // Coordenada X
    double y; // Coordenada Y
    double z; // Coordenada Z
};

// Estructura para representar los datos de un sensor LIDAR
struct DatosLidar {
    float distancia_frontal;
    float distancia_trasera;
    float distancia_izquierda;
    float distancia_derecha;
    int puntos_detectados;
};

// Estructura para representar el estado completo del dron
struct EstadoDron {
    std::string id_dron; // Identificador del dron
    Punto3D posicion_global; // Posición global usando la struct Punto3D
    float velocidad_mps;     // Velocidad en metros por segundo
    int carga_bateria_porcentaje; // Batería restante
    bool modo_autonomo_activo; // true si está en modo autónomo
};

int main() {
    std::cout << "--- Arrays y Estructuras en la Simulacion del Dron ---" << std::endl;

    // --- Ejemplo de Array ---
    // Un array para almacenar las lecturas de temperatura de 5 zonas del dron
    // Los arrays se declaran con un tamaño fijo.
    const int NUM_SENSORES_TEMP = 5; // Mejor usar una constante para el tamaño
    double temperaturas_dron[NUM_SENSORES_TEMP];

    // Asignar valores a las temperaturas
    temperaturas_dron[0] = 75.2; // Motor 1
    temperaturas_dron[1] = 68.9; // Motor 2
    temperaturas_dron[2] = 55.1; // Bateria
    temperaturas_dron[3] = 48.7; // Placa controladora
    temperaturas_dron[4] = 30.0; // Ambiente (simulacion)

    std::cout << "\n--- Lecturas de Temperatura (Array) ---" << std::endl;
    for (int i = 0; i < NUM_SENSORES_TEMP; ++i) {
        std::cout << "Zona " << i + 1 << " temperatura: " << temperaturas_dron[i] << " °C" << std::endl;
        if (temperaturas_dron[i] > 70.0) {
            std::cout << "   ¡Advertencia! Temperatura alta en esta zona." << std::endl;
        }
    }

    // --- Ejemplo de Estructura: Punto3D ---
    Punto3D objetivo_mision;
    objetivo_mision.x = 100.5;
    objetivo_mision.y = 250.2;
    objetivo_mision.z = 50.0;

    std::cout << "\n--- Punto Objetivo de la Mision (Struct Punto3D) ---" << std::endl;
    std::cout << "Objetivo X: " << objetivo_mision.x << "m" << std::endl;
    std::cout << "Objetivo Y: " << objetivo_mision.y << "m" << std::endl;
    std::cout << "Objetivo Z: " << objetivo_mision.z << "m" << std::endl;

    // --- Ejemplo de Estructura: DatosLidar ---
    DatosLidar lecturas_lidar_actuales;
    lecturas_lidar_actuales.distancia_frontal = 2.5f;
    lecturas_lidar_actuales.distancia_trasera = 3.1f;
    lecturas_lidar_actuales.distancia_izquierda = 1.8f;
    lecturas_lidar_actuales.distancia_derecha = 4.0f;
    lecturas_lidar_actuales.puntos_detectados = 1500;

    std::cout << "\n--- Datos del Sensor LIDAR (Struct DatosLidar) ---" << std::endl;
    std::cout << "Distancia frontal: " << lecturas_lidar_actuales.distancia_frontal << "m" << std::endl;
    std::cout << "Puntos LIDAR detectados: " << lecturas_lidar_actuales.puntos_detectados << std::endl;
    if (lecturas_lidar_actuales.puntos_detectados < 500) {
        std::cout << "   Advertencia: Pocos puntos detectados, posible mala lectura." << std::endl;
    }

    // --- Ejemplo de Estructura Anidada: EstadoDron ---
    EstadoDron mi_tello;
    mi_tello.id_dron = "Tello_RTK_001"; // Necesitamos #include <string> para esto
    mi_tello.posicion_global.x = 10.0; // Accediendo a un miembro de la struct anidada
    mi_tello.posicion_global.y = 5.0;
    mi_tello.posicion_global.z = 2.0;
    mi_tello.velocidad_mps = 3.5f;
    mi_tello.carga_bateria_porcentaje = 85;
    mi_tello.modo_autonomo_activo = true;

    std::cout << "\n--- Estado Actual del Dron (Struct EstadoDron) ---" << std::endl;
    std::cout << "ID del dron: " << mi_tello.id_dron << std::endl;
    std::cout << "Posicion (X, Y, Z): (" << mi_tello.posicion_global.x << ", "
              << mi_tello.posicion_global.y << ", " << mi_tello.posicion_global.z << ")m" << std::endl;
    std::cout << "Velocidad: " << mi_tello.velocidad_mps << " m/s" << std::endl;
    std::cout << "Bateria: " << mi_tello.carga_bateria_porcentaje << "%" << std::endl;
    std::cout << "¿Modo autonomo activo? " << (mi_tello.modo_autonomo_activo ? "Si" : "No") << std::endl;

    if (mi_tello.carga_bateria_porcentaje < 30 && mi_tello.modo_autonomo_activo) {
        std::cout << "   Accion: Bateria baja en modo autonomo, el dron intentara regresar a base." << std::endl;
    } else if (mi_tello.posicion_global.z < 1.0) {
        std::cout << "   Accion: Altitud muy baja, el dron intentara ganar altura." << std::endl;
    }

    std::cout << "\n--- Fin de la simulacion ---" << std::endl;

    return 0;
}