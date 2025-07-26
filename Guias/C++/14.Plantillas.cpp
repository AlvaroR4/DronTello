#include <iostream> // Para std::cout, std::endl
#include <string>   // Para std::string
#include <vector>   // Para std::vector (un ejemplo de plantilla de clase)

// --- CLASE DronTello (MOVIDA AQUI, ANTES DE MAIN) ---
class DronTello {
public:
    DronTello(std::string id) : id_dron(id), altitud_actual(0.0) {
        std::cout << "Dron " << id_dron << " ha sido registrado." << std::endl;
    }

    void set_altitud(double altitud) {
        altitud_actual = altitud;
    }

    std::string get_id() const { // Método para obtener el ID
        return id_dron;
    }

    double get_altitud() const {
        return altitud_actual;
    }

private:
    std::string id_dron;
    double altitud_actual;
};
// --- FIN DE CLASE DronTello ---


// --- Estructura Punto3D (reutilizada, tambien antes de main) ---
struct Punto3D {
    double x, y, z;

    // Sobrecarga del operador * para que funcione con escalar_medida para Punto3D
    Punto3D operator*(double factor) const {
        return {x * factor, y * factor, z * factor};
    }

    // Constructor por defecto, necesario si creas un vector de Punto3D sin inicializar.
    Punto3D() : x(0.0), y(0.0), z(0.0) {}
    Punto3D(double val_x, double val_y, double val_z) : x(val_x), y(val_y), z(val_z) {}

};

// Para imprimir un Punto3D directamente con std::cout <<
std::ostream& operator<<(std::ostream& os, const Punto3D& p) {
    os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return os;
}


// --- 1. Plantilla de Función: 'imprimir_info' ---
// Una funcion generica que puede imprimir informacion de cualquier tipo.
template <typename T>
void imprimir_info(const std::string& prefijo, T valor) {
    std::cout << prefijo << ": " << valor << std::endl;
}

// Plantilla de funcion para simular una operacion que podria ocurrir en vision por computadora.
template <typename T>
T escalar_medida(T medida, double factor_escala) {
    return medida * factor_escala;
}


// --- 2. Plantilla de Clase: 'SensorGenerico' ---
template <typename TipoLectura> // 'TipoLectura' es el parametro de tipo
class SensorGenerico {
public:
    SensorGenerico(std::string nombre, TipoLectura valor_inicial)
        : nombre_sensor(nombre), ultima_lectura(valor_inicial) {
        std::cout << "Sensor " << nombre_sensor << " inicializado con valor: " << ultima_lectura << std::endl;
    }

    void tomar_lectura(TipoLectura nueva_lectura) {
        ultima_lectura = nueva_lectura;
        std::cout << "Sensor " << nombre_sensor << " nueva lectura: " << ultima_lectura << std::endl;
    }

    TipoLectura get_ultima_lectura() const {
        return ultima_lectura;
    }

private:
    std::string nombre_sensor;
    TipoLectura ultima_lectura; // El tipo de esta variable es el tipo de la plantilla
};


// --- Función main: El punto de entrada del programa ---
int main() {
    std::cout << "--- Demostracion de Plantillas (Templates) en C++ ---" << std::endl;

    // --- 1. Uso de Plantillas de Funcion ---
    std::cout << "\n--- Plantillas de Funcion ---" << std::endl;

    // El compilador deduce el tipo 'T' como int
    imprimir_info("Nivel de Bateria", 85);
    // El compilador deduce el tipo 'T' como double
    imprimir_info("Altitud Actual", 15.75);
    // El compilador deduce el tipo 'T' como std::string
    imprimir_info("Estado del Dron", std::string("En Vuelo"));

    // Usando la plantilla de funcion 'escalar_medida'
    int velocidad_int = 10;
    int velocidad_escalada_int = escalar_medida(velocidad_int, 1.5); // T es int
    std::cout << "Velocidad int escalada: " << velocidad_escalada_int << std::endl;

    double distancia_double = 5.25;
    double distancia_escalada_double = escalar_medida(distancia_double, 2.0); // T es double
    std::cout << "Distancia double escalada: " << distancia_escalada_double << std::endl;

    Punto3D punto_original = {1.0, 2.0, 3.0};
    // Ahora podemos escalar un Punto3D gracias a la sobrecarga del operador *
    Punto3D punto_escalado = escalar_medida(punto_original, 3.0); // T es Punto3D
    std::cout << "Punto original: " << punto_original << ", Punto escalado: " << punto_escalado << std::endl;


    // --- 2. Uso de Plantillas de Clase ---
    std::cout << "\n--- Plantillas de Clase ---" << std::endl;

    // Crear un sensor que lee enteros (ej. numero de puntos clave detectados)
    SensorGenerico<int> sensor_puntos_clave("ORB_Features", 500);
    sensor_puntos_clave.tomar_lectura(650);
    std::cout << "Ultima lectura de " << sensor_puntos_clave.get_ultima_lectura() << " puntos clave." << std::endl;

    // Crear un sensor que lee doubles (ej. lectura de IMU)
    SensorGenerico<double> sensor_imu("IMU_Roll", 0.05);
    sensor_imu.tomar_lectura(-0.12);
    std::cout << "Ultima lectura de " << sensor_imu.get_ultima_lectura() << " radianes de Roll." << std::endl;

    // Crear un sensor que lee strings (ej. estado de conectividad)
    SensorGenerico<std::string> sensor_conectividad("WiFi_Status", "Conectado");
    sensor_conectividad.tomar_lectura("Signal_Loss");
    std::cout << "Ultima lectura de " << sensor_conectividad.get_ultima_lectura() << " en conectividad." << std::endl;

    // Crear un sensor que lee Puntos3D (ej. posicion GPS)
    SensorGenerico<Punto3D> sensor_gps("GPS_Posicion", {10.5, 20.3, 5.1});
    sensor_gps.tomar_lectura({10.6, 20.4, 5.2});
    std::cout << "Ultima lectura de " << sensor_gps.get_ultima_lectura() << " del GPS." << std::endl;


    // --- 3. Plantillas en STL: std::vector (repaso y confirmacion) ---
    std::cout << "\n--- Plantillas en STL: std::vector ---" << std::endl;

    // Un vector de enteros
    std::vector<int> datos_enteros = {10, 20, 30};
    imprimir_info("Primer elemento de int vector", datos_enteros[0]);

    // Un vector de objetos DronTello
    std::vector<DronTello> flota_autonoma;
    flota_autonoma.push_back(DronTello("Dron_Autonomo_A"));
    flota_autonoma.push_back(DronTello("Dron_Autonomo_B"));
    
    std::cout << "Numero de drones en la flota autonoma: " << flota_autonoma.size() << std::endl;
    // CORRECCION: Usar get_id() en lugar de imprimir_id()
    std::cout << "ID del primer dron: " << flota_autonoma[0].get_id() << std::endl; 

    std::cout << "\n--- Fin de la demostracion de plantillas ---" << std::endl;

    return 0;
}