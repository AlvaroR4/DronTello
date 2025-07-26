#include <iostream> // Para std::cout, std::endl
#include <string>   // Para std::string
#include <vector>   // Para std::vector
#include <stdexcept> // Para excepciones estándar como std::runtime_error, std::out_of_range

// Estructura simple para representar una lectura de sensor
struct LecturaSensor {
    std::string nombre_sensor;
    double valor;
};

// Clase DronTello (simplificada)
class DronTello {
public:
    DronTello(std::string id, int bateria_inicial) : id_dron(id), carga_bateria_porcentaje(bateria_inicial) {
        std::cout << "Dron " << id_dron << " creado con " << carga_bateria_porcentaje << "% de bateria." << std::endl;
    }

    // Metodo que podria lanzar una excepcion de bateria baja
    void despegar() {
        if (carga_bateria_porcentaje < 20) {
            // Lanzar una excepcion si la bateria es muy baja
            throw std::runtime_error("Error de despegue: Bateria insuficiente (" +
                                     std::to_string(carga_bateria_porcentaje) + "%).");
        }
        std::cout << id_dron << ": Despegando con exito. Bateria: "
                  << carga_bateria_porcentaje << "%." << std::endl;
        carga_bateria_porcentaje -= 5; // Simular consumo
    }

    // Metodo que podria lanzar una excepcion de division por cero
    double calcular_potencia_requerida(double carga_util_kg, double eficiencia_motor) {
        if (eficiencia_motor == 0.0) {
            throw std::invalid_argument("Error de calculo: La eficiencia del motor no puede ser cero.");
        }
        return carga_util_kg / eficiencia_motor;
    }

    // Metodo para simular acceso a un sensor, que podria no existir
    LecturaSensor obtener_lectura_sensor(const std::string& nombre_sensor, const std::vector<LecturaSensor>& sensores_disponibles) {
        for (const auto& sensor : sensores_disponibles) {
            if (sensor.nombre_sensor == nombre_sensor) {
                std::cout << "Sensor '" << nombre_sensor << "' encontrado." << std::endl;
                return sensor;
            }
        }
        // Si no se encuentra el sensor, lanzar una excepcion
        throw std::out_of_range("Error de sensor: El sensor '" + nombre_sensor + "' no esta disponible.");
    }

private:
    std::string id_dron;
    int carga_bateria_porcentaje;
};

int main() {
    std::cout << "--- Demostracion de Manejo de Excepciones ---" << std::endl;

    // Crear un objeto DronTello
    DronTello mi_dron("Dron_Excepciones", 15); // Bateria baja intencionadamente

    // --- 1. Ejemplo de 'try-catch' con bateria baja ---
    std::cout << "\n--- Intento de Despegue ---" << std::endl;
    try {
        mi_dron.despegar(); // Esto lanzara una excepcion porque la bateria es < 20
        std::cout << "¡Despegue completado (esto no deberia imprimirse)!" << std::endl;
    } catch (const std::runtime_error& e) { // Captura una excepcion de tipo std::runtime_error
        std::cerr << "Excepcion capturada al despegar: " << e.what() << std::endl;
        std::cout << "Accion: Recargando bateria o intentando de nuevo mas tarde." << std::endl;
    }

    // --- 2. Ejemplo de 'try-catch' con division por cero ---
    std::cout << "\n--- Intento de Calcular Potencia Requerida ---" << std::endl;
    double carga_util_dron = 1.0; // kg
    double eficiencia = 0.0;     // Intencionadamente cero para causar error

    try {
        double potencia = mi_dron.calcular_potencia_requerida(carga_util_dron, eficiencia);
        std::cout << "Potencia requerida: " << potencia << " unidades." << std::endl;
    } catch (const std::invalid_argument& e) { // Captura una excepcion de tipo std::invalid_argument
        std::cerr << "Excepcion capturada al calcular potencia: " << e.what() << std::endl;
        std::cout << "Accion: Verifique los parametros de entrada, la eficiencia no puede ser cero." << std::endl;
    }

    // --- 3. Ejemplo con multiples 'catch' para diferentes tipos de excepciones ---
    std::cout << "\n--- Intento de Obtener Lectura de Sensor ---" << std::endl;
    std::vector<LecturaSensor> sensores_disponibles = {
        {"LIDAR_Frontal", 2.3},
        {"Camara_RGB", 0.0},
        {"IMU_Principal", 1.5}
    };

    std::string sensor_a_buscar1 = "LIDAR_Frontal";
    std::string sensor_a_buscar2 = "Sensor_Inexistente";
    std::string sensor_a_buscar3 = "Camara_RGB";

    // Intento 1: Sensor existente
    try {
        LecturaSensor lectura = mi_dron.obtener_lectura_sensor(sensor_a_buscar1, sensores_disponibles);
        std::cout << "Lectura de '" << lectura.nombre_sensor << "': " << lectura.valor << std::endl;
    } catch (const std::out_of_range& e) { // Captura especifica para out_of_range
        std::cerr << "Error especificado (out_of_range): " << e.what() << std::endl;
    } catch (const std::exception& e) { // Captura generica para cualquier otra excepcion estandar
        std::cerr << "Otro tipo de excepcion estandar: " << e.what() << std::endl;
    } catch (...) { // Captura cualquier tipo de excepcion (ellipsis)
        std::cerr << "Excepcion desconocida capturada." << std::endl;
    }


    // Intento 2: Sensor inexistente
    try {
        LecturaSensor lectura = mi_dron.obtener_lectura_sensor(sensor_a_buscar2, sensores_disponibles);
        std::cout << "Lectura de '" << lectura.nombre_sensor << "': " << lectura.valor << std::endl;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error especificado (out_of_range): " << e.what() << std::endl;
        std::cout << "Accion: Verificar configuracion de sensores o hardware." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Otro tipo de excepcion estandar: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Excepcion desconocida capturada." << std::endl;
    }

    // Intento 3: Otro sensor existente
    try {
        LecturaSensor lectura = mi_dron.obtener_lectura_sensor(sensor_a_buscar3, sensores_disponibles);
        std::cout << "Lectura de '" << lectura.nombre_sensor << "': " << lectura.valor << std::endl;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error especificado (out_of_range): " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Otro tipo de excepcion estandar: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Excepcion desconocida capturada." << std::endl;
    }

    std::cout << "\n--- Fin de la demostracion de manejo de excepciones ---" << std::endl;

    return 0;
}