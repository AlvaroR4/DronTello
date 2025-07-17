#include <iostream>      // Para std::cout, std::endl
#include <string>        // Para std::string
#include <vector>        // Para std::vector (reutilizamos)
#include <map>           // Para std::map
#include <unordered_map> // Para std::unordered_map
#include <stdexcept>     // Para std::out_of_range si usas .at()

// Clase DronTello simplificada
class DronTello {
public:
    // Constructor que requiere un ID (este es el que el mapa intentaba llamar sin argumentos)
    DronTello(std::string id) : id_dron(id), altitud_actual(0.0) {
        std::cout << "Dron " << id_dron << " ha sido registrado." << std::endl;
    }

    // Un constructor de copia que el compilador genera por defecto si no hay otros,
    // o puedes definirlo explícitamente si necesitas lógica especial al copiar.
    // DronTello(const DronTello& otro) = default; // Esto es implicito si no defines el constructor de movimiento.

    // No hay constructor por defecto DronTello(), asi que las operaciones que lo necesiten fallaran.

    void set_altitud(double altitud) {
        altitud_actual = altitud;
    }

    std::string get_id() const {
        return id_dron;
    }

    double get_altitud() const {
        return altitud_actual;
    }

private:
    std::string id_dron;
    double altitud_actual;
};

int main() {
    std::cout << "--- Uso de std::string, std::map y std::unordered_map ---" << std::endl;

    // --- 1. Demostracion de std::string ---
    std::cout << "\n--- Ejemplos de std::string ---" << std::endl;
    std::string nombre_operador = "Alvaro Sanchez";
    std::string mensaje_inicial = "Sistema iniciado por ";

    // Concatenacion
    std::string mensaje_completo = mensaje_inicial + nombre_operador + ".";
    std::cout << mensaje_completo << std::endl;

    // Comparacion
    if (nombre_operador == "Alvaro Sanchez") {
        std::cout << "Bienvenido, " << nombre_operador << "!" << std::endl;
    }

    // Subcadenas y busqueda
    std::string datos_mision = "Mision_Exploracion_Area7_V2.0";
    if (datos_mision.find("Area7") != std::string::npos) { // find devuelve std::string::npos si no encuentra
        std::cout << "La mision es para el Area 7." << std::endl;
    }
    std::string version = datos_mision.substr(datos_mision.find("V") + 1);
    std::cout << "Version de la mision: " << version << std::endl;


    // --- 2. Demostracion de std::map (mapa ordenado) ---
    std::cout << "\n--- Ejemplos de std::map (Configuracion del Dron) ---" << std::endl;
    std::map<std::string, double> configuracion_dron; // Clave: nombre parametro (string), Valor: valor (double)

    // Asignar valores (inserta si la clave no existe, actualiza si ya existe)
    configuracion_dron["PID_Ganancia_P"] = 0.5;
    configuracion_dron["PID_Ganancia_I"] = 0.1;
    configuracion_dron["Umbral_Obstaculo"] = 1.5; // metros
    configuracion_dron["Velocidad_Crucero"] = 5.0; // m/s

    // Acceder a valores
    std::cout << "Ganancia P del PID: " << configuracion_dron["PID_Ganancia_P"] << std::endl;

    // Iterar sobre el mapa (claves en orden alfabetico)
    std::cout << "Todos los parametros de configuracion:" << std::endl;
    for (const auto& par : configuracion_dron) {
        std::cout << "  " << par.first << ": " << par.second << std::endl;
    }

    // Intentar acceder a una clave que no existe (¡cuidado, la inserta con valor por defecto si existe un constructor por defecto!)
    // double valor_no_existente = configuracion_dron["Parametro_Fantasma"]; // Esto crearia el elemento si DronTello() existiera
    // std::cout << "Valor de Parametro_Fantasma: " << valor_no_existente << std::endl;

    // Mejor usar .count() o .find() para verificar existencia
    if (configuracion_dron.count("Umbral_Obstaculo")) {
        std::cout << "Umbral de obstaculo existe: " << configuracion_dron["Umbral_Obstaculo"] << std::endl;
    }

    // --- 3. Demostracion de std::unordered_map (mapa no ordenado, mas rapido) ---
    std::cout << "\n--- Ejemplos de std::unordered_map (Estados de Vuelo) ---" << std::endl;
    std::unordered_map<int, std::string> codigos_estado_vuelo; // Clave: int, Valor: string

    codigos_estado_vuelo[100] = "En tierra, esperando despegue";
    codigos_estado_vuelo[200] = "Volando, mision activa";
    codigos_estado_vuelo[300] = "Aterrizando, retorno a base";
    codigos_estado_vuelo[400] = "Emergencia, fallo de sistema";
    codigos_estado_vuelo[201] = "Volando, modo de exploracion"; // El orden de insercion no afecta el orden interno

    std::cout << "Estado de vuelo 200: " << codigos_estado_vuelo[200] << std::endl;

    // Iterar sobre el unordered_map (el orden no es predecible)
    std::cout << "Todos los codigos de estado de vuelo:" << std::endl;
    for (const auto& par : codigos_estado_vuelo) {
        std::cout << "  Codigo: " << par.first << ", Descripcion: " << par.second << std::endl;
    }

    // --- Aplicacion combinada: Almacenar la flota de drones por ID ---
    // Usaremos un std::map para almacenar los objetos DronTello por su ID (string)
    std::cout << "\n--- Gestion de Flota de Drones con std::map ---" << std::endl;
    std::map<std::string, DronTello> flota_drones;

    // Insertar drones en el mapa. La clave es el ID del dron.
    // Usamos emplace, que construye el objeto DronTello directamente en el mapa
    flota_drones.emplace("DRONE_A_001", DronTello("DRONE_A_001"));
    flota_drones.emplace("DRONE_B_002", DronTello("DRONE_B_002"));
    flota_drones.emplace("DRONE_C_003", DronTello("DRONE_C_003"));

    // --- CORRECCIÓN DE LA LÍNEA QUE DABA ERROR ---
    // En lugar de flota_drones["DRONE_B_002"].set_altitud(50.5);
    // Usamos find() para localizar el dron y solo modificarlo si existe.
    std::string id_a_actualizar = "DRONE_B_002";
    auto it_dron_b = flota_drones.find(id_a_actualizar); // find devuelve un iterador

    if (it_dron_b != flota_drones.end()) { // Si find no devuelve .end(), significa que encontro la clave
        std::cout << "\nActualizando altitud de " << id_a_actualizar << "..." << std::endl;
        it_dron_b->second.set_altitud(50.5); // it->second es el objeto DronTello
        std::cout << "Altitud de " << id_a_actualizar << " actualizada a " << it_dron_b->second.get_altitud() << "m." << std::endl;
    } else {
        std::cout << "\nError: El dron " << id_a_actualizar << " no fue encontrado para actualizar su altitud." << std::endl;
    }
    // --- FIN DE LA CORRECCIÓN ---


    // Iterar sobre la flota y mostrar su estado
    std::cout << "\nEstado actual de la flota:" << std::endl;
    for (const auto& par : flota_drones) {
        // par.first es la clave (ID del dron)
        // par.second es el valor (el objeto DronTello)
        std::cout << "  Dron ID: " << par.first
                  << ", Altitud: " << par.second.get_altitud() << "m" << std::endl;
    }
    
    // Buscar un dron específico (este ya funcionaba)
    std::string id_a_buscar = "DRONE_A_001";
    auto it = flota_drones.find(id_a_buscar); // find devuelve un iterador

    if (it != flota_drones.end()) { // Si find no devuelve .end(), significa que encontro la clave
        std::cout << "\nEl dron " << id_a_buscar << " fue encontrado. Altitud: "
                  << it->second.get_altitud() << "m" << std::endl; // it->second es el objeto DronTello
    } else {
        std::cout << "\nEl dron " << id_a_buscar << " NO fue encontrado." << std::endl;
    }


    std::cout << "\n--- Fin de la demostracion ---" << std::endl;

    return 0;
}