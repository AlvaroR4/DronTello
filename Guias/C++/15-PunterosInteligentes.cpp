#include <iostream> // Para std::cout, std::endl
#include <string>   // Para std::string
#include <memory>   // ¡Para std::unique_ptr y std::shared_ptr!
#include <vector>   // Para std::vector

// Una clase simple para simular un recurso que necesita ser gestionado
class RegistroVuelo {
public:
    std::string id_registro;
    std::string datos_vuelo;

    // Constructor
    RegistroVuelo(std::string id, std::string datos) : id_registro(id), datos_vuelo(datos) {
        std::cout << "  [RegistroVuelo] Constructor: Registro '" << id_registro << "' creado." << std::endl;
    }

    // Destructor: Se llama automaticamente cuando el objeto se va a destruir
    ~RegistroVuelo() {
        std::cout << "  [RegistroVuelo] Destructor: Registro '" << id_registro << "' destruido." << std::endl;
    }

    void imprimir_datos() const {
        std::cout << "  Registro ID: " << id_registro << ", Datos: " << datos_vuelo << std::endl;
    }
};

// Una clase Dron que usa punteros inteligentes para gestionar un RegistroVuelo
class Dron {
public:
    std::string nombre_dron;
    
    // El dron tiene una propiedad unica de su registro de vuelo principal
    std::unique_ptr<RegistroVuelo> registro_principal; 

    // El dron podria compartir acceso a un registro de sistema global
    std::shared_ptr<RegistroVuelo> registro_sistema_compartido;

    Dron(std::string nombre) : nombre_dron(nombre) {
        std::cout << "Dron " << nombre_dron << " creado." << std::endl;
    }

    // El destructor del dron liberara automaticamente los unique_ptr y decrementara los shared_ptr
    ~Dron() {
        std::cout << "Dron " << nombre_dron << " destruido." << std::endl;
    }
    
    void asignar_registro_principal(std::string id, std::string datos) {
        // Creamos un unique_ptr. make_unique es la forma preferida (C++14 en adelante)
        registro_principal = std::make_unique<RegistroVuelo>(id, datos);
        std::cout << nombre_dron << ": Registro principal asignado." << std::endl;
    }

    void asignar_registro_sistema(std::shared_ptr<RegistroVuelo> shared_reg) {
        registro_sistema_compartido = shared_reg; // Copia el shared_ptr, incrementa el contador
        std::cout << nombre_dron << ": Registro de sistema compartido asignado." << std::endl;
    }

    void mostrar_registros() const {
        std::cout << nombre_dron << " - Registros:" << std::endl;
        if (registro_principal) { // Verifica si el unique_ptr no es nulo
            registro_principal->imprimir_datos(); // Acceso con -> como un puntero normal
        } else {
            std::cout << "  No hay registro principal asignado." << std::endl;
        }

        if (registro_sistema_compartido) { // Verifica si el shared_ptr no es nulo
            registro_sistema_compartido->imprimir_datos();
            std::cout << "  Conteo de referencias del registro compartido: "
                      << registro_sistema_compartido.use_count() << std::endl;
        } else {
            std::cout << "  No hay registro de sistema compartido asignado." << std::endl;
        }
    }
};

// Funcion que toma un unique_ptr por valor (lo "mueve" de propiedad)
void procesar_registro_unico(std::unique_ptr<RegistroVuelo> reg) {
    if (reg) { // Asegurarse de que no sea nulo despues del movimiento
        std::cout << "\n[Funcion] Procesando registro unico: ";
        reg->imprimir_datos();
    }
    // Cuando 'reg' sale de ambito aqui, el RegistroVuelo sera destruido automaticamente
    std::cout << "[Funcion] 'reg' saliendo de ambito, liberando memoria (si es el ultimo propietario)." << std::endl;
}

// Funcion que toma un shared_ptr por copia (incrementa el conteo)
void procesar_registro_compartido(std::shared_ptr<RegistroVuelo> reg) {
    std::cout << "\n[Funcion] Procesando registro compartido: ";
    reg->imprimir_datos();
    std::cout << "[Funcion] Conteo de referencias dentro de la funcion: " << reg.use_count() << std::endl;
    // Cuando 'reg' sale de ambito aqui, el conteo de referencias se decrementa
    std::cout << "[Funcion] 'reg' saliendo de ambito, decrementando conteo." << std::endl;
}


int main() {
    std::cout << "--- Demostracion de Punteros Inteligentes ---" << std::endl;

    // --- 1. std::unique_ptr ---
    std::cout << "\n--- Uso de std::unique_ptr (propiedad unica) ---" << std::endl;
    { // Este bloque crea un nuevo scope para ver la destruccion automatica
        std::unique_ptr<RegistroVuelo> mi_registro_temporal = std::make_unique<RegistroVuelo>("Temp_Log_01", "Datos de prueba temporales.");
        mi_registro_temporal->imprimir_datos();
        std::cout << "mi_registro_temporal en ambito." << std::endl;
        // Cuando salimos de este bloque, mi_registro_temporal se destruye y libera la memoria.
    } // Aqui se llama al destructor de RegistroVuelo "Temp_Log_01"

    std::cout << "\n--- Movimiento de std::unique_ptr a una funcion ---" << std::endl;
    std::unique_ptr<RegistroVuelo> registro_transferible = std::make_unique<RegistroVuelo>("Transfer_Log", "Datos para transferir.");
    std::cout << "Registro transferible antes del movimiento: ";
    registro_transferible->imprimir_datos();
    
    procesar_registro_unico(std::move(registro_transferible)); // Transferimos la propiedad
    
    // Despues del movimiento, registro_transferible es nulo
    if (registro_transferible == nullptr) {
        std::cout << "Registro transferible es ahora nulo despues del movimiento." << std::endl;
    }


    // --- 2. std::shared_ptr ---
    std::cout << "\n--- Uso de std::shared_ptr (propiedad compartida) ---" << std::endl;
    std::shared_ptr<RegistroVuelo> registro_global_sistema = std::make_shared<RegistroVuelo>("Global_System_Log", "Registro vital del sistema.");
    std::cout << "Conteo inicial de registro_global_sistema: " << registro_global_sistema.use_count() << std::endl;

    // Un Dron lo usa (crea una copia de shared_ptr, incrementa el conteo)
    Dron dron_alpha("Alpha");
    dron_alpha.asignar_registro_sistema(registro_global_sistema);
    std::cout << "Conteo despues de que Dron Alpha lo asigna: " << registro_global_sistema.use_count() << std::endl;
    
    // Otro Dron lo usa (otra copia, otro incremento)
    Dron dron_beta("Beta");
    dron_beta.asignar_registro_sistema(registro_global_sistema);
    std::cout << "Conteo despues de que Dron Beta lo asigna: " << registro_global_sistema.use_count() << std::endl;

    // Pasarlo a una funcion por copia (otro incremento, luego decremento al salir de la funcion)
    procesar_registro_compartido(registro_global_sistema);
    std::cout << "Conteo despues de la funcion: " << registro_global_sistema.use_count() << std::endl;
    
    dron_alpha.mostrar_registros();
    dron_beta.mostrar_registros();

    std::cout << "\n--- Registros de drones unicos ---" << std::endl;
    dron_alpha.asignar_registro_principal("Alpha_Vuelo_1", "Datos de la primera mision de Alpha.");
    dron_beta.asignar_registro_principal("Beta_Vuelo_2", "Datos de la segunda mision de Beta.");
    dron_alpha.mostrar_registros();
    dron_beta.mostrar_registros();

    std::cout << "\n--- Salida de ambito de Drones y registro_global_sistema ---" << std::endl;
    // dron_alpha y dron_beta se destruyen aqui, decrementando el conteo de registro_global_sistema
    // Finalmente, cuando registro_global_sistema sale de ambito, su conteo llega a cero y la memoria se libera.

    return 0;
} // Aqui se destruyen dron_alpha, dron_beta, y finalmente registro_global_sistema.