#include <iostream> // Para std::cout, std::endl
#include <string>   // Para std::string

// --- 1. Definición de la Clase DronTello ---
// Una clase es una plantilla para crear objetos.
class DronTello {
public: // Miembros publicos: accesibles desde fuera de la clase
    // Constructor: Es un metodo especial que se llama automaticamente cuando se crea un objeto de la clase.
    // Se usa para inicializar los atributos del objeto.
    DronTello(std::string id, int bateria_inicial) {
        // Inicializamos los atributos al crear un nuevo dron
        id_dron = id;
        altitud_actual = 0.0; // Los drones Tello empiezan en el suelo
        carga_bateria_porcentaje = bateria_inicial;
        velocidad_actual_mps = 0.0;
        esta_volando = false;
        std::cout << "Dron " << id_dron << " creado con " << carga_bateria_porcentaje << "% de bateria." << std::endl;
    }

    // Metodos publicos (acciones que el dron puede realizar)
    void despegar() {
        if (!esta_volando && carga_bateria_porcentaje >= 20) {
            esta_volando = true;
            altitud_actual = 1.0; // Despega a 1 metro
            velocidad_actual_mps = 0.5;
            carga_bateria_porcentaje -= 5; // Gasta bateria al despegar
            std::cout << id_dron << ": Despegando. Altitud: " << altitud_actual << "m." << std::endl;
            mostrar_estado();
        } else if (esta_volando) {
            std::cout << id_dron << ": Ya esta en el aire." << std::endl;
        } else {
            std::cout << id_dron << ": Bateria demasiado baja para despegar (" << carga_bateria_porcentaje << "%)." << std::endl;
        }
    }

    void aterrizar() {
        if (esta_volando) {
            esta_volando = false;
            altitud_actual = 0.0;
            velocidad_actual_mps = 0.0;
            carga_bateria_porcentaje -= 3; // Gasta bateria al aterrizar
            std::cout << id_dron << ": Aterrizando. Posicion en el suelo." << std::endl;
            mostrar_estado();
        } else {
            std::cout << id_dron << ": Ya esta en el suelo." << std::endl;
        }
    }

    void mover(double distancia_metros, double velocidad_deseada) {
        if (esta_volando) {
            if (carga_bateria_porcentaje < 10) { // Umbral para no moverse
                std::cout << id_dron << ": Bateria critica. No puedo moverme." << std::endl;
                return; // Salir de la funcion
            }
            velocidad_actual_mps = velocidad_deseada;
            // Simular consumo de bateria por distancia (simple)
            int consumo_bateria = static_cast<int>(distancia_metros * 0.5); // 0.5% por metro
            carga_bateria_porcentaje -= consumo_bateria;
            if (carga_bateria_porcentaje < 0) carga_bateria_porcentaje = 0; // Evitar bateria negativa

            std::cout << id_dron << ": Moviendose " << distancia_metros << "m a "
                      << velocidad_deseada << "m/s." << std::endl;
            altitud_actual += 0.1; // Simular un pequeño cambio de altitud al moverse
            mostrar_estado();
        } else {
            std::cout << id_dron << ": No puedo moverme si no estoy volando." << std::endl;
        }
    }

    // Un "getter" para obtener el valor de un atributo privado
    int get_carga_bateria() const { // 'const' significa que este metodo no modifica el objeto
        return carga_bateria_porcentaje;
    }
    
    // Un "getter" para obtener el ID del dron
    std::string get_id() const {
        return id_dron;
    }

private: // Miembros privados: solo accesibles desde dentro de la propia clase
    std::string id_dron;
    double altitud_actual;
    int carga_bateria_porcentaje;
    double velocidad_actual_mps;
    bool esta_volando;

    // Metodo privado (ayuda interna para la clase, no accesible desde fuera)
    void mostrar_estado() {
        std::cout << "  [INFO] " << id_dron << " Estado: Altitud=" << altitud_actual << "m, Bateria="
                  << carga_bateria_porcentaje << "%, Velocidad=" << velocidad_actual_mps << "m/s, Volando="
                  << (esta_volando ? "Si" : "No") << std::endl;
    }
}; // ¡No olvides el punto y coma al final de la declaración de la clase!

// --- Función main: El punto de entrada del programa ---
int main() {
    std::cout << "--- Simulacion de Drones con Clases (POO) ---" << std::endl;

    // 2. Creacion de objetos (instancias) de la clase DronTello
    // Se llama al constructor que definimos
    DronTello mi_primer_dron("Tello_Explorador", 90);
    DronTello mi_segundo_dron("Tello_Cartografo", 75);

    std::cout << "\n--- Operaciones con mi_primer_dron (" << mi_primer_dron.get_id() << ") ---" << std::endl;
    mi_primer_dron.despegar();
    mi_primer_dron.mover(10.0, 2.0); // Mover 10 metros a 2 m/s
    mi_primer_dron.mover(5.0, 1.5);
    std::cout << "Bateria de " << mi_primer_dron.get_id() << " antes de aterrizar: "
              << mi_primer_dron.get_carga_bateria() << "%" << std::endl;
    mi_primer_dron.aterrizar();
    mi_primer_dron.mover(2.0, 1.0); // Intentar mover en el suelo

    std::cout << "\n--- Operaciones con mi_segundo_dron (" << mi_segundo_dron.get_id() << ") ---" << std::endl;
    mi_segundo_dron.despegar();
    mi_segundo_dron.mover(30.0, 3.0); // Mover 30 metros a 3 m/s (gastara mas bateria)
    mi_segundo_dron.aterrizar();
    mi_segundo_dron.despegar(); // Intenta despegar de nuevo
    mi_segundo_dron.mover(5.0, 1.0); // Mueve un poco mas
    mi_segundo_dron.aterrizar();

    // Podemos interactuar con los objetos
    int bateria_dron1 = mi_primer_dron.get_carga_bateria();
    int bateria_dron2 = mi_segundo_dron.get_carga_bateria();

    if (bateria_dron1 < bateria_dron2) {
        std::cout << "\n" << mi_primer_dron.get_id() << " tiene menos bateria que " << mi_segundo_dron.get_id() << "." << std::endl;
    } else {
        std::cout << "\n" << mi_segundo_dron.get_id() << " tiene menos bateria que " << mi_primer_dron.get_id() << "." << std::endl;
    }


    std::cout << "\n--- Fin de la simulacion de clases ---" << std::endl;

    return 0;
}