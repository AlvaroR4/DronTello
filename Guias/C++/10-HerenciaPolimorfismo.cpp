#include <iostream> // Para std::cout, std::endl
#include <string>   // Para std::string
#include <vector>   // Para std::vector (lo veremos mas a fondo en el siguiente concepto)

// --- 1. Clase Base: Dron ---
// Representa un dron generico con funcionalidades basicas.
class Dron {
public:
    // Constructor de la clase base
    Dron(std::string id, int bateria_inicial) : id_dron(id), carga_bateria_porcentaje(bateria_inicial) {
        std::cout << "Dron base " << id_dron << " creado con " << carga_bateria_porcentaje << "% de bateria." << std::endl;
    }

    // Metodo virtual: puede ser sobrescrito por las clases derivadas
    // El 'virtual' permite el polimorfismo
    virtual void ejecutar_mision() {
        std::cout << id_dron << ": Iniciando mision generica." << std::endl;
        cargar_log(); // Un metodo que solo accede a traves de 'protected' o la propia clase
    }

    // Otro metodo virtual (aqui puro virtual, lo que hace a Dron una clase abstracta)
    // El '= 0' indica que este metodo DEBE ser implementado por cualquier clase derivada concreta.
    // Esto significa que no se pueden crear objetos directamente de la clase Dron.
    virtual void mostrar_tipo() = 0; // Metodo virtual puro

    int get_carga_bateria() const {
        return carga_bateria_porcentaje;
    }

    std::string get_id() const {
        return id_dron;
    }

protected: // Miembros protegidos: accesibles por la clase base y sus derivadas
    std::string id_dron;
    int carga_bateria_porcentaje;

    // Metodo protegido, solo accesible por la clase base y sus derivadas
    void cargar_log() {
        std::cout << "  (Log) Registrando actividad para " << id_dron << "." << std::endl;
    }
};

// --- 2. Clases Derivadas ---

// Clase DronAereo: Deriva de Dron, anade funcionalidades aereas
class DronAereo : public Dron {
public:
    // Constructor de DronAereo llama al constructor de la clase base Dron
    DronAereo(std::string id, int bateria_inicial, double altitud_max)
        : Dron(id, bateria_inicial), altitud_maxima(altitud_max), altitud_actual(0.0) {
        std::cout << "Dron Aereo " << id_dron << " con altitud maxima " << altitud_maxima << "m creado." << std::endl;
    }

    // Sobrescribe el metodo virtual de la clase base
    // 'override' es opcional pero buena practica para indicar que se sobrescribe un virtual
    void ejecutar_mision() override {
        if (carga_bateria_porcentaje < 20) {
            std::cout << id_dron << ": Bateria baja para mision aerea (" << carga_bateria_porcentaje << "%). Aterrizando." << std::endl;
            return;
        }
        std::cout << id_dron << ": Realizando mision de vigilancia aerea." << std::endl;
        volar(altitud_maxima / 2.0); // Vuela a la mitad de su altitud maxima
        carga_bateria_porcentaje -= 10; // Gasta bateria
        cargar_log(); // Accediendo a un metodo protected de la clase base
    }
    
    // Implementacion del metodo virtual puro de la clase base
    void mostrar_tipo() override {
        std::cout << id_dron << " es un Dron Aereo." << std::endl;
    }

    void volar(double altitud_deseada) {
        altitud_actual = altitud_deseada;
        std::cout << id_dron << ": Volando a " << altitud_actual << "m." << std::endl;
    }

private:
    double altitud_maxima;
    double altitud_actual;
};

// Clase DronTerrestre: Deriva de Dron, anade funcionalidades terrestres
class DronTerrestre : public Dron {
public:
    DronTerrestre(std::string id, int bateria_inicial, double velocidad_max)
        : Dron(id, bateria_inicial), velocidad_maxima(velocidad_max), velocidad_actual(0.0) {
        std::cout << "Dron Terrestre " << id_dron << " con velocidad maxima " << velocidad_maxima << "m/s creado." << std::endl;
    }

    // Sobrescribe el metodo virtual de la clase base
    void ejecutar_mision() override {
        if (carga_bateria_porcentaje < 15) {
            std::cout << id_dron << ": Bateria baja para mision terrestre (" << carga_bateria_porcentaje << "%). Volviendo a base." << std::endl;
            return;
        }
        std::cout << id_dron << ": Realizando mision de exploracion terrestre." << std::endl;
        moverse_por_tierra(velocidad_maxima * 0.8); // Se mueve al 80% de su velocidad maxima
        carga_bateria_porcentaje -= 5; // Gasta bateria
        cargar_log(); // Accediendo a un metodo protected
    }

    // Implementacion del metodo virtual puro de la clase base
    void mostrar_tipo() override {
        std::cout << id_dron << " es un Dron Terrestre." << std::endl;
    }

    void moverse_por_tierra(double velocidad_deseada) {
        velocidad_actual = velocidad_deseada;
        std::cout << id_dron << ": Moviendose por tierra a " << velocidad_actual << "m/s." << std::endl;
    }

private:
    double velocidad_maxima;
    double velocidad_actual;
};


// --- Funcion main: El punto de entrada del programa ---
int main() {
    std::cout << "--- Simulacion de Drones con Herencia y Polimorfismo ---" << std::endl;

    // NO PODEMOS crear un objeto directamente de Dron porque tiene un metodo virtual puro
    // Dron dron_generico("D-000", 50); // Esto daria error de compilacion

    // Crear objetos de las clases derivadas
    DronAereo dron_aereo("Drone_A1", 90, 150.0);
    DronTerrestre dron_terrestre("Rover_T1", 80, 5.0);

    std::cout << "\n--- Ejecutando Misiones Especificas ---" << std::endl;
    dron_aereo.ejecutar_mision();
    dron_terrestre.ejecutar_mision();

    // --- Demostracion de Polimorfismo ---
    // Usamos un vector de punteros a la clase base Dron.
    // Esto nos permite almacenar diferentes tipos de drones juntos.
    std::cout << "\n--- Demostracion de Polimorfismo (vector de punteros Dron*) ---" << std::endl;
    
    // NOTA: Cuando usas punteros de la clase base para objetos derivados (como Dron* p_dron_aereo = &dron_aereo),
    // si el objeto fue creado en el stack, no necesitas 'delete'.
    // Si lo creas con 'new', SÍ necesitarías 'delete' al final para liberar la memoria.
    // Lo veremos en detalle con punteros inteligentes.
    
    // Creamos punteros a la clase base Dron, pero que apuntan a objetos derivados
    Dron* p_dron_aereo = &dron_aereo;
    Dron* p_dron_terrestre = &dron_terrestre;

    std::vector<Dron*> flota_drones; // Un vector para almacenar punteros a Drones
    flota_drones.push_back(p_dron_aereo);
    flota_drones.push_back(p_dron_terrestre);
    
    // Simulamos un dron que se crea dinamicamente (lo veremos mas adelante, pero para el ejemplo de polimorfismo)
    DronAereo* dron_dinamico = new DronAereo("D_Temp", 70, 100.0);
    flota_drones.push_back(dron_dinamico);


    // Iteramos sobre el vector de punteros a Dron
    // Gracias al polimorfismo (metodo virtual 'ejecutar_mision'),
    // se llamara a la version correcta de 'ejecutar_mision' para cada tipo de dron.
    for (Dron* dron_actual : flota_drones) {
        std::cout << "\nProcesando Dron: " << dron_actual->get_id() << std::endl;
        dron_actual->mostrar_tipo();       // Llama al metodo virtual mostrar_tipo() de la clase derivada
        dron_actual->ejecutar_mision();    // Llama al metodo virtual ejecutar_mision() de la clase derivada
        std::cout << "Bateria restante: " << dron_actual->get_carga_bateria() << "%" << std::endl;
    }
    
    // IMPORTANTE: Liberar memoria si se usó 'new'
    delete dron_dinamico; // Libera la memoria asignada con 'new'
    dron_dinamico = nullptr; // Buena practica para evitar punteros 'dangling'

    std::cout << "\n--- Fin de la simulacion de herencia y polimorfismo ---" << std::endl;

    return 0;
}