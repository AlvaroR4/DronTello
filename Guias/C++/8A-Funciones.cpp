#include <iostream> // Para std::cout, std::cin, std::endl
#include <cmath>    // Para std::sqrt

// --- 1. Definición de Estructura (fuera de main) ---
struct Punto3D {
    double x;
    double y;
    double z;
};

// --- 2. Prototipos de Funciones (Declaraciones) ---
// Es buena práctica declarar las funciones antes de main si su implementación está después.
// Esto permite a main "saber" que existen y cómo llamarlas.

// Funcion que no devuelve nada (void) y no recibe parametros
void mostrar_mensaje_bienvenida();

// Funcion que recibe un parametro por valor y devuelve un double
double convertir_metros_a_cm(double metros);

// Funcion que calcula la distancia entre dos puntos 3D (recibe structs por valor)
// Aunque para structs grandes es mas eficiente por referencia constante (const&),
// para este ejemplo simple por valor esta bien.
double calcular_distancia_entre_puntos(Punto3D p1, Punto3D p2);

// Funcion que modifica un valor recibido por referencia (double&)
void elevar_altitud_dron(double& altitud, double incremento);

// Funcion que simula el control de un motor del dron
void controlar_motor(int id_motor, double potencia_porcentaje);

// Funcion para verificar si la bateria es critica (recibe por valor, devuelve bool)
bool es_bateria_critica(int porcentaje_bateria, int umbral);

// --- Función main: El punto de entrada del programa ---
int main() {
    std::cout << "--- Simulacion de Dron con Funciones ---" << std::endl;

    // Llamada a funcion sin parametros ni retorno
    mostrar_mensaje_bienvenida();

    // --- Uso de funcion con retorno y parametro por valor ---
    double distancia_en_metros = 2.5;
    double distancia_en_cm = convertir_metros_a_cm(distancia_en_metros);
    std::cout << "2.5 metros son " << distancia_en_cm << " centimetros." << std::endl;

    // --- Uso de funcion con structs y retorno ---
    Punto3D posicion_actual = {10.0, 20.0, 5.0};
    Punto3D destino = {15.0, 28.0, 7.0};
    double distancia_al_destino = calcular_distancia_entre_puntos(posicion_actual, destino);
    std::cout << "Distancia al destino: " << distancia_al_destino << " metros." << std::endl;

    // --- Uso de funcion con parametro por referencia ---
    double altitud_dron = 10.0;
    std::cout << "Altitud inicial del dron: " << altitud_dron << "m" << std::endl;
    elevar_altitud_dron(altitud_dron, 5.0); // La funcion modificará altitud_dron original
    std::cout << "Altitud despues de elevar: " << altitud_dron << "m" << std::endl;

    // --- Uso de funcion con multiples parametros ---
    controlar_motor(1, 80.0); // Motor 1 al 80% de potencia
    controlar_motor(2, 65.5); // Motor 2 al 65.5% de potencia

    // --- Uso de funcion con retorno booleano y condicional ---
    int bateria_actual;
    std::cout << "\nIntroduce el porcentaje de bateria actual del dron: ";
    std::cin >> bateria_actual;

    if (es_bateria_critica(bateria_actual, 25)) { // Usamos un umbral de 25% para este ejemplo
        std::cout << "¡ATENCION! La bateria esta en nivel critico. Aterrizar inmediatamente." << std::endl;
    } else {
        std::cout << "Nivel de bateria OK. " << bateria_actual << "% restante." << std::endl;
    }

    std::cout << "\n--- Fin de la simulacion de funciones ---" << std::endl;

    return 0;
}

// --- 3. Implementaciones de Funciones (después de main o en otro archivo .cpp) ---

void mostrar_mensaje_bienvenida() {
    std::cout << "\nBienvenido al sistema de control de vuelo del dron Tello." << std::endl;
    std::cout << "Este programa demuestra el uso de funciones en C++." << std::endl;
}

double convertir_metros_a_cm(double metros) {
    return metros * 100.0;
}

double calcular_distancia_entre_puntos(Punto3D p1, Punto3D p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void elevar_altitud_dron(double& altitud, double incremento) {
    altitud += incremento;
    std::cout << "Elevando altitud en " << incremento << " metros." << std::endl;
}

void controlar_motor(int id_motor, double potencia_porcentaje) {
    std::cout << "Controlando Motor " << id_motor << ": Potencia al " << potencia_porcentaje << "%." << std::endl;
    // Aquí iría el código real para enviar la señal al motor
}

bool es_bateria_critica(int porcentaje_bateria, int umbral) {
    return porcentaje_bateria < umbral;
}