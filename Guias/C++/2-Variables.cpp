#include <iostream> // Para usar std::cout y std::endl

int main() {
    // Declaración de una variable entera
    int edad; // 'edad' es una variable de tipo entero

    // Inicialización de la variable
    edad = 30; // Asignamos el valor 30 a 'edad'

    // Declaración e inicialización en la misma línea (¡buena práctica!)
    double peso_dron = 1.25; // Peso en kilogramos, usamos double para precisión

    // Variable booleana
    bool dron_encendido = true;

    // Variable para un contador de frames
    int numero_frames = 0;

    // Variable para una coordenada X
    float coordenada_x = 15.75f; // La 'f' al final indica que es un float

    // Imprimir los valores de las variables
    std::cout << "Edad: " << edad << " años" << std::endl;
    std::cout << "Peso del dron: " << peso_dron << " kg" << std::endl;
    std::cout << "¿Dron encendido? " << dron_encendido << std::endl; // true se imprime como 1, false como 0
    std::cout << "Frames procesados: " << numero_frames << std::endl;
    std::cout << "Coordenada X: " << coordenada_x << std::endl;

    return 0;
}