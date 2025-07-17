#include <iostream>

int main() {
    int contador_errores = 0;
    std::cout << "Errores iniciales: " << contador_errores << std::endl;

    contador_errores += 1; // Un error detectado
    std::cout << "Después del primer error: " << contador_errores << std::endl;

    contador_errores *= 2; // El error se duplica por alguna razón
    std::cout << "Después de duplicar errores: " << contador_errores << std::endl;

    double bateria_porcentaje = 100.0;
    bateria_porcentaje -= 10.5; // Batería baja un 10.5%
    std::cout << "Batería restante: " << bateria_porcentaje << "%" << std::endl;

    return 0;
}