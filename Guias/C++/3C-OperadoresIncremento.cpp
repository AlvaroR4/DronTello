#include <iostream>

int main() {
    int puntos_clave_detectados = 5;

    // Post-incremento
    std::cout << "Post-incremento:" << std::endl;
    int a = puntos_clave_detectados++; // 'a' toma el valor actual de puntos_clave_detectados (5), LUEGO se incrementa puntos_clave_detectados a 6
    std::cout << "Valor de 'a': " << a << std::endl; // Salida: 5
    std::cout << "Valor de puntos_clave_detectados: " << puntos_clave_detectados << std::endl; // Salida: 6

    puntos_clave_detectados = 5; // Reiniciar para el siguiente ejemplo

    // Pre-incremento
    std::cout << "\nPre-incremento:" << std::endl;
    int b = ++puntos_clave_detectados; // puntos_clave_detectados se incrementa a 6, LUEGO 'b' toma ese nuevo valor (6)
    std::cout << "Valor de 'b': " << b << std::endl; // Salida: 6
    std::cout << "Valor de puntos_clave_detectados: " << puntos_clave_detectados << std::endl; // Salida: 6

    // Uso simple (fuera de una expresión, no hay diferencia)
    puntos_clave_detectados--; // puntos_clave_detectados ahora es 5
    std::cout << "\nDespués de un decremento simple: " << puntos_clave_detectados << std::endl; // Salida: 5

    return 0;
}