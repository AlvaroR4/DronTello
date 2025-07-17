#include <iostream>

// Con esta línea, le decimos al compilador que queremos usar
// todos los nombres del espacio de nombres 'std' directamente.
using namespace std;

int main() {
    // Ahora no necesitamos std::
    cout << "¡Hola, mundo sin std::!" << endl;
    return 0;
}