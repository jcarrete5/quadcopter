#include <iostream>

#include <sys/utsname.h>


int main() {
    std::cout << "Hello, World!" << std::endl;
    utsname u{};
    uname(&u);
    std::cout << u.sysname << " " << u.machine << "\n";
    return 0;
}
