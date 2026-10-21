#include <cstdio>

int main() {
    int sum = 2 + 2;
    if (sum != 4) {
        std::printf("FAIL: arithmetic is broken\n");
        return 1;
    }
    std::printf("PASS: build pipeline works\n");
    return 0;
}
