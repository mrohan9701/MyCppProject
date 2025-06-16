// factorial.cpp
#include "factorial.h"

int factorial(int n) {
    if (n <= 1)
        return 1;
    return n * factorial(n - 1);
}

/*int factorial(int n) {
    int result = 1;
    for (int i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
} */
