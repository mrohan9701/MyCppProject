// linear_search.cpp

#include "linear_search.h"

int LinearSearch::searchForLoop(int key) {
    for (int i = 0; i < size; i++) {
        if (arr[i] == key) {
            return i; // Element found
        }
    }
    return -1; // Not found
}
