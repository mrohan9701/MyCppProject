// linear_search.cpp

#include "linear_search.h"

// Linear Search using for loop
int linearSearchFor(int arr[], int size, int key) {
    for (int i = 0; i < size; i++) {
        if (arr[i] == key) {
            return i;
        }
    }
    return -1;
}

// Linear Search using while loop
int linearSearchWhile(int arr[], int size, int key) {
    int i = 0;
    while (i < size) {
        if (arr[i] == key) {
            return i;
        }
        i++;
    }
    return -1;
}

// Linear Search using do-while loop
int linearSearchDoWhile(int arr[], int size, int key) {
    int i = 0;
    if (size <= 0) return -1;

    do {
        if (arr[i] == key) {
            return i;
        }
        i++;
    } while (i < size);

    return -1;
}
