#include <iostream>
#include "quicksort.h"

int main() {
    int arr[] = {8, 4, 7, 3, 9, 2};
    int n = sizeof(arr) / sizeof(arr[0]);

    std::cout << "Original array: ";
    for (int i = 0; i < n; i++)
        std::cout << arr[i] << " ";

    quickSort(arr, 0, n - 1);

    std::cout << "\nSorted array: ";
    for (int i = 0; i < n; i++)
        std::cout << arr[i] << " ";

    std::cout << std::endl;
    return 0;
}
