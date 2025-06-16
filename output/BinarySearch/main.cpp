#include <iostream>
#include "binary.h"
using namespace std;

int main() {
    int arr[] = {2, 4, 6, 8, 10, 12};
    int size = sizeof(arr) / sizeof(arr[0]);
    int key;

    cout << "Enter number to search: ";
    cin >> key;
    
    // Call recursive binary search
    int result = binarySearch(arr, 0, size - 1, key);
    if (result != -1)
        cout << "Element found at index " << result << endl;
    else
        cout << "Element not found" << endl;

    return 0;
}
