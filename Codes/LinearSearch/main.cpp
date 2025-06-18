// main.cpp

#include <iostream>
#include "linear_search.h"
using namespace std;

int main() {
    int arr[] = {5, 10, 15, 20, 25, 30};
    int size = sizeof(arr) / sizeof(arr[0]);
    int key;

    cout << "Enter the number to search: ";
    cin >> key;

    // Using for loop
    int resultFor = linearSearchFor(arr, size, key);
    cout << "[For Loop] ";
    if (resultFor != -1)
        cout << "Element found at index " << resultFor << endl;
    else
        cout << "Element not found.\n";

    // Using while loop
    int resultWhile = linearSearchWhile(arr, size, key);
    cout << "[While Loop] ";
    if (resultWhile != -1)
        cout << "Element found at index " << resultWhile << endl;
    else
        cout << "Element not found.\n";

    // Using do-while loop
    int resultDoWhile = linearSearchDoWhile(arr, size, key);
    cout << "[Do-While Loop] ";
    if (resultDoWhile != -1)
        cout << "Element found at index " << resultDoWhile << endl;
    else
        cout << "Element not found.\n";

    return 0;
}
