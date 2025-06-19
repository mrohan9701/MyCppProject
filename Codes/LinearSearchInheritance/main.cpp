// main.cpp

#include <iostream>
#include "linear_search.h"
using namespace std;

int main() {
    LinearSearch ls;
    ls.getArray();

    int key;
    cout << "Enter element to search: ";
    cin >> key;

    int index = ls.searchForLoop(key);

    if (index != -1)
        cout << "Element found at index " << index << endl;
    else
        cout << "Element not found in the array." << endl;

    return 0;
}
