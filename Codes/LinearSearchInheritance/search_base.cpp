// search_base.cpp

#include <iostream>
#include "search_base.h"
using namespace std;

void SearchBase::getArray() {
    cout << "Enter number of elements: ";
    cin >> size;

    cout << "Enter " << size << " elements:\n";
    for (int i = 0; i < size; i++) {
        cin >> arr[i];
    }

    // Optional: Show array after input
    cout << "You entered: ";
    for (int i = 0; i < size; i++) {
        cout << arr[i] << " ";
    }
    cout << endl;
}

int SearchBase::getSize() const {
    return size;
}

int* SearchBase::getArrayPointer() {
    return arr;
}
