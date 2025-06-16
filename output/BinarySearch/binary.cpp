#include "binary.h"

// Recursive function to perform binary search on a sorted array
int binarySearch(int arr[], int left, int right, int key) {
    if (left > right) return -1;

    int mid = (left + right) / 2;

    if (arr[mid] == key) return mid;
    else if (arr[mid] > key)
        return binarySearch(arr, left, mid - 1, key);
    else
        return binarySearch(arr, mid + 1, right, key);
}
