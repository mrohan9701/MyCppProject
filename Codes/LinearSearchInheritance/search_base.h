// search_base.h

#ifndef SEARCH_BASE_H
#define SEARCH_BASE_H

class SearchBase {
protected:
    int arr[100];
    int size;

public:
    void getArray();
    int getSize() const;
    int* getArrayPointer();
};

#endif
