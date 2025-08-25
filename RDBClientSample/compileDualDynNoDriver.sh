#!/bin/bash

# compile the RDB client example

g++ -o dualDynNoDriver ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc DualDynNoDriver.cpp -I../VtdFramework/VtdToolkit/include
