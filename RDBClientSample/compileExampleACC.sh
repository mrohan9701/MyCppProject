#!/bin/bash

# compile the RDB client example

g++ -o exampleACC ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleACC.cpp -I../VtdFramework/VtdToolkit/include
