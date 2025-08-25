#!/bin/bash

# compile the RDB client example

g++ -o sampleVehDynRDB ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleVehDynInteg.cpp -I../VtdFramework/VtdToolkit/include
