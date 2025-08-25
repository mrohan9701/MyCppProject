#!/bin/bash

# compile the RDB client example

g++ -o sampleClientRDB ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleConsole.cpp -I../VtdFramework/VtdToolkit/include
