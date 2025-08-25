#!/bin/tcsh

# compile the RDB client example

g++ -o sampleClientProxyPkg ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleConsoleProxyPkg.cpp -I../VtdFramework/VtdToolkit/include
