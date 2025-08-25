#!/bin/tcsh

# compile the RDB client example

g++ -o sampleClientRDBDriverCtrl ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleConsoleDriverCtrl.cpp -I../VtdFramework/VtdToolkit/include
g++ -o sampleClientRDBDriverCtrlTorque ../VtdFramework/VtdToolkit/src/Common/RDBHandler.cc ExampleConsoleDriverCtrlTorque.cpp -I../VtdFramework/VtdToolkit/include
