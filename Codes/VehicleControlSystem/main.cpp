#include "EngineSystem.h"
#include "TransmissionSystem.h"
#include "ABSSystem.h"
#include "InfotainmentSystem.h"
#include <vector>
#include <iostream>

void simulateVehicleStartup(std::vector<VehicleSystem*>& systems) {
    std::cout << "\nStarting vehicle systems...\n";
    for (auto* sys : systems) sys->start();
}

void simulateVehicleShutdown(std::vector<VehicleSystem*>& systems) {
    std::cout << "\nShutting down vehicle systems...\n";
    for (auto* sys : systems) sys->stop();
}

void printSystemStatus(std::vector<VehicleSystem*>& systems) {
    std::cout << "\nVehicle System Status Report:\n";
    for (auto* sys : systems) {
        std::cout <<   sys->name() << ": ";
        sys->status();
    }
}

int main() {
    EngineSystem engine;
    TransmissionSystem transmission;
    ABSSystem abs;
    InfotainmentSystem infotainment;

    std::vector<VehicleSystem*> systems = { &engine, &transmission, &abs, &infotainment };

    simulateVehicleStartup(systems);
    printSystemStatus(systems);
    simulateVehicleShutdown(systems);

    return 0;
}