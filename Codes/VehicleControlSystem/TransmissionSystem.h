#ifndef TRANSMISSIONSYSTEM_H
#define TRANSMISSIONSYSTEM_H

#include "VehicleSystem.h"
#include <iostream>

class TransmissionSystem : public VehicleSystem {
public:
    void start() override {
        std::cout << "[Transmission] Gears engaged.\n";
    }
    void stop() override {
        std::cout << "[Transmission] Transmission disengaged.\n";
    }
    void status() const override {
        std::cout << "[Transmission] Gearbox temp normal.\n";
    }
    std::string name() const override {
        return "Transmission";
    }
};

#endif