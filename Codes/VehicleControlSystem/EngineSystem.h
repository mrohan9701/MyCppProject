#ifndef ENGINESYSTEM_H
#define ENGINESYSTEM_H

#include "VehicleSystem.h"
#include <iostream>

class EngineSystem : public VehicleSystem {
public:
    void start() override {
        std::cout << "[Engine] Ignition ON. Engine started.\n";
    }
    void stop() override {
        std::cout << "[Engine] Ignition OFF. Engine stopped.\n";
    }
    void status() const override {
        std::cout << "[Engine] All systems nominal.\n";
    }
    std::string name() const override {
        return "Engine";
    }
};

#endif