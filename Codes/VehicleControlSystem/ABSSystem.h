#ifndef ABSSYSTEM_H
#define ABSSYSTEM_H

#include "VehicleSystem.h"
#include <iostream>

class ABSSystem : public VehicleSystem {
public:
    void start() override {
        std::cout << "[ABS] ABS sensors activated.\n";
    }
    void stop() override {
        std::cout << "[ABS] ABS deactivated.\n";
    }
    void status() const override {
        std::cout << "[ABS] Ready for braking events.\n";
    }
    std::string name() const override {
        return "ABS";
    }
};

#endif