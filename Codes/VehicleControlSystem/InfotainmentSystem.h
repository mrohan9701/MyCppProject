#ifndef INFOTAINMENTSYSTEM_H
#define INFOTAINMENTSYSTEM_H

#include "VehicleSystem.h"
#include <iostream>

class InfotainmentSystem : public VehicleSystem {
public:
    void start() override {
        std::cout << "[Infotainment] Media and Navigation ON.\n";
    }
    void stop() override {
        std::cout << "[Infotainment] Shutting down screens.\n";
    }
    void status() const override {
        std::cout << "[Infotainment] GPS signal acquired, playing audio.\n";
    }
    std::string name() const override {
        return "Infotainment";
    }
};

#endif