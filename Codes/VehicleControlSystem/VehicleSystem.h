#ifndef VEHICLESYSTEM_H
#define VEHICLESYSTEM_H

#include <string>

class VehicleSystem {
public:
    virtual ~VehicleSystem() {}
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void status() const = 0;
    virtual std::string name() const = 0;
};

#endif