#ifndef PLANT_MODEL_HPP
#define PLANT_MODEL_HPP

#include "casadi/casadi.hpp"

// define the system dynamics
class System
{
public:
    System();
    ~System(){}

    casadi::Function getDynamics(){return NumDynamics;}
    casadi::Function getOutputMapping(){return OutputMap;}
private:
    casadi::SX state;
    casadi::SX control;
    casadi::SX Dynamics;

    casadi::Function NumDynamics;
    casadi::Function OutputMap;
};

System::System()
{
    casadi::SX u = casadi::SX::sym("u");
    casadi::SX D = casadi::SX::sym("D");
    casadi::SX v = casadi::SX::sym("v"); // resource rate

    casadi::SX x = casadi::SX::sym("x", 2);
    casadi::SX e = casadi::SX::sym("e"); // resources

    state = casadi::SX::vertcat({x,e});
    control = casadi::SX::vertcat({u,D,v});

    const double gamma = 0.25; // resource_recharge_rate

    casadi::SX mu_D = v; // first case

    Dynamics = casadi::SX::vertcat({D * u, D * x(0), D * (gamma - v)});
    NumDynamics = casadi::Function("Dynamics", {state, control}, {Dynamics});

    /** define output mapping */
    OutputMap = casadi::Function("Map",{state}, {x(1)});
}



#endif