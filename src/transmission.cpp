#include "../include/transmission.h"

#include "../include/units.h"

#include <cmath>

Transmission::Transmission() {
    m_gear = -1;
    m_newGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_clutchPressure = 0.0;
    m_diskPosition = 1.0;
    m_engine = nullptr;
    maySlide = false;
    m_diskMin = 0.7;
    m_diskMax = 3.1;
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }

    m_gearRatios = nullptr;
}

void Transmission::initialize(const Parameters &params) {
    m_gearCount = params.GearCount;
    m_maxClutchTorque = params.MaxClutchTorque;
    m_gearRatios = new double[params.GearCount];
    memcpy(m_gearRatios, params.GearRatios, sizeof(double) * m_gearCount);
    if (params.GearCount > 1)
    {
        m_diskMax = m_gearRatios[0];
        m_diskMin = m_gearRatios[params.GearCount - 1];
    }
}

void Transmission::update(double dt) {
    if (m_gear == -1) {
        m_clutchConstraint.m_minTorque = 0;
        m_clutchConstraint.m_maxTorque = 0;
    }
    else {
        m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure;
        m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure;
    }

    if (maySlide) {
        slideGear();
    }
}

void Transmission::addToSystem(
    atg_scs::RigidBodySystem *system,
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    m_rotatingMass = rotatingMass;
    m_vehicle = vehicle;

    m_clutchConstraint.setBody1(&engine->getOutputCrankshaft()->m_body);
    m_clutchConstraint.setBody2(m_rotatingMass);

    system->addConstraint(&m_clutchConstraint);

    m_engine = engine;
}

void Transmission::enableSlidingDisk() {
    maySlide = true;
    if(m_gear > -1) {
        m_gear = 0;
    }
}

void Transmission::changeGear(int newGear) {
    if (maySlide && newGear > 0) return;
    if (newGear < -1 || newGear >= m_gearCount) return;
    else if (newGear != -1) {
        const double m_car = m_vehicle->getMass();
        const double gear_ratio = m_gearRatios[newGear];
        const double diff_ratio = m_vehicle->getDiffRatio();
        const double tire_radius = m_vehicle->getTireRadius();
        const double f = tire_radius / (diff_ratio * gear_ratio);

        const double new_I = m_car * f * f;
        const double E_r =
            0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
        const double new_v_theta = m_rotatingMass->v_theta < 0
            ? -std::sqrt(E_r * 2 / new_I)
            : std::sqrt(E_r * 2 / new_I);

        m_rotatingMass->I = new_I;
        m_rotatingMass->p_x = m_rotatingMass->p_y = 0;
        m_rotatingMass->m = m_car;
        m_rotatingMass->v_theta = new_v_theta;
    }
    
    m_gear = newGear;
}

void Transmission::slideGear(void) {
    double current_rpm = m_engine->getRpm();
    double redline = m_engine->getRedline(); //todo: algorithm
    
    const double m_car = m_vehicle->getMass();
    const double diff_ratio = m_vehicle->getDiffRatio();
    const double tire_radius = m_vehicle->getTireRadius();
    const double f = tire_radius / (diff_ratio * (clamp(m_diskPosition)*(m_diskMax - m_diskMin) + m_diskMin));
    
    const double new_I = m_car * f * f;
    const double E_r = 0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
    const double new_v_theta = m_rotatingMass->v_theta < 0 ? -std::sqrt(E_r * 2 / new_I) : std::sqrt(E_r * 2 / new_I);

    m_rotatingMass->I = new_I;
    m_rotatingMass->p_x = m_rotatingMass->p_y = 0;
    m_rotatingMass->m = m_car;
    m_rotatingMass->v_theta = new_v_theta;
}

