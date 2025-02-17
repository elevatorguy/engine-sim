#ifndef ATG_ENGINE_SIM_TRANSMISSION_H
#define ATG_ENGINE_SIM_TRANSMISSION_H

#include "vehicle.h"
#include "engine.h"
#include "scs.h"
#include "utilities.h"

class Transmission {
    public:
        struct Parameters {
            int GearCount;
            const double *GearRatios;
            double MaxClutchTorque;
        };

    public:
        Transmission();
        ~Transmission();

        void initialize(const Parameters &params);
        void update(double dt);
        void addToSystem(
            atg_scs::RigidBodySystem *system,
            atg_scs::RigidBody *rotatingMass,
            Vehicle *vehicle,
            Engine *engine);
        void changeGear(int newGear);
        void slideGear(void);
        inline int getGear() const { return m_gear; }
        inline void setClutchPressure(double pressure) { m_clutchPressure = pressure; }
        inline double getClutchPressure() const { return m_clutchPressure; }
        inline void setDiskPosition(double position) { if (maySlide) { m_diskPosition = clamp(position); } }
        inline double getDiskPosition() const { return m_diskPosition; }
        void unlockSlidingDisk();
        void lockSlidingDisk();
        inline bool isVariable() { return maySlide; }
    protected:
        atg_scs::ClutchConstraint m_clutchConstraint;
        atg_scs::RigidBody *m_rotatingMass;
        Vehicle *m_vehicle;

        Engine *m_engine;
        double m_diskPosition;
        double m_diskMin;
        double m_diskMax;

        int m_gear;
        int m_newGear;
        int m_gearCount;
        double *m_gearRatios;
        double m_maxClutchTorque;
        double m_clutchPressure;
        bool maySlide;
};

#endif /* ATG_ENGINE_SIM_TRANSMISSION_H */
