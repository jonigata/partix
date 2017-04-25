// 2008/10/10 Naoyuki Hirayama

/*!
	@file	  FourLegs.hpp
	@brief	  <ŠT—v>

	<à–¾>
*/

#ifndef FOURLEGS_HPP_
#define FOURLEGS_HPP_

#include "Actor.h"

class DataProvider {
public:
    virtual ~DataProvider() {}

    virtual float GetTotalGripCoefficient() = 0;
    virtual float GetFrontGripCoefficient() = 0;
    virtual float GetRearGripCoefficient() = 0;
    virtual float GetSensoryBalanceSpeed() = 0;
    virtual float GetSensoryBalanceMax() = 0;
    virtual float GetSensoryBalanceDecrease() = 0;
    virtual float GetBalanceAngleMax() = 0;
    virtual float GetTurningAngleMax() = 0;
    virtual float GetBackbendAngleFactor() = 0;
    virtual float GetBankRatio() = 0;
    virtual float GetBalanceRatio() = 0;
    virtual float GetTurningRatio() = 0;
    virtual float GetBackbendRatio() = 0;
    virtual float GetBrakeAngle() = 0;
    virtual float GetFrictionFactor() = 0;
    virtual float GetAccelFactor() = 0;
    virtual float GetTurboThreshold() = 0;
    virtual float GetTurboMultiplier() = 0;
};

class FourLegs {
public:
    FourLegs();
    ~FourLegs();

    void Move();
    void SetAccel(const Vector&);
    void DoJump();
    void AddExternalForce(const Vector&);

    void SetDataProvider(DataProvider*);

protected:
    void OnAssignPhysie(Physie*, float, float);

private:
    int     id_;
    DataProvider* default_data_provider_;
    DataProvider* data_provider_;
    float   sensory_balance_;
    float   backbend_balance_;
    Matrix  orientation_;
    Vector  user_accel_;
    Vector  velocity_;

};

#endif // FOURLEGS_H_
