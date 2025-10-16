#ifndef RADIANT_VEX_H
#define RADIANT_VEX_H

#include "vex.h"

namespace radiant_vex {

// System / init
void init();

// Low-level control
void setMotorVoltage(double targetVoltL, double targetVoltR, double targetTurnS);

// Threads
void intakeThread();
void turnThread();
void newThread();

// Generic PID config
void setPID(double p, double i, double d);
void resetPID();

// Turn / Move APIs
void turn(double target_angle_deg, double timeout_s);
void wait_for_arm();
void move(double sl, double sr, double t);
void just_move(double sl, double sr, double t);

// Distance-based drive
void move_dis(double s, double distance_in, double tol_in);
void move_dis_s(double s, double distance_in, double tol_in);
void move_frontDistance(double s, double target_in, double tol_in);
void move_backDistance(double s, double target_in, double tol_in);
void move_rightDistance(double s, double target_in, double t);
void move_leftDistance(double s, double target_in, double t);

// Subsystems
void intake(int s1, int s2);
void armCtrl(int tas, int aa);

// Telemetry
int  maxTemp();

// Tunables
void setTurnPID(double p, double i, double d);
void setMovePID(double p, double i, double d, double calib);
void setFrontPID(double p, double i, double d);
void setBackPID(double p, double i, double d);
void setLeftSidePID(double p, double d);
void setRightSidePID(double p, double d);
void setArmP(double kp);
void setWheel(double wheelIn);
void setVoltScale(double k);

} // namespace radiant_vex

#endif // RADIANT_VEX_H
