// src/radiant_vex.cpp
#include "vex.h"
#include <algorithm>  

using namespace vex;

extern brain Brain;
extern controller Controller1;

extern motor L1, L2, L3, R1, R2, R3;
extern motor intake1, intake2, arm;
extern inertial inertialSensor;
extern distance DistanceBack, DistanceFront, DistanceRight, DistanceLeft;
extern optical ColorSort;
extern rotation RotationArm;
extern digital_out switchGoal, MatchLoad, CSP, Door;

extern double wheelDiameter,gearRatio;
extern double move_speed, move_speedl, move_speedr, turn_speed, aim_speed, targetAngle, targetDistance;
extern double tp, ti, td;
extern double vkp;
extern double akp;
extern double intake1_target, intake2_target;
extern double arm_position, arm_target, armMax;
extern int turn_flag, drive_flag, thread_flag, motor_flag;
extern double last_angle;
extern const char* ColorMode;
extern bool running;

static double mv_kP=0.07, mv_kI=0.02, mv_kD=0.4, mv_calib=1.33;
static double fr_kP=2.10, fr_kI=0.10, fr_kD=5.00;
static double bk_kP=3.00, bk_kI=0.10, bk_kD=5.00;
static double lt_kP=5.00, lt_kD=0.00;
static double rt_kP=5.00, rt_kD=0.00;

namespace radiant_vex {

void setTurnPID(double p,double i,double d){ tp=p; ti=i; td=d; }
void setMovePID(double p,double i,double d,double calib){ mv_kP=p; mv_kI=i; mv_kD=d; mv_calib=calib; }
void setFrontPID(double p,double i,double d){ fr_kP=p; fr_kI=i; fr_kD=d; }
void setBackPID(double p,double i,double d){ bk_kP=p; bk_kI=i; bk_kD=d; }
void setLeftSidePID(double p,double d){ lt_kP=p; lt_kD=d; }
void setRightSidePID(double p,double d){ rt_kP=p; rt_kD=d; }
void setArmP(double kp){ akp=kp; }
void setWheel(double wheelIn){ wheelDiameter=wheelIn; }
void setVoltScale(double k){ vkp=k; }

static inline double clampd(double v,double lo,double hi){ return v<lo?lo:(v>hi?hi:v); }

void init() {
  L1.setStopping(coast); L2.setStopping(coast); L3.setStopping(coast);
  R1.setStopping(coast); R2.setStopping(coast); R3.setStopping(coast);
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) wait(50, msec);
}

void setMotorVoltage(double targetVoltL, double targetVoltR, double targetTurnS) {
  targetVoltL = targetVoltL / 100 * 12.0 * vkp;
  targetVoltR = targetVoltR / 100 * 12.0 * vkp;
  double maxV = 11.0, minV = -11.0, dead = 0.8;
  if (fabs(targetVoltL) > 0 && fabs(targetVoltL) < dead) targetVoltL = targetVoltL > 0 ? dead : -dead;
  if (fabs(targetVoltR) > 0 && fabs(targetVoltR) < dead) targetVoltR = targetVoltR > 0 ? dead : -dead;
  targetVoltL = clampd(targetVoltL, minV, maxV);
  targetVoltR = clampd(targetVoltR, minV, maxV);
  L1.spin(forward, targetVoltL + targetTurnS, volt);
  L2.spin(forward, targetVoltL + targetTurnS, volt);
  L3.spin(forward, targetVoltL + targetTurnS, volt);
  R1.spin(forward, targetVoltR - targetTurnS, volt);
  R2.spin(forward, targetVoltR - targetTurnS, volt);
  R3.spin(forward, targetVoltR - targetTurnS, volt);
}

void intakeThread() {
  intake1.spin(forward);
  intake2.spin(forward);
  double ct = Brain.Timer.time(seconds);
  while (running) {
    if(fabs(arm_position)>40) { intake1.spin(forward, -12, volt); intake2.spin(forward, -12, volt); }
    else { intake1.spin(forward, intake1_target/100.0*12, volt); intake2.spin(forward, intake2_target/100.0*12, volt); }
    if(ColorSort.isNearObject() && ((ColorSort.color()==red && strcmp(ColorMode,"blue")==0) || (ColorSort.color()==blue && strcmp(ColorMode,"red")==0))) {
      if(Brain.Timer.time(seconds)-ct>0.1) { CSP.set(true); wait(0.4,seconds); CSP.set(false); ct = Brain.Timer.time(seconds); }
    } else { ct = Brain.Timer.time(seconds); }
    arm_position = RotationArm.position(degrees);
    if(arm_position>360) arm_position-=360;
    double e = arm_target - arm_position;
    double as = e * akp;
    if (fabs(e) <= 1) { arm.setStopping(brake); arm.stop(); }
    else { if (as > armMax) as = armMax; if (as < -100) as = -100; arm.spin(forward, as/100.0*12, volt); }
    wait(0.01, seconds);
  }
}

void turnThread() {
  double ei=0, last = inertialSensor.heading(degrees);
  const double vmax = 8, vmin = -8;
  while (running) {
    if (motor_flag || turn_flag==0) { wait(0.1, seconds); continue; }
    double h = inertialSensor.heading(degrees);
    double e = targetAngle - h; if(e<-180) e+=360; if(e>180) e-=360;
    if (fabs(e)>3) ei = 0; else ei += e;
    double d = h - last; if (fabs(d)>180) d = 0; last = h;
    turn_speed = tp*e + ti*ei + td*d;
    if (turn_speed>vmax) turn_speed=vmax; if (turn_speed<vmin) turn_speed=vmin;
    setMotorVoltage(move_speedl, move_speedr, turn_speed);
    wait(0.05, seconds);
  }
}

void setPID(double np,double ni,double nd){ tp=np; ti=ni; td=nd; }
void resetPID(){ tp=0.18; ti=0.0; td=-0.3; }

void turn(double t_a,double t_t){
  targetAngle = t_a; if (targetAngle<0) targetAngle+=360;
  double t0 = Brain.Timer.time(seconds), t00=t0, t000=t0;
  while(fabs(targetAngle - inertialSensor.heading(degrees))>5 && fabs(targetAngle - inertialSensor.heading(degrees))<355 && Brain.Timer.time(seconds)-t0<t_t && Brain.Timer.time(seconds)-t000<2){
    if(fabs(targetAngle - inertialSensor.heading(degrees))>10 && fabs(targetAngle - inertialSensor.heading(degrees))<350) t00 = Brain.Timer.time(seconds);
    if(Brain.Timer.time(seconds)-t00>0.5) break;
    wait(0.05,seconds);
  }
  wait(0.05,seconds);
}

void wait_for_arm(){ while(fabs(arm_position-arm_target)>5) wait(0.05,seconds); }

void move(double sl,double sr,double t){ if(sl!=sr) turn_flag=0; move_speedl=sl; move_speedr=sr; wait(t,seconds); move_speedl=0; move_speedr=0; if(sl!=sr) turn_flag=1; }
void just_move(double sl,double sr,double t){ turn_flag=0; move_speedl=sl; move_speedr=sr; wait(t,seconds); move_speedl=0; move_speedr=0; }

void move_dis(double s,double distance,double ee){
  double ee_deg = (ee/(wheelDiameter*M_PI))*360.0;
  double te = (2/(wheelDiameter*M_PI))*360.0;
  const double kP = mv_kP, kI = mv_kI, kD = mv_kD;
  int cnt=0; double e=0, lastE=0, I=0, D=0;
  double sl=L2.position(degrees), sr=R2.position(degrees);
  double target=(distance/(wheelDiameter*M_PI))*360.0 / gearRatio;
  double t0 = Brain.Timer.time(seconds);
  while(true){
    double cur=((L2.position(degrees)-sl)+(R2.position(degrees)-sr))/2.0;
    e=target-cur; if(fabs(e)>100) I=0; else I+=e; if(I>500) I=500; if(I<-500) I=-500; if(fabs(e)<1) I=0;
    D=e-lastE; lastE=e;
    double adj=kP*e + kI*I + kD*D;
    move_speedl = fmax(fmin(adj, s), -s);
    move_speedr = fmax(fmin(adj, s), -s);
    if (fabs(e)<ee_deg) cnt++; else cnt=0;
    if (cnt>1) break;
    if (fabs(e)>te) t0 = Brain.Timer.time(seconds);
    if (Brain.Timer.time(seconds)-t0>0.5) break;
    wait(10,msec);
  }
  if(s>0){ move_speedl=-10; move_speedr=-10; wait(50,msec);} else { move_speedl=10; move_speedr=10; wait(50,msec);}
  move_speedl=0; move_speedr=0;
}

void move_dis_s(double s,double distance,double ee){
  double ee_deg = (ee/(wheelDiameter*M_PI))*360.0;
  double te = (2/(wheelDiameter*M_PI))*360.0;
  const double kP=0.07,kI=0.02,kD=0.4;
  int cnt=0; double e=0, lastE=0, I=0, D=0;
  double sl=L2.position(degrees), sr=R2.position(degrees);
  double target=(distance/(wheelDiameter*M_PI))*360.0 / gearRatio;
  double t0 = Brain.Timer.time(seconds);
  while(true){
    double cur=((L2.position(degrees)-sl)+(R2.position(degrees)-sr))/2.0;
    e=target-cur; if(fabs(e)>100) I=0; else I+=e; if(I>500) I=500; if(I<-500) I=-500; if(fabs(e)<1) I=0;
    D=e-lastE; lastE=e;
    double adj=kP*e + kI*I + kD*D;
    move_speedl = fmax(fmin(adj, s), -s);
    move_speedr = fmax(fmin(adj, s), -s);
    if (fabs(e)<ee_deg) cnt++; else cnt=0;
    if (cnt>1) break;
    if (fabs(e)>te) t0 = Brain.Timer.time(seconds);
    if (Brain.Timer.time(seconds)-t0>0.5) break;
    wait(10,msec);
  }
  move_speedl=0; move_speedr=0;
}

void move_frontDistance(double s,double target,double ee){
  double kP = fr_kP, kI = fr_kI, kD = fr_kD, prev=0, I=0; int br=0; double t0=Brain.Timer.time(seconds);
  while(true){
    double cur=DistanceFront.objectDistance(inches);
    double e=cur-target; double D=e-prev; if(fabs(e)>2) I=0; else I+=e;
    move_speedl=kP*e+kI*I+kD*D; move_speedr=move_speedl;
    if(move_speedl>fabs(s)) move_speedl=fabs(s); if(move_speedl<-fabs(s)) move_speedl=-fabs(s);
    if(move_speedr>fabs(s)) move_speedr=fabs(s); if(move_speedr<-fabs(s)) move_speedr=-fabs(s);
    if(fabs(e)<ee) br++; else br=0; if(br>3) break;
    if(fabs(e)>2) t0=Brain.Timer.time(seconds); if(Brain.Timer.time(seconds)-t0>1) break;
    prev=e; wait(0.01,seconds);
  }
  move_speedl=0; move_speedr=0;
}

void move_backDistance(double s,double target,double ee){
  double kP = bk_kP, kI = bk_kI, kD = bk_kD, prev=0, I=0; int br=0; double t0=Brain.Timer.time(seconds);
  while(true){
    double cur=DistanceBack.objectDistance(inches);
    double e=-(cur-target); double D=e-prev; if(fabs(e)>2) I=0; else I+=e;
    move_speedl=kP*e+kI*I+kD*D; move_speedr=move_speedl;
    if(move_speedl>fabs(s)) move_speedl=fabs(s); if(move_speedl<-fabs(s)) move_speedl=-fabs(s);
    if(move_speedr>fabs(s)) move_speedr=fabs(s); if(move_speedr<-fabs(s)) move_speedr=-fabs(s);
    if(fabs(e)<ee) br++; else br=0; if(br>3) break;
    if(fabs(e)>2) t0=Brain.Timer.time(seconds); if(Brain.Timer.time(seconds)-t0>1) break;
    prev=e; wait(0.01,seconds);
  }
  move_speedl=0; move_speedr=0;
}

void move_rightDistance(double s,double target,double t){
  double kP_right = rt_kP, kD_right = rt_kD, prev=0, t0=Brain.Timer.time(seconds);
  while(true){
    double d=DistanceRight.objectDistance(inches);
    double h=inertialSensor.heading(); double ch=h, cd=d;
    if(fabs(h-0)<=45){ch=h; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-90)<=45){ch=h-90; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-180)<=45){ch=h-180; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-270)<=45){ch=h-270; cd=d*cos(ch*M_PI/180.0);}
    double e=target-cd; double D=e-prev;
    double adj=kP_right*e+kD_right*D; if(s<0) adj=-adj;
    move_speedl=s-adj; move_speedr=s+adj;
    prev=e; if(Brain.Timer.time(seconds)-t0>t) break; wait(0.01,seconds);
  }
  move_speedl=0; move_speedr=0;
}

void move_leftDistance(double s,double target,double t){
  double kP_left = lt_kP, kD_left = lt_kD, prev=0, t0=Brain.Timer.time(seconds);
  while(true){
    double d=DistanceLeft.objectDistance(inches);
    double h=inertialSensor.heading(); double ch=h, cd=d;
    if(fabs(h-0)<=45){ch=h; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-90)<=45){ch=h-90; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-180)<=45){ch=h-180; cd=d*cos(ch*M_PI/180.0);}
    else if(fabs(h-270)<=45){ch=h-270; cd=d*cos(ch*M_PI/180.0);}
    double e=target-cd; double D=e-prev;
    double adj=kP_left*e+kD_left*D; if(s<0) adj=-adj;
    move_speedl=s+adj; move_speedr=s-adj;
    prev=e; if(Brain.Timer.time(seconds)-t0>t) break; wait(0.01,seconds);
  }
  move_speedl=0; move_speedr=0;
}

void intake(int s1,int s2){ intake1_target=s1; intake2_target=s2; }
void armCtrl(int tas,int aa){ armMax=tas; arm_target=aa; }

int maxTemp(){
  int m=0;
  m=std::max(m,(int)L1.temperature(percent));
  m=std::max(m,(int)L2.temperature(percent));
  m=std::max(m,(int)L3.temperature(percent));
  m=std::max(m,(int)R1.temperature(percent));
  m=std::max(m,(int)R2.temperature(percent));
  m=std::max(m,(int)R3.temperature(percent));
  return m;
}

void newThread(){
  while(running){
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1,1);
    if(strcmp(ColorMode,"red")==0) Controller1.Screen.print("Blue");
    else if(strcmp(ColorMode,"blue")==0) Controller1.Screen.print("Red");
    else if(strcmp(ColorMode,"off")==0) Controller1.Screen.print("Off");
    Controller1.Screen.setCursor(1,7);
    Controller1.Screen.print(maxTemp());
    wait(0.1,seconds);
  }
}

} // namespace radiant_vex
