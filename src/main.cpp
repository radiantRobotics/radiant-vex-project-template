#include "vex.h"
#include "radiant_vex.h"
using namespace vex;
using namespace radiant_vex;



brain Brain;
controller Controller1(primary);

motor L1(PORT1, ratio6_1, true);
motor L2(PORT2, ratio6_1, true);
motor L3(PORT3, ratio6_1, true);
motor R1(PORT8, ratio6_1, false);
motor R2(PORT9, ratio6_1, false);
motor R3(PORT10, ratio6_1, false);
motor intake1(PORT6, ratio18_1, false);
motor intake2(PORT17, ratio18_1, true);
motor arm(PORT15, ratio18_1, false);
rotation RotationArm(PORT11, false);
inertial inertialSensor(PORT7);
distance DistanceBack(PORT4);
distance DistanceFront(PORT16);
distance DistanceRight(PORT20);
distance DistanceLeft(PORT5);
optical ColorSort(PORT19);
digital_out switchGoal(Brain.ThreeWirePort.A);
digital_out MatchLoad(Brain.ThreeWirePort.B);
digital_out CSP(Brain.ThreeWirePort.C);
digital_out Door(Brain.ThreeWirePort.D);

double wheelDiameter = 2.75 , gearRatio = 36/48*600;
double move_speed = 0, move_speedl = 0, move_speedr = 0, turn_speed = 0, aim_speed = 0, SpeedLeft = 0, SpeedRight = 0, targetAngle = 0, targetDistance = 0, speedL = 0, speedR = 0;
double tp = 0.17, ti = 0.0 , td = -0.25;
double vkp = 1.3, previousError = 0.0;
double akp = 2;
double intake3_target = 0,intake4_target = 0,intake1_target = 0,intake2_target = 0;
double arm_position = 0,arm_target = 0,armMax = 0;
int intake3_flag = 1,intake4_flag = 1,intake1_flag = 1,intake2_flag = 1;
int turn_flag = 1, drive_flag = 1,thread_flag = 0,motor_flag = 0;
double last_angle = 0;
const char* ColorMode = "red";
bool running = true;

competition Competition;

void drive()
{
  
  L1.setStopping(coast);
  L2.setStopping(coast);
  L3.setStopping(coast);
  R1.setStopping(coast);
  R2.setStopping(coast);
  R3.setStopping(coast);
  double maxSpeed=12;
  ColorMode = "off";
  int load_flag=0 , door_flag=0 , switch_flag=0;
  double lx=0,ly=0,rx=0,ry=0;
  float SpeedLeft,SpeedRight;
  turn_flag=0;
  drive_flag=1;
  while(1)
  {
    if (drive_flag == 0) { wait(20, msec); continue; }  
    wait(0.02,seconds);

    lx=Controller1.Axis4.position();
    ly=Controller1.Axis3.position();
    rx=Controller1.Axis1.position();
    ry=Controller1.Axis2.position();

    move_speed=ly*1;
    turn_speed=-rx*0.6;

    if(Controller1.ButtonDown.pressing())
    {
      arm_target = -20;
      wait(0.2,seconds);
      RotationArm.setPosition(0,degrees);
    }

    if(Controller1.ButtonY.pressing()){
      intake(100,100);
      Door.set(true);
      armCtrl(100,70);
    }
    else{
      if(Controller1.ButtonL2.pressing())
      {
        armCtrl(30,0);
      }
      else if(Controller1.ButtonR2.pressing())
      {
        intake(100,100);

        if(Controller1.ButtonR1.pressing())
        {
          door_flag=0;
          Door.set(true);
          if(switch_flag)
          {
            akp = 0.5;
            armCtrl(100,140);
          }
          else
          {
            akp = 2;
            armCtrl(100,130);
          }
        }
        else
        {
          akp = 2;
          if(RotationArm.position(degrees)<10)
          {
          }
          armCtrl(0,0);
        }
      }
      else if(Controller1.ButtonR1.pressing())
      {
        akp = 2;
        armCtrl(0,0);
        intake(-70,-70);
      }
      else
      {
        Door.set(door_flag);
        akp = 2;
        armCtrl(0,0);
        if(RotationArm.position(degrees)<10)
        {
        }
        intake(0,0);
      }
    }

    if(Controller1.ButtonA.pressing())
    {
      door_flag = !door_flag;
      while(Controller1.ButtonA.pressing());
    }
    if(Controller1.ButtonL1.pressing())
    {
      load_flag = !load_flag;
      while(Controller1.ButtonL1.pressing());
    }
    if(Controller1.ButtonX.pressing())
    {
      switch_flag = !switch_flag;
      while(Controller1.ButtonX.pressing());
    }

    switchGoal.set(switch_flag);
    MatchLoad.set(load_flag);

    SpeedLeft =(move_speed-turn_speed-aim_speed)/100*12;
    SpeedRight=(move_speed+turn_speed+aim_speed)/100*12;

    if (SpeedLeft > maxSpeed)
    {
      SpeedLeft = maxSpeed;
      SpeedRight = maxSpeed - (SpeedLeft - SpeedRight)*1.3;
    }
    else if (SpeedRight > maxSpeed)
    {
      SpeedRight = maxSpeed;
      SpeedLeft = maxSpeed - (SpeedRight - SpeedLeft)*1.3;
    }

    L1.spin(forward, SpeedLeft, volt);
    L2.spin(forward, SpeedLeft, volt);
    L3.spin(forward, SpeedLeft, volt);
    R1.spin(forward, SpeedRight, volt);
    R2.spin(forward, SpeedRight, volt);
    R3.spin(forward, SpeedRight, volt);
  }
}


void auton() 
{
  ColorMode = "off";
  turn_flag = 1;
  drive_flag = 0;
  move(50,50,1);
  move_dis(20,15,5);
  turn(90,99);
  Door.set(true);
  wait(0.1,seconds);
}

void setup(int ta0) 
{
  radiant_vex::init(); 
  targetAngle = ta0; 
  inertialSensor.setHeading(targetAngle, degrees);
  wait(300, msec);
  thread t1(turnThread);
  thread t2(intakeThread);
  thread t3(newThread);
}

int main() 
{
  setup(0);
  Competition.autonomous(auton);
  Competition.drivercontrol(drive);
  while (true) wait(100, msec);
}
