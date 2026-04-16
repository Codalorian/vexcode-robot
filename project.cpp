#pragma region VEXcode Generated Robot Configuration
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

brain Brain;

// Motors
motor leftMotorA = motor(PORT8, 1, false);
motor leftMotorB = motor(PORT7, 1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);

motor rightMotorA = motor(PORT2, 1, true);
motor rightMotorB = motor(PORT1, 1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);

drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 200, 173, 76, mm, 1);

sonar ultrasound10 = sonar(PORT10);

// Random seed
void initializeRandomSeed(){
  wait(100, msec);
  int seed = int(Brain.Timer.system() * 1000 + Brain.Battery.voltage());
  srand(seed);
}

void vexcodeInit() {
  initializeRandomSeed(); 
}
#pragma endregion

#include "vex.h"
using namespace vex;

// Constants
const int DETECT_DISTANCE = 254;   // 10 inches (mm)
const int COMMIT_DEGREES = 914;   // ~20 inches of travel

int whenStarted1() {
  while (true) {

    // Drive forward
    Drivetrain.drive(forward);

    // If object is close
    if (ultrasound10.foundObject() && ultrasound10.distance(mm) < DETECT_DISTANCE) {

      Drivetrain.stop();

      // Random turn
      int randomTurn = rand() % 2;

      if (randomTurn == 0) {
        Drivetrain.turnFor(left, 90, degrees);
      } else {
        Drivetrain.turnFor(right, 90, degrees);
      }

      // Reset motor positions (in degrees)
      LeftDriveSmart.setPosition(0, degrees);
      RightDriveSmart.setPosition(0, degrees);

      // Move forward up to ~20 inches or until obstacle
      while (true) {
        Drivetrain.drive(forward);

        double avgDegrees =
          (fabs(LeftDriveSmart.position(degrees)) +
           fabs(RightDriveSmart.position(degrees))) / 2;

        // Stop after traveling enough distance
        if (avgDegrees >= COMMIT_DEGREES) {
          break;
        }

        // Stop early if another object appears
        if (ultrasound10.foundObject() && ultrasound10.distance(mm) < DETECT_DISTANCE) {
          break;
        }

        wait(20, msec);
      }

      Drivetrain.stop();
    }

    wait(20, msec);
  }
  return 0;
}

int main() {
  vexcodeInit();
  whenStarted1();
}
