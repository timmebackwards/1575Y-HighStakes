#include "main.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>
#include <type_traits>

using namespace std;
using namespace pros;
using namespace lemlib;



// Motor configuration
#pragma region Motor Configuration
MotorGroup right_motors({8,10}, MotorGearset::blue);
MotorGroup left_motors({-7, -12}, MotorGearset::blue);
#pragma endregion

// Drivetrain settings
#pragma region Drivetrain Settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13.50, // 10 inch track width
                              Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
#pragma endregion


// Sensors
#pragma region Sensors
Imu imu(19);
Rotation horizontal_sensor(6);
Rotation vertical_sensor(5);

// horizontal tracking wheel
TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, Omniwheel::NEW_2, 3.5);
// vertical tracking wheel
TrackingWheel vertical_tracking_wheel(&vertical_sensor, Omniwheel::NEW_2, 4);

OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
#pragma endregion

// Operator Control
#pragma region Operator Control
Controller controller(E_CONTROLLER_MASTER);


// Robot configuration
adi::Pneumatics mogoClamp('H', false);
pros::Motor intake(-3);
pros::Motor secondStage(-20);
pros::Motor fish(0);

// PID controllers
#pragma region PID Controllers
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


#pragma endregion
// Create the chassis
#pragma region Chassis
Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
#pragma endregion

// Declare functions for selector...
void RedLeft();
void RedRight();
void BlueLeft();
void BlueRight();
void Skills();
// Autonomous Selector and Screen Display Variables
int selectedAuton = 1;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            const char* autonNames[] = {"None", "Red Left", "Red Right", "Blue Left", "Blue Right", "Skills Auto"};
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Drive Temp: %f", left_motors.get_temperature() + right_motors.get_temperature() / 2);
            pros::lcd::print(4, "Intake: %f", intake.get_temperature());
            pros::lcd::print(5, "Auton Mode: %s", autonNames[selectedAuton]);  // Display selected auton
            pros::delay(20);
        }
    });
}



// Disabled
#pragma region Disabled
void disabled() {}
#pragma endregion

// Competition Initialize
#pragma region Competition Initialize
void competition_initialize() {}
#pragma endregion

// Autonomous
#pragma region Autonomous
// path file name is "example.txt".
// "." is replaced with "_" to overcome c++ limitations

void logPosition(const std::string& positionName, const Pose& pose) {
    std::cout << positionName << " - X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
}

int toggle = 1;
void clampMogo() {
    toggle *= -1;
    if(toggle == 1){
        mogoClamp.retract();
    } else {
        mogoClamp.extend();
    }
    delay(250);
}

void autonomous() {
    // Select and execute the appropriate autonomous routine
    if (selectedAuton == 1) {
        RedLeft();
    } else if (selectedAuton == 2) {
        RedRight();
    } else if (selectedAuton == 3) {
        BlueLeft();
    } else if (selectedAuton == 4) {
        BlueRight();
    } else if (selectedAuton == 5) {
        Skills();
    } else {
        pros::lcd::print(1, "No autonomous selected.");
    }
}

void RedLeft()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -24, 1000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	clampMogo();
	pros::delay(500);
	intake.move(100);
	secondStage.move(100);
	pros::delay(500);
	intake.move(0);
	secondStage.move(0); 
	chassis.turnToHeading(90, 1000);
	chassis.waitUntilDone();
}

void RedRight()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -33, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
	clampMogo();
    pros::delay(500);
	intake.move(100);
	secondStage.move(100);
}

void BlueLeft()
{
    chassis.setPose(0, 0, 0);
    
}

void BlueRight()
{
    chassis.setPose(0, 0, 0);
    
}

void Skills()
{
    chassis.setPose(0, 0, 0);
}
#pragma endregion

int toWattage(int power)
{
	return power / 100 * 127;
}

void opcontrol() {
    while (true) {
        const char* autonNames[] = {"None", "Red Left", "Red Right", "Blue Left", "Blue Right", "Skills"};
        // Auton and color selector controls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton = (selectedAuton + 1) % 6;  // Adjust range if adding more autons
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton = (selectedAuton == 0) ? 5 : selectedAuton - 1;  // Adjust range if adding more autons
        }
        // Drivetrain control
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) clampMogo();
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake.move(127);
                secondStage.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(-127);
            secondStage.move(-127);
        } else {
            intake.move(0);
            secondStage.move(0);
        }

        pros::delay(25);
    }
}
#pragma endregion
