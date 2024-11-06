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
#include <cstdio>
#include <iostream>
#include <string>

using namespace std;
using namespace pros;
using namespace lemlib;

enum RingIntake {
	Blue,
	Red,
	None
};

// Motor configuration
#pragma region Motor Configuration
MotorGroup left_motors({-8,-9, 10}, MotorGearset::blue);
MotorGroup right_motors({-6, 4, 5}, MotorGearset::blue);
#pragma endregion

// Drivetrain settings
#pragma region Drivetrain Settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13.38, // 10 inch track width
                              Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
#pragma endregion

// Sensors
#pragma region Sensors
Imu imu(1);
Rotation horizontal_sensor(7);
Rotation vertical_sensor(11);

// horizontal tracking wheel
TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, Omniwheel::NEW_2, 2.54);
// vertical tracking wheel
TrackingWheel vertical_tracking_wheel(&vertical_sensor, Omniwheel::NEW_2, 3.74);

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
pros::adi::Pneumatics mogoClamp('A', false);
pros::Motor intake(-2);
pros::Motor secondStage(-21);
pros::Optical colorSensor(14);
pros::Rotation intakeRotation(12);
RingIntake currentRing = None;
RingIntake robotColor = None;
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
void red_Solo();
void red_Finals();

// Autonomous Selector and Screen Display Variables
int selectedAuton = 0; // 0: No auton, 1: red_Solo, 2: red_Finals
int selectedColor = 0; // 0: None, 1: Blue, 2: Red


// Initialize Function with Fixed Task
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    intakeRotation.set_position(0);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            string color;
            if (currentRing == Blue) {
                color = "Blue";
            } else if (currentRing == Red) {
                color = "Red";
            } else {
                color = "None";
            }
            pros::lcd::print(1, "X: %f", chassis.getPose().x);
            pros::lcd::print(2, "Y: %f", chassis.getPose().y);
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(4, "Color: %s", color.c_str());
            pros::lcd::print(1, "Auton Mode: %s", selectedAuton == 1 ? "Red Solo" : selectedAuton == 2 ? "Red Finals" : "None");
            pros::lcd::print(2, "Ring Color: %s", selectedColor == 1 ? "Blue" : selectedColor == 2 ? "Red" : "None");
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
ASSET(skillspath_txt);
ASSET(fullfieldtest_txt)
void autonomous() {
    // Select and execute the appropriate autonomous routine
    if (selectedAuton == 1) {
        red_Solo();
    } else if (selectedAuton == 2) {
        red_Finals();
    } else {
        pros::lcd::print(1, "No autonomous selected.");
    }
}

void red_Solo()
{
	chassis.setPose(0, 0, 180);
	logPosition("Set pose", chassis.getPose());
	chassis.moveToPoint(0, 16, 1000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	mogoClamp.extend();
	pros::delay(500);
	intake.move(100);
	secondStage.move(100);
	pros::delay(500);
	intake.move(0);
	secondStage.move(0); 

	chassis.turnToHeading(270, 1000);
	chassis.waitUntilDone();
	// intake.move(-100);
	// pros::delay(400);
	// chassis.setPose(0, 0, 0);
	// intake.move(100);
	// chassis.moveToPoint(0, 14.75, 1000, {.maxSpeed = 82});
	// chassis.waitUntilDone();
	// secondStage.move(100);
	// delay(500);
	// chassis.turnToHeading(170, 1000);
	// chassis.waitUntilDone();
	// chassis.setPose(0, 0, 0);
	// pros::delay(100);
	// intake.move(100);
	// secondStage.move(100);
	// chassis.moveToPoint(0, 38, 1000, {.maxSpeed = 100});
	// chassis.waitUntil(28);
	// mogoClamp.retract();
	// chassis.waitUntilDone();
	// pros::delay(400);
	// intake.move(0);
	// secondStage.move(0);
	// chassis.turnToHeading(45+180, 1000);
	// chassis.waitUntilDone();
	// chassis.setPose(0, 0, 0);
	// pros::delay(300);
	// chassis.moveToPose(0, -30, 0, 1000, {.forwards = false});
	// chassis.waitUntilDone();
	// intake.move(100);
	// secondStage.move(100);
	// pros::delay(550);
	// intake.move(0);
	// secondStage.move(0); 
}
void red_Finals()
{
}
#pragma endregion

int toWattage(int power)
{
	return power / 100 * 127;
}

void opcontrol() {
    const char* autonNames[] = {"None", "Red Solo", "Red Finals"};
    const char* colorNames[] = {"None", "Blue", "Red"};

    while (true) {
        // Auton and color selector controls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton = (selectedAuton + 1) % 3;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton = (selectedAuton == 0) ? 2 : selectedAuton - 1;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            selectedColor = (selectedColor + 1) % 3;
        }

        // Set robot color based on selection
        if (selectedColor == 1) {
            robotColor = Blue;
        } else if (selectedColor == 2) {
            robotColor = Red;
        } else {
            robotColor = None;
        }

        // Display selected auton mode and robot color on the controller
        controller.clear();
        controller.set_text(0, 0, autonNames[selectedAuton]);
        controller.set_text(1, 0, colorNames[selectedColor]);

        // Detect ring color
        colorSensor.set_led_pwm(100);
        if (colorSensor.get_proximity() >= 60) {
            if (colorSensor.get_hue() >= 180 && colorSensor.get_hue() <= 220) {
                currentRing = Blue;
            } else if (colorSensor.get_hue() >= 7 && colorSensor.get_hue() <= 20) {
                currentRing = Red;
            } else {
                currentRing = None;
            }
        } else {
            currentRing = None;
        }

        // Drivetrain control
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

        // Toggle mogoClamp with L1 button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) mogoClamp.toggle();

        // Intake management with auto-sorting
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake.move(100);
                secondStage.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Manual reverse intake with R2 button
            intake.move(-127);
            secondStage.move(-127);
        } else {
            // Stop intake motors when no input
            intake.move(0);
            secondStage.move(0);
        }

        pros::delay(25);
    }
}
#pragma endregion
