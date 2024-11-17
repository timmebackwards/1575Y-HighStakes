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
adi::Pneumatics mogoClamp('A', false);
pros::Motor intake(-2);
pros::Motor secondStage(-21);
pros::Optical colorSensor(14);
pros::Rotation intakeRotation(12);
RingIntake currentRing = None;
adi::Pneumatics doinker('B', false);

bool isRingDetected() {
    // Check if proximity is high enough to detect a ring
    if (colorSensor.get_proximity() >= 60) {
        int hue = colorSensor.get_hue();
        // Check if hue is within range for either blue or red
        if ((hue >= 180 && hue <= 220) || (hue >= 7 && hue <= 20)) {
            return true;  // Either blue or red ring is detected
        }
    }
    return false;  // No ring detected
}

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

// Autonomous Selector and Screen Display Variables
int selectedAuton = 1;


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
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

            // Update the auton names array and display selectedAuton
            const char* autonNames[] = {"None", "Red Left", "Red Right", "Blue Left", "Blue Right"};
            pros::lcd::print(1, "X: %f", chassis.getPose().x);
            pros::lcd::print(2, "Y: %f", chassis.getPose().y);
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(4, "Color: %s", color.c_str());
            pros::lcd::print(5, "Auton Mode: %s", autonNames[selectedAuton]);  // Display selected auton
            pros::lcd::print(6, "Drive Temp: %f", left_motors.get_temperature() + right_motors.get_temperature() / 2);
            pros::lcd::print(7, "Intake: %f", intake.get_temperature());
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

int doinkWow = 1;
void doink() {
    doinkWow *= -1;
    if(doinkWow == 1){
        doinker.retract();
    } else {
        doinker.extend();
    }
    delay(250);
}
ASSET(skillspath_txt);
ASSET(fullfieldtest_txt)
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
    } else {
        pros::lcd::print(1, "No autonomous selected.");
    }
}

void RedLeft()
{
    chassis.setPose(0, 0, 0);
    while(!isRingDetected())
    {
        colorSensor.set_led_pwm(25);
        intake.move(50);
        delay(50);
    }
    intake.move(0);
    chassis.moveToPoint(0, -8.5, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampMogo();
    delay(400);
    intake.move(100);
    delay(300);
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    secondStage.move(127);
    intake.move(127);
    chassis.setPose(0, 0, 90);
    chassis.moveToPoint(18, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 13, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(140, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 19, 1000, {.maxSpeed = 50});
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(0, 15, 270, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 23, 1000);
}

void RedRight()
{
    chassis.setPose(0, 0, 0);
    while(!isRingDetected())
    {
        colorSensor.set_led_pwm(25);
        intake.move(50);
        delay(50);
    }
    intake.move(0);
    chassis.moveToPoint(0, -10, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampMogo();
    delay(300);
    intake.move(127);
    delay(300);
    intake.move(0);
    delay(400);
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    intake.move(127);
    secondStage.move(127);
    delay(300);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 15, 1000);
    chassis.waitUntilDone();
    delay(1500);
    intake.move(0);
    secondStage.move(0);
    clampMogo();
}

void BlueLeft()
{
    chassis.setPose(0, 0, 0);
    while(!isRingDetected())
    {
        colorSensor.set_led_pwm(25);
        intake.move(50);
        delay(50);
    }
    intake.move(0);
    chassis.moveToPoint(0, -7.5, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampMogo();
    delay(300);
    intake.move(127);
    delay(300);
    intake.move(0);
    delay(400);
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    intake.move(127);
    secondStage.move(127);
    delay(300);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 15, 1000);
    chassis.waitUntilDone();
}

void BlueRight()
{
        chassis.setPose(0, 0, 0);
    while(!isRingDetected())
    {
        colorSensor.set_led_pwm(25);
        intake.move(50);
        delay(50);
    }
    intake.move(0);
    chassis.moveToPoint(0, -8.5, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampMogo();
    delay(400);
    intake.move(100);
    delay(300);
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();
    secondStage.move(127);
    intake.move(127);
    chassis.setPose(0, 0, 90);
    chassis.moveToPoint(18, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-180, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 13, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-140, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 19, 1000, {.maxSpeed = 50});
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(-270, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(0, 15, -270, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 23, 1000);
}

#pragma endregion

int toWattage(int power)
{
	return power / 100 * 127;
}

void opcontrol() {
    const char* autonNames[] = {"None", "Red Left", "Red Right", "Blue Left", "Blue Right"};
    const char* colorNames[] = {"None", "Blue", "Red"};

    while (true) {
        // Auton and color selector controls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            selectedAuton = (selectedAuton + 1) % 5;  // Adjust range if adding more autons
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            selectedAuton = (selectedAuton == 0) ? 4 : selectedAuton - 1;  // Adjust range if adding more autons
        }

        colorSensor.set_led_pwm(25);
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
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) clampMogo();

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) doink();

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
