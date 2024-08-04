#include "main.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <iostream>

using namespace std;
using namespace pros;
using namespace lemlib;
using namespace rd;


// Motor configuration
#pragma region Motor Configuration
MotorGroup left_motors({-1,-2, -3}, MotorGearset::blue);
MotorGroup right_motors({18, 19, 20}, MotorGearset::blue);
#pragma endregion

// Drivetrain settings
#pragma region Drivetrain Settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              15, // 10 inch track width
                              Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
#pragma endregion

// Sensors
#pragma region Sensors
Imu imu(7);
Rotation horizontal_sensor(8);
Rotation vertical_sensor(9);

// horizontal tracking wheel
TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, Omniwheel::NEW_275, -1);
// vertical tracking wheel
TrackingWheel vertical_tracking_wheel(&vertical_sensor, Omniwheel::NEW_275, -7);

OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
#pragma endregion

// PID controllers
#pragma region PID Controllers
// lateral PID controller
ControllerSettings lateral_controller(11, // proportional gain (kP)
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
ControllerSettings angular_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
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


// Initialization
#pragma region Initialization
void initialize() {
    lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            lcd::print(0, "X: %f", chassis.getPose().x); // x
            lcd::print(1, "Y: %f", chassis.getPose().y); // y
            lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            lcd::print(11, "Horizontal Wheel Position: %f", horizontal_sensor.get_position());
            lcd::print(12, "Vertical Wheel Position: %f", vertical_sensor.get_position());
            // delay to save resources
            delay(20);
        }
    });
}
#pragma endregion

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
ASSET(skillspath_txt);

void logPosition(const std::string& positionName, const std::string& command, const Pose& pose) {
    std::cout << positionName << " - Command: " << command << " - X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
}

void autonomous() {
    // set chassis pose
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(45, 5000);
    chassis.turnToHeading(90, 5000);
    chassis.moveToPoint(10, 10, 5000);
    chassis.moveToPose(0, 0, 0, 5000);
}   
#pragma endregion

// Operator Control
#pragma region Operator Control
Controller controller(E_CONTROLLER_MASTER);
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        delay(25);
    }
}
#pragma endregion
