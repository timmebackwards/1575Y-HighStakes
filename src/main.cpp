#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

// Motor configuration
#pragma region Motor Configuration
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue);
#pragma endregion

// Drivetrain settings
#pragma region Drivetrain Settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              1200, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
#pragma endregion

// Sensors
#pragma region Sensors
// create an imu on port 10
pros::Imu imu(7);

// create a v5 rotation sensor on port 1
pros::Rotation horizontal_sensor(8);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_sensor(9);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_325, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
#pragma endregion

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
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
#pragma endregion

// Initialization
#pragma region Initialization
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
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

void autonomous() {
    // set chassis pose
    chassis.setPose(0, 0, 0);
    // lookahead distance: 15 inches
    // timeout: 2000 ms
    chassis.follow(skillspath_txt, 15, 2000);
}
#pragma endregion

// Operator Control
#pragma region Operator Control
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        pros::delay(25);
    }
}
#pragma endregion
