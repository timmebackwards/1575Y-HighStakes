#include "main.h"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "robodash/views/selector.hpp"
#include <iostream>

using namespace std;
using namespace pros;
using namespace lemlib;
using namespace rd;

// Motor configuration
#pragma region Motor Configuration
MotorGroup left_motors({-8,-9, 10}, MotorGearset::blue);
MotorGroup right_motors({-6, 4, 5}, MotorGearset::blue);
#pragma endregion

// Drivetrain settings
#pragma region Drivetrain Settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13, // 10 inch track width
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
TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, Omniwheel::NEW_2, 4.5);
// vertical tracking wheel
TrackingWheel vertical_tracking_wheel(&vertical_sensor, Omniwheel::NEW_2, -1.0);

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
pros::Motor intake(2);
pros::Motor secondStage(-21);

// PID controllers
#pragma region PID Controllers
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// 2, 10
// angular PID controller
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
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

// Robodash selector initialization...
Selector selector(
	{
		{"Red Solo", &red_Solo},
		{"Red Finals", &red_Finals},
	}
);

// Robodash console initialization...
Console console;

// Creates the custom pages
rd_view_t *view = rd_view_create("Info");
rd_view_t *auto_override = rd_view_create("Override");

int update_ui()
{
	// Creates the UI elements
	lv_obj_t *override_btn = lv_btn_create(rd_view_obj(auto_override));
	lv_obj_center(override_btn);
	lv_obj_add_event_cb(override_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED){
			rd_view_alert(view, "Running autonomous...");
			selector.run_auton();
		} }, LV_EVENT_ALL, NULL);
	lv_obj_t *override_label = lv_label_create(override_btn);
	lv_label_set_text(override_label, "Run Selected Autonomous");
	lv_obj_center(override_label);

	lv_obj_t *calibrate_btn = lv_btn_create(rd_view_obj(view));	  // Creates Button
	lv_obj_align(calibrate_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20); // Moves Button

	lv_obj_t *calibrate_label = lv_label_create(calibrate_btn); // Creates Label
	lv_label_set_text(calibrate_label, "Calibrate Inertial");	// Sets the text
	lv_obj_center(calibrate_label);								// Centers it on the parent button
	lv_obj_add_event_cb(calibrate_btn, [](lv_event_t *e) {		// Adds event listener
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)			// If the button is clicked
		{
			rd_view_alert(view, "Calibrating IMU..."); // Alert the user.
			imu.reset();							   // Calibrate IMU.
		}
	},
						LV_EVENT_ALL, NULL);

	lv_obj_t *reset_pose_btn = lv_btn_create(rd_view_obj(view));
	lv_obj_align(reset_pose_btn, LV_ALIGN_TOP_RIGHT, -20, 60);

	lv_obj_t *reset_pose_label = lv_label_create(reset_pose_btn);
	lv_label_set_text(reset_pose_label, "Reset Pose");
	lv_obj_center(reset_pose_label);
	lv_obj_add_event_cb(reset_pose_btn, [](lv_event_t *e)
						{
		if (lv_event_get_code(e) == LV_EVENT_CLICKED)
		{
			rd_view_alert(view, "Calibrating and Reseting...");
			chassis.calibrate();
			chassis.resetLocalPosition(); 
		} }, LV_EVENT_ALL, NULL);

	lv_obj_t *imu_heading_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(imu_heading_label, LV_ALIGN_TOP_LEFT, 5, 5);

	lv_obj_t *drive_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(drive_temp_label, LV_ALIGN_TOP_LEFT, 5, 25);

		lv_obj_t *intake_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(intake_temp_label, LV_ALIGN_TOP_LEFT, 5, 40);

		lv_obj_t *lift_temp_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lift_temp_label, LV_ALIGN_TOP_LEFT, 5, 55);

	lv_obj_t *lem_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lem_label, LV_ALIGN_TOP_LEFT, 5, 65);

	lv_obj_t *horizontal_one_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(horizontal_one_label, LV_ALIGN_TOP_LEFT, 5, 175);

	lv_obj_t *lift_encoder_label = lv_label_create(rd_view_obj(view));
	lv_obj_align(lift_encoder_label, LV_ALIGN_TOP_LEFT, 5, 200);
	// Creates the UI elements

	// Update Loop
	while (true)
	{
		// Updates the labels to show the most recent data.
		lv_label_set_text(imu_heading_label, ("IMU Heading: " + to_string(round(imu.get_heading()))).c_str());
		lv_label_set_text(drive_temp_label, ("Average drive temp: " + to_string((left_motors.get_temperature() + right_motors.get_temperature()) / 2)).c_str());
		
			
		lv_label_set_text(lem_label, ("X: " + to_string(chassis.getPose().x) + "\nY: " + to_string(chassis.getPose().y) + "\nTheta: " + to_string(chassis.getPose().theta)).c_str());
		lv_label_set_text(horizontal_one_label, ("Horizontal tracking 1: " + to_string(horizontal_tracking_wheel.getDistanceTraveled())).c_str());
		controller.print(0, 0, "Connection:%d ", controller.is_connected());
		delay(100); // Waits 100ms before updating again to make it readable.
	}
	// Update Loop
}

/**
 * Checks if a device is plugged in at the given port.
 *
 * @param port The port number of the device.
 * @param deviceName The name of the device.
 *
 * @throws None
 */
void check_device_plugged_in(int port, std::string deviceName)
{
	if (!isPluggedIn(port))
	{
		controller.rumble("---");
		controller.print(0, 0, "check console...");
		console.println((deviceName + " not plugged in!").c_str());
	}
}

// Initialization
#pragma region Initialization
void initialize() {
    console.println("Checking device status...");
	check_device_plugged_in(imu.get_port(), "IMU");
	check_device_plugged_in(left_motors.get_port(0), "Left Motor Group 1");
	check_device_plugged_in(left_motors.get_port(1), "Left Motor Group 2");
	check_device_plugged_in(left_motors.get_port(2), "Left Motor Group 3");
	check_device_plugged_in(right_motors.get_port(0), "Right Motor Group 1");
	check_device_plugged_in(right_motors.get_port(1), "Right Motor Group 2");
	check_device_plugged_in(right_motors.get_port(2), "Right Motor Group 3");
	check_device_plugged_in(horizontal_sensor.get_port(), "Horizontal Encoder");
	chassis.calibrate();
	// Start the update_ui task
	Task t(update_ui);
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

void logPosition(const std::string& positionName, const std::string& command, const Pose& pose) {
    std::cout << positionName << " - Command: " << command << " - X: " << pose.x << ", Y: " << pose.y << ", Theta: " << pose.theta << std::endl;
}
ASSET(skillspath_txt);
ASSET(fullfieldtest_txt)
void autonomous() {
    console.println("Running auton...");
	selector.run_auton();
}

void red_Solo()
{

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
    // loop forever
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

		// mogo clamp
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			mogoClamp.toggle();
		}

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake.move(100);
			secondStage.move(70);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(-100);
			secondStage.move(-70);
		} else {
			intake.move(0);
			secondStage.move(toWattage(0));
		}
        // delay to save resources
       
	    delay(25);
    }
}
#pragma endregion
