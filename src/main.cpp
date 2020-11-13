#include "main.h"

bool leftAuton = true;

Controller controller;
ControllerButton intake_fwd(ControllerDigital::L1);
ControllerButton intake_rev(ControllerDigital::L2);
ADIButton ball_detect('A');

auto drive = ChassisControllerBuilder().withMotors(1, -2, -10, 9).withDimensions(AbstractMotor::gearset::green, {{4_in, 14_in}, imev5GreenTPR}).build();
auto profiling = AsyncMotionProfileControllerBuilder().withLimits({1.0, 2.0, 10.0}).withOutput(drive).buildMotionProfileController();

MotorGroup intake({11, -12});

void toggle_auton() {
	leftAuton = !leftAuton;
	if (leftAuton) {
		pros::lcd::set_text(2, "auton: LEFT  (change with btn0)");
	} else {
		pros::lcd::set_text(2, "auton: RIGHT (change with btn0)");
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "344R - Change Up - York 11/14/2020");
	pros::lcd::print(1, "Compiled at %s %s", __DATE__, __TIME__);

	pros::lcd::set_text(2, "auton: LEFT  (change with btn0)");
	pros::lcd::register_btn0_cb(toggle_auton);

	pros::lcd::set_text(4, "Auton starts 24\" horiz. from ctr home goal");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	drive->setTurnsMirrored(!leftAuton);

	// Start: 24" horizontally from the center home goal

	// Do a U-turn to face the goal head-on
	drive->moveDistance(24_in);
	drive->turnAngle(90_deg);
	drive->moveDistance(24_in);
	drive->turnAngle(90_deg);

	// Start the intake, go in and extract the ball, then back up
	intake.moveVoltage(12000);
	drive->moveDistance(24_in);
	drive->moveDistance(-24_in);
	intake.moveVoltage(0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

int intake_state = 0;

void opcontrol() {

	while (true) {
		drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                              controller.getAnalog(ControllerAnalog::rightX));

		if (ball_detect.changedToPressed()) {
			if (intake_state > 0) {
				intake_state = 0;
			}
		} else if (intake_fwd.changedToPressed()) {
			if (intake_state > 0) {
				intake_state = 0;
			} else {
				intake_state = 12000;
			}
		} else if (intake_rev.changedToPressed()) {
			if (intake_state < 0) {
				intake_state = 0;
			} else {
				intake_state = -12000;
			}
		}
		intake.moveVoltage(intake_state);
		pros::delay(10);
	}
}
