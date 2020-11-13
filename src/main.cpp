#include "main.h"

bool leftAuton = true;

enum class AutonRoutine {
	OnePoint,
	TestDrive,
	TestTurn
};

AutonRoutine auton_routine = AutonRoutine::OnePoint;

Controller controller;
ControllerButton intake_fwd(ControllerDigital::L1);
ControllerButton intake_rev(ControllerDigital::L2);
ADIButton ball_detect('A');

ControllerButton auton_one(ControllerDigital::A);
ControllerButton auton_two(ControllerDigital::Y);

auto drive = ChassisControllerBuilder().withMotors(1, -2, -10, 9).withDimensions(AbstractMotor::gearset::green, {{4.1_in, /*19.5_in*/ 15.5_in}, imev5GreenTPR}).build();
auto profiling = AsyncMotionProfileControllerBuilder().withLimits({0.5, 1.0, 5.0}).withOutput(drive).buildMotionProfileController();

MotorGroup intake({11, -12});

void draw_screen() {
	// pros::lcd::clear();

	pros::lcd::set_text(0, "344R - Change Up - York 11/14/2020");
	pros::lcd::print(1, "Compiled at %s %s", __DATE__, __TIME__);

	if (leftAuton) {
		pros::lcd::set_text(2, "auton: LEFT  (change with btn0)");
	} else {
		pros::lcd::set_text(2, "auton: RIGHT (change with btn0)");
	}

	switch (auton_routine) {
		case AutonRoutine::OnePoint:
			pros::lcd::set_text(4, "Auton starts 24\" horiz. from ctr home goal");
			break;
		case AutonRoutine::TestDrive:
		  pros::lcd::set_text(4, "TEST AUTON - Drive forward 17.5\"");
			break;
		case AutonRoutine::TestTurn:
		  pros::lcd::set_text(4, "TEST AUTON - Turn 90 degrees");
			break;
	}

	pros::lcd::set_text(5, "(change with btn1)");
}

void toggle_auton() {
	leftAuton = !leftAuton;
	draw_screen();
}

void toggle_test() {
	auton_routine = (AutonRoutine)(((int)auton_routine) + 1);

	if ((int)auton_routine == 3) {
		auton_routine = AutonRoutine::OnePoint;
	}
	draw_screen();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
double default_vel;

void initialize() {
	pros::lcd::initialize();

	pros::lcd::register_btn0_cb(toggle_auton);
	pros::lcd::register_btn1_cb(toggle_test);

	draw_screen();

	default_vel = drive->getMaxVelocity();
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

void one_point_auto() {
	// Start: 24" horizontally from the center home goal

	// Do a U-turn to face the goal head-on
	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{36_in, 18_in, 0_deg}
	}, true);

	// Start the intake, go in and extract the ball, then back up
	intake.moveVoltage(12000);

	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{36_in, 0_in, 0_deg}
	});

	while (!ball_detect.isPressed());
	pros::c::delay(500);

	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{36_in, 0_in, 0_deg}
	}, true);

	// Turn to the side and spit out the ball

	drive->setMaxVelocity(100);
	drive->turnAngle(-90_deg);
	intake.moveVoltage(-12000);
	pros::c::delay(5000);
	intake.moveVoltage(0);

	// Move towards the corner goal

	intake.moveVoltage(0);
	drive->turnAngle(-180_deg);
	drive->setMaxVelocity(default_vel);

	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{48_in, 0_in, 45_deg}
	});

	// Start the intake, go in and extract the ball, then back up
	intake.moveVoltage(12000);

	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{36_in, 0_in, 0_deg}
	});

	while (!ball_detect.isPressed());
	pros::c::delay(500);

	profiling->moveTo({
		{0_in, 0_in, 0_deg},
		{36_in, 0_in, 0_deg}
	}, true);

	pros::c::delay(5000);
	intake.moveVoltage(0);
}

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

	switch (auton_routine) {
		case AutonRoutine::OnePoint:
			one_point_auto();
			break;
		case AutonRoutine::TestDrive:
			profiling->moveTo({
				{0_in, 0_in, 0_deg},
				{17.5_in, 0_in, 0_deg}
			});
			break;
		case AutonRoutine::TestTurn:
			/*
			profiling->moveTo({
				{0_in, 0_in, 0_deg},
				{17.5_in, 17.5_in, 90_deg}
			});
			*/
			drive->setMaxVelocity(100);
			drive->turnAngle(90_deg);
			drive->setMaxVelocity(default_vel);

			pros::c::delay(1000);

			profiling->moveTo({
				{0_in, 0_in, 0_deg},
				{17.5_in, 17.5_in, 90_deg}
			});
			break;
	}
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

		if (auton_one.isPressed() && auton_two.isPressed()) {
			autonomous();
		}

		intake.moveVoltage(intake_state);
		pros::delay(10);
	}
}
