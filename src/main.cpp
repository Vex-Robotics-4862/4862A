#include "main.h"
#include "okapi/api.hpp"
#include "math.h"
#define LB_MTR 4
#define LF_MTR 5
#define RB_MTR 2
#define RF_MTR 3
#define LIFT_1 6
#define LIFT_2 7 //yet to be connected
#define L_INTAKE 8
#define R_INTAKE 9

enum autoType { competition1, competition2, skills1, skills2, test_one_tile};
int side = -1; // -1 is red; 1 is blue
std::string sides[] = {"red", "???", "blue"};
autoType whatAuto = skills1;
int sideDelay = 0;
int minusButtonDelay = 0;
int plusButtonDelay = 0;
int temp[] = {0,0,0,0,0};
const bool checkTemp = true;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Version 0.3!");
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
void competition_initialize() {
while (true) {
	if (sideDelay > 0)
		sideDelay--;
	if (minusButtonDelay > 0)
		minusButtonDelay--;
	if (plusButtonDelay > 0)
		plusButtonDelay--;
	if (pros::lcd::read_buttons() > 0) {
		switch (pros::lcd::read_buttons()) {
			case 1:
				if (sideDelay < 1) {
					side = -side;
				}
				sideDelay = 10;
			break;
			case 2:
				whatAuto = skills1;
			break;
			case 4:
				whatAuto = skills2;
			break;
			case 6:
				whatAuto = competition1;
			break;
			default:
			break;
			pros::lcd::set_text(1, "Auto: " + std::to_string(whatAuto) + " on " + sides[side+1]);
		}
	}
	pros::delay(40);
	}

}


void autonomous() {
	pros::Motor lb_mtr (LB_MTR, pros::E_MOTOR_GEARSET_06, true);
	pros::Motor lf_mtr (LF_MTR, pros::E_MOTOR_GEARSET_06, true);
	pros::Motor rb_mtr (RB_MTR, pros::E_MOTOR_GEARSET_06);
	pros::Motor rf_mtr (RF_MTR, pros::E_MOTOR_GEARSET_06);
	okapi::MotorGroup left_mtr({LB_MTR, LF_MTR});
	okapi::MotorGroup right_mtr({RB_MTR, RF_MTR});


}

void opcontrol() {
	pros::Controller control(pros::E_CONTROLLER_MASTER);
	pros::Motor lb_mtr (LB_MTR, pros::E_MOTOR_GEARSET_06, true);
	pros::Motor lf_mtr (LF_MTR, pros::E_MOTOR_GEARSET_06, true);
	pros::Motor rb_mtr (RB_MTR, pros::E_MOTOR_GEARSET_06);
	pros::Motor rf_mtr (RF_MTR, pros::E_MOTOR_GEARSET_06);
	pros::Motor allmotors[] = {lb_mtr, lf_mtr, rb_mtr, rf_mtr};
	okapi::MotorGroup left_mtr({LB_MTR, LF_MTR});
	okapi::MotorGroup right_mtr({RB_MTR, RF_MTR});
	int left, right, forw, rot, i;
	int maxTemp, motorMaxTemp;
	enum driveType { tank, right_only };
	driveType drive = right_only;
	while (true) {

		switch (drive) {
			case tank:
				left = control.get_analog(ANALOG_LEFT_Y);
				right = control.get_analog(ANALOG_RIGHT_Y);
				left_mtr.moveVelocity(left);
				right_mtr.moveVelocity(right);
				break;
			case right_only:
				// mag = hypot(control.get_analog(ANALOG_RIGHT_X), control.get_analog(ANALOG_RIGHT_Y)) * 1.42; //mult by sqrt(2)
				// dir = atan2((double)control.get_analog(ANALOG_RIGHT_Y), (double)control.get_analog(ANALOG_RIGHT_X));
				// pros::lcd::set_text(0, std::to_string(dir));

				forw = control.get_analog(ANALOG_RIGHT_Y) * .5;
				rot = control.get_analog(ANALOG_RIGHT_X) * .3;
				left = (forw + rot);
				right = (forw - rot);
				if (true) {
					lb_mtr = left;
					lf_mtr = left;
					rb_mtr = right;
					rf_mtr = right;
				} else {
					left_mtr.moveVelocity(left);
					right_mtr.moveVelocity(right);
				}



				break;
		}
		if (control.get_digital(DIGITAL_B) && lb_mtr.get_brake_mode() == pros::E_MOTOR_BRAKE_COAST) {
			lb_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rb_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		} else if (lb_mtr.get_brake_mode() != pros::E_MOTOR_BRAKE_COAST) {
			lb_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rb_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		pros::lcd::set_text(1, std::to_string(lb_mtr.get_voltage()/100));
		pros::lcd::set_text(2, std::to_string(rb_mtr.get_voltage()/100));
		if (checkTemp) {
			maxTemp = 0;
			for(i = 0; i<4; i++) { //do NOT set i higher than the number of motors
				if (allmotors[i].get_temperature() > maxTemp) {
					maxTemp = allmotors[i].get_temperature();
					motorMaxTemp = i;
				}
			}
			if (maxTemp > 50) {
				pros::lcd::clear();
				pros::lcd::set_text(2, "OVERHEATING");
				pros::lcd::set_text(4, "Disable Bot if Possible!");
			}
			pros::lcd::set_text(6, "Mtr" + std::to_string(motorMaxTemp) + " @ Temp " + std::to_string(maxTemp) + "C");

		}
		pros::delay(20);
	}
}
