#include "main.h"
#include "globals.h"
#include "pros/rtos.h"
#include <iostream>
#include <fstream>
#include <filesystem>

bool enableArm = false;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	inertial.reset();
	
	pros::Task odom (odometry);
}

void odometry(void) {
	std::cout << "hi";
	pros::delay(300);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

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
	leftMiddle();
}
/**
 * 
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
void opcontrol() {
	ring.move_velocity(-600);

	while(true) {
			pros::delay(10);
			
		//gas gas gas
			int a4 = 1000*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			int a3 = 1000*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			FL.move_voltage(a4 + a3);
			FR.move_voltage(a4 - a3);
			MR.move_voltage(a4 - a3);
			ML.move_voltage(a4 + a3);
			BL.move_voltage(a4 + a3);
			BR.move_voltage(a4 - a3);

		//rings
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				ring.move_voltage(12000);
			}
			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				ring.move_voltage(-12000);

			}
			else if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) || (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))){
				ring.move_voltage(0);
			}

		//arm
			if((!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) && (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))) {
				arm.move_velocity(0);
				arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}
			
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				arm.move_voltage(-12000);
				arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				arm.move_voltage(12000);
				arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
		
		//front
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
				pistonF.set_value(false);
			}
			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
				pistonF.set_value(true);	
			}
			
		//back
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				pistonB1.set_value(false);
				pistonB2.set_value(false);
			}
			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				pistonB1.set_value(true);
				pistonB2.set_value(true);
			}
		//testing
			// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			// 	autonomous();
			// }
		}
}

void moveArm(double time, double speed) {
	//speed pos = down
	//speed neg = up
	time*=1000;
	arm.move_velocity(speed);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::delay(time);
	arm.move_velocity(0);
}

void forward(double time, double speed) {
	FR.move_velocity(-speed);
	FL.move_velocity(speed);
	MR.move_velocity(-speed);
	ML.move_velocity(speed);
	BR.move_velocity(-speed);
	BL.move_velocity(speed);
	time*=1000;
	pros::delay(time);
	FL.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
    FR.move_velocity(0);
    MR.move_velocity(0);
    BR.move_velocity(0);
}
void backward(double time, double speed) {
	FR.move_velocity(speed);
	FL.move_velocity(-speed);
	MR.move_velocity(speed);
	ML.move_velocity(-speed);
	BR.move_velocity(speed);
	BL.move_velocity(-speed);
	time*=1000;
	pros::delay(time);
	FL.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
    FR.move_velocity(0);
    MR.move_velocity(0);
    BR.move_velocity(0);
}
void left(double time, double speed) {
	FR.move_velocity(-speed);
	FL.move_velocity(-speed);
	MR.move_velocity(-speed);
	ML.move_velocity(-speed);
	BR.move_velocity(-speed);
	BL.move_velocity(-speed);
	time*=1000;
	pros::delay(time);
	FL.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
    FR.move_velocity(0);
    MR.move_velocity(0);
    BR.move_velocity(0);
}
void right(double time, double speed) {
	FR.move_velocity(speed);
	FL.move_velocity(speed);
	MR.move_velocity(speed);
	ML.move_velocity(speed);
	BR.move_velocity(speed);
	BL.move_velocity(speed);
	time*=1000;
	pros::delay(time);
	FL.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
    FR.move_velocity(0);
    MR.move_velocity(0);
    BR.move_velocity(0);
}

void arc(int angle, double time, double speed) {
	inertial.tare_rotation();

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;

	double kP = 4.8;
	double kI = 18.1;
	double kD = 0.02;
	double target = inertial.get_rotation() + angle;

	while(fabs(target - inertial.get_rotation()) > 0.1) {
		error = target - inertial.get_rotation();
		prevError = error;
		integral = integral + error*0.02;
		derivative = error*0.02 - prevError;

		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
            integral = 0;
        }

		power = (error * kP) + (integral * kI) + (derivative*kD);
		power*=0.9;


		FR.move(power);
		FL.move(power);
		MR.move(power);
		ML.move(power);
		BR.move(power);
		BL.move(power);
		pros::delay(20);

		forward(time, speed);
	}

	FL.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
    FR.move_velocity(0);
    MR.move_velocity(0);
    BR.move_velocity(0);
}
void turn(int angle) {
	inertial.tare_rotation();

	double error = 0;
	double prevError = 0;
	// double integral = 0;
	double derivative = 0;


	double power = 0;


	double kP = 7.8;
	// double kI = 0;
	double kD = 1.45;
	// double kD = 0.15;

	//TARGET = where we want to turn to
	double target = inertial.get_rotation() + angle;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		
		//Equation for ERROR = Target - CurrentState
		error = target - inertial.get_rotation();

		//In a while loop, it will keep running until the conditions are met
		//Assign previous error the value of error = X-1 gets what X was before
		prevError = error;


		// integral = integral + error*0.015;

		//Rate of change of error, find the difference between error and the previous error
		derivative = error - prevError;


		// if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
        //     integral = 0;
        // }

		//Error * kP = PROPORTIONAL TERM
		//Derivative * kD = DERIVATIVE TERM
		//Adding the power, getting PD
		power = (error * kP) + (derivative*kD);
		power*=0.9;

		//MOTORS TURN TO THE RIGHT
		FR.move_velocity(power);
		FL.move_velocity(power);
		MR.move_velocity(power);
		ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);

		//wait to let it rest
		pros::delay(15);
	}

	//stop all motors
	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}
void turn2(int angle) {
	inertial.tare_rotation();

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 7.9;
	double kI = 0;
	double kD = 1.2;
	double target = inertial.get_rotation() + angle;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		error = target - inertial.get_rotation();
		prevError = error;
		integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
            integral = 0;
        }
		power = (error * kP) + (integral * kI) + (derivative*kD);
		power*=0.9;

		FR.move_velocity(power);
		FL.move_velocity(power);
		MR.move_velocity(power);
		ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(15);
	}
	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}
void turn1(int angle) {
	inertial.tare_rotation();

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 7.8;
	double kI = 0;
	double kD = 0.8;
	double target = inertial.get_rotation() + angle;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		error = target - inertial.get_rotation();
		prevError = error;
		integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
            integral = 0;
        }
		power = (error * kP) + (integral * kI) + (derivative*kD);
		power*=0.9;

		FR.move_velocity(power);
		FL.move_velocity(power);
		MR.move_velocity(power);
		ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(15);
	}
	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}
void autopark() {
	int balance = 0;
	inertial.tare_pitch();

	//forward for 1sec
	MR.move_velocity(-600);
	FR.move_velocity(-600);
    BR.move_velocity(-600);
    ML.move_velocity(600);
    BL.move_velocity(600);
	FL.move_velocity(600);
	pros::delay(1000);
	MR.move_velocity(0);
	FR.move_velocity(0);
    BR.move_velocity(0);
    ML.move_velocity(0);
    BL.move_velocity(0);
	FL.move_velocity(0);

	while((inertial.get_pitch() > 0) && (inertial.get_pitch() < 50)) {
		MR.move_velocity(-200);
		FR.move_velocity(-200);
		BR.move_velocity(-200);
		ML.move_velocity(200);
		BL.move_velocity(200);
		FL.move_velocity(200);
	}

	MR.move_velocity(-600);
	FR.move_velocity(-600);
    BR.move_velocity(-600);
    ML.move_velocity(600);
    BL.move_velocity(600);
	FL.move_velocity(600);
	pros::delay(300);


	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);

}

void programmingSkills() {
	ring.move_velocity(-600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(300,-300);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	inertial.tare_rotation();
	move_encoder(300, 300);
	turn(-90);
	forward(0.5,600);
	inertial.tare_rotation();
	move_encoder(1800, -300);
	pros::delay(100);
	turn1(-155);
	move_encoder(2100, 400);
	pistonF.set_value(true);

	turn2(20);
	moveArm(2, -100);
	inertial.tare_rotation();
	move_encoder(3000, 400);
	turn2(0-inertial.get_rotation());
	turn2(-55);
	move_encoder(300,500);
	inertial.tare_rotation();
	moveArm(0.8, 100);
	pistonF.set_value(false);
	moveArm(0.8, -100);
	inertial.tare_rotation();

	move_encoder(150,-200);
	moveArm(2, 100);

	turn1(90);
	inertial.tare_rotation();
	move_encoder(100, 200);
	turn1(0-inertial.get_rotation());
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(100, 200);
	turn(180);
	move_encoder(500, 200);
	pistonF.set_value(true);
	moveArm(2, -100);
	turn1(90);
	move_encoder(300, 400);
	moveArm(0.7, 100);
	pistonF.set_value(false);
	moveArm(0.7, -100);
	inertial.tare_rotation();
	move_encoder(200, -300);
	turn(0-inertial.get_rotation());
	turn(180);	
	move_encoder(1000, 300);
	moveArm(2, 100);
	move_encoder(600, 400);
	pistonF.set_value(true);
	moveArm(0.3, -100);
	turn2(180);
	moveArm(1.3, -100);
	inertial.tare_rotation();
	move_encoder(1800, 400);
	turn2(0-inertial.get_rotation());


	// forward(2, 600);
	// pistonF.set_value(false);
	// inertial.tare_rotation();
	// move_encoder(300, 500);
	// turn(0-inertial.get_rotation());
	// turn(-135);
	// backward(0.5, 600);
	// turn1(180);

	// move_encoder(1500, 400);
	// moveArm(2, -100);
	// turn1(30);

	// move_encoder(300, 400);
	// moveArm(0.8, 100);
	// pistonF.set_value(false);
	// moveArm(0.8, -100);

	// inertial.tare_rotation();
	// move_encoder(900, -400);
	// turn(0-inertial.get_rotation());
	// turn(90);
	// move_encoder(1200, 400);
	// pistonF.set_value(true);
	// moveArm(2, -100);
	// turn1(140);

	// move_encoder(2700, 400);


	// turn(0-inertial.get_rotation());
	// turn1(180);
	// inertial.tare_rotation();
	// move_encoder(900, 400);
	// turn(0-inertial.get_rotation());
	// pistonF.set_value(true);
	// moveArm(0.4, -100);
	// turn2(180);
	// moveArm(1, -100);
	// move_encoder(900, 400);
	// moveArm(0.8, 100);
	// pistonF.set_value(false);
	// moveArm(0.8, -100);



	// moveArm(0.4, -100);
	// move_encoder(1400, 400);
	// moveArm(0.4, 100);
	// move_encoder(400, 400);
	// pistonF.set_value(true);
	// turn2(140);
	// moveArm(1.8, -100);

	// move_encoder(3300, 400);
	// moveArm(0.5, 100);
	// pistonF.set_value(false);
	// moveArm(0.5, -100);
	// move_encoder(3300, -400);
	// moveArm(1.8, 100);
	// turn1(130);
	// move_encoder(600, 400);
	// pistonF.set_value(true);
	// moveArm(1.8, -100);
	// turn2(-130);
	
	// pistonB1.set_value(true);
	// pistonB2.set_value(true);
	// move_encoder(400,400);
	// turn(180);
	// move_encoder(500,400);
	// pistonF.set_value(true);
	// inertial.tare_rotation();
	// move_encoder(600,-400);
	// turn(0-inertial.get_rotation());
	// moveArm(2, -100);
	// inertial.tare_rotation();
	// double error = 0;
	// double prevError = 0;
	// double integral = 0;
	// double derivative = 0;
	// double power = 0;
	// double kP = 7.8;
	// double kI = 0;
	// double kD = 0.15;
	// double target = inertial.get_rotation() + (-90);
	
	// while(fabs(target - inertial.get_rotation()) > 0.5) {
	// 	error = target - inertial.get_rotation();
	// 	prevError = error;
	// 	integral = integral + error*0.015;
	// 	derivative = error*0.015 - prevError;
	// 	if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
    //         integral = 0;
    //     }
	// 	power = (error * kP) + (integral * kI) + (derivative*kD);
	// 	power*=0.7;

	// 	FR.move_velocity(power);
	// 	FL.move_velocity(power);
	// 	MR.move_velocity(power);
	// 	ML.move_velocity(power);
	// 	BR.move_velocity(power);
	// 	BL.move_velocity(power);
	// 	pros::delay(15);
	// }
	// FL.move_voltage(0);
    // ML.move_voltage(0);
    // BL.move_voltage(0);
    // FR.move_voltage(0);
    // MR.move_voltage(0);
    // BR.move_voltage(0);
	// inertial.tare_rotation();
	// move_encoder(300, 400);
	// moveArm(0.4, 100);
	// pistonF.set_value(false);
	// moveArm(0.4, -100);
	// move_encoder(3000, -600);
	// turn(-45-inertial.get_rotation());
	// inertial.tare_rotation();
	// move_encoder(1200, -500);
	// turn(-45-inertial.get_rotation());
	// move_encoder(1500,-500);

	// pistonB1.set_value(false);
	// pistonB2.set_value(false);
}
void test() {

	//https://www.youtube.com/watch?v=gC47LDYibTc
	//section 1

	//back clamp
	inertial.tare_rotation();
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	backward(0.3, 450);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	inertial.tare_rotation();
	forward(0.4, 500);
	turn(0-inertial.get_rotation());
	turn(-90);
	inertial.tare_rotation();
	backward(0.57, 600);
	turn(0-inertial.get_rotation());
	turn(-90);
	pistonF.set_value(true);
	

	//forward and then clamp
	inertial.tare_rotation();
	move(30, 600);
	turn(0-inertial.get_rotation());
	pistonF.set_value(false);
	

	//turn on rings
	//back up and raise arm
	//turn -135 deg
	//drive forward fast then slow down (while lifting up the arm)
	//turn 45 deg
	//place tower

	//section 2
	//turn 180 degrees
	//lower arm
	//go forward and clamp
	//turn 180 degrees
	//go forward and raise
	//place tower

	//section 3
	//lift a little bit and then back up slightly
	//turn(-135 degrees) [to the yellow tower]
	//lower arm
	//forward and clamp
	//turn(160 degrees)
	//forward
	//place tower

	//section 4
	//bring arm down
	//turn til ur in parallel to the other tower behind
	//drop back tower
	//180 deg
	//get tower and stack

	//section 5
	//get blue in back
	//turn small degrees
	//forward and clamp last yellow
	//turn and raise arm
	//forward
	//place tower

	//section 6
	//get the last blue
	//go to balance tower
	//and then park

}


void move_encoder(double ticks, double speed) {
	reset_encoder();
	
	FR.move_velocity(-speed);
	FL.move_velocity(speed);
	MR.move_velocity(-speed);
	ML.move_velocity(speed);
	BR.move_velocity(-speed);
	BL.move_velocity(speed);

	while(fabs(average_encoders())<ticks){
		pros::delay(2);
	}

	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}


void speed(double ticks, double speed) {
	reset_encoder();
	
	FR.move_voltage(-speed);
	FL.move_voltage(speed);
	MR.move_voltage(-speed);
	ML.move_voltage(speed);
	BR.move_voltage(-speed);
	BL.move_voltage(speed);

	while(fabs(average_encoders())<ticks){
			pros::delay(2);
	}

	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}
void reset_encoder() {
	FL.tare_position();
	ML.tare_position();
	BL.tare_position();
	FR.tare_position();
	MR.tare_position();
	BR.tare_position();
}
double average_encoders() {
	return (fabs(BL.get_position()) +
		fabs(ML.get_position()) +
		fabs(FL.get_position()) +
		fabs(BR.get_position()) +
		fabs(MR.get_position()) +
		fabs(FR.get_position()) ) / 6;
}

//regular
void rightMiddle() {
	//maybe wp
	inertial.tare_rotation();
	move_encoder(3900, 500);
	pistonF.set_value(true);
	move_encoder(2400, -300);
	turn(-65);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	backward(1,600);
	ring.move_velocity(-600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	moveArm(0.5, -100);
	turn(90);
	move_encoder(2600, 400);
	backward(2, 600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	left(0.3, 600);
	forward(0.3, 600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
}
void rightTriple() {
	inertial.tare_rotation();
	move_encoder(2700, 600);
	pistonF.set_value(true);
	pros::delay(50);
	moveArm(0.2, -100);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(1700, -400);
	turn1(-90);
	inertial.tare_rotation();

	popHood();
	
	backward(0.5, 500);	
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	// turn2(0-inertial.get_rotation());
	ring.move_velocity(-600);
	inertial.tare_rotation();
	move_encoder(2000, 400);
	moveArm(0.2, 100);
	
	pistonF.set_value(false);
	move_encoder(300, -300);
	turn1(45);
	move_encoder(3000, 300);
	pistonF.set_value(true);
	backward(1.7, 600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	pros::delay(100);
	turn1(-20);
	forward(0.2, 600);
}
void rightComplexWP() {
	inertial.tare_rotation();
	move_encoder(2400, 600);
	pros::delay(50);
	pistonF.set_value(true);
	moveArm(0.2, -100);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(1700, -300);
	turn1(-90);
	ring.move_velocity(-600);
	pros::delay(200);
	ring.brake();
	inertial.tare_rotation();
	backward(0.7, 500);	
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	inertial.tare_rotation();
	moveArm(0.7, -100);
	turn2(90);
	ring.move_velocity(-600);
	move_encoder(2800, 300);
	pros::delay(500);
	backward(1.5, 600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	left(0.2, 600);
	forward(0.3, 600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
}
void leftComplexWP() {
	inertial.tare_rotation();
	move_encoder(3000, 600);
	pistonF.set_value(true);
	moveArm(0.2, -100);
	backward(1, 600);
	left(0.06, 300);
	backward(1, 600);
	turn(-90);
	moveArm(1.4, -100);
	ring.move_velocity(-600);
	pros::delay(300);
	ring.brake();
	pistonB1.set_value(true);
	pistonB2.set_value(true);

	move_encoder(1100, -300);

	moveArm(0.5, -100);
	
	pistonB1.set_value(false);
	pistonB2.set_value(false);

	ring.move_velocity(-600);

	pros::delay(900);
	forward(2, 200);
	backward(0.5, 600);
	pros::delay(900);
	forward(1.5, 200);
	backward(0.8, 300);
	// forward(3, 100);
	// backward(1, 300);
	moveArm(1.4, 100);

	pistonB1.set_value(true);
	pistonB2.set_value(true);

	forward(0.4, 600);
	right(0.1, 600);
}

//kickstand
void KICKrightComplexWP() {
	// std::string stem = "/usd/debug.txt";
	// std::ofstream debugLog(stem);
	// std::uint32_t now = pros::millis();

	inertial.tare_rotation();
	move_encoder(2300, 600);
	pistonF.set_value(true);

	// std::uint32_t then = pros::millis();
	// double test = then-now;
	// debugLog << test << ",";

	moveArm(0.2, -100);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(1600, -300);
	turn1(-90);

	inertial.tare_rotation();
	backward(0.6, 500);	
				// ring.move_velocity(-500);
				// pros::delay(200);
				// ring.brake();
	dropOne();

	pistonB1.set_value(false);
	pistonB2.set_value(false);
	inertial.tare_rotation();
	moveArm(0.7, -100);
	forward(0.1, 400);
	turn2(90);
	ring.move_velocity(-600);
	move_encoder(2800, 300);

	move_encoder(3600, 600);

	pros::delay(500);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	left(0.3, 600);
	forward(0.3, 600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);

	// debugLog.close();
}
void KICKrightMiddle() {
	//maybe wp
	inertial.tare_rotation();
	move_encoder(3500, 500);
	pistonF.set_value(true);
	move_encoder(2200, -300);

					// ring.move_velocity(-500);
					// pros::delay(200);
					// ring.brake();
	popHood();

	turn(-65);
	
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	backward(1,600);
	
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	ring.move_velocity(-600);
	moveArm(0.5, -100);
	turn(90);
	move_encoder(2600, 400);
	backward(2, 600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	left(0.3, 600);
	forward(0.3, 600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
}
void KICKrightTriple() {
	inertial.tare_rotation();
	move_encoder(2175, 600);
	pistonF.set_value(true);
	moveArm(0.2, -100);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	move_encoder(1700, -300);
	turn1(-90);
	inertial.tare_rotation();

					// ring.move_velocity(-600);
					// pros::delay(200);
					// ring.brake();
	popHood();

	backward(0.7, 500);	
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	// turn2(0-inertial.get_rotation());
	ring.move_velocity(-600);
	inertial.tare_rotation();
	move_encoder(2000, 400);
	moveArm(0.2, 100);
	
	pistonF.set_value(false);
	move_encoder(300, -300);
	turn1(47);
	move_encoder(3000, 300);
	pistonF.set_value(true);
	backward(1.6, 600);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	pros::delay(100);
	turn1(-20);
	forward(0.2, 600);
}
void KICKleftComplexWP() {
	inertial.tare_rotation();
	move_encoder(2500, 600);
	pistonF.set_value(true);
	moveArm(0.3, -100);
	backward(2, 600);
	// left(0.03, 600);
	// backward(1, 600);
	moveArm(1.2, -100);
	turn1(-90);
	
	right(0.1, 400);
	backward(0.4, 400);
	
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	left(0.1,400);

	move_encoder(1100, -300);

					// ring.move_velocity(-600);
					// pros::delay(200);
					// ring.brake();
	dropOne();

	moveArm(0.5, -100);
	
	pistonB1.set_value(false);
	pistonB2.set_value(false);

	ring.move_velocity(-600);

	pros::delay(900);
	forward(2.5, 250);
	backward(0.5, 600);
	// pros::delay(900);
	// forward(1.5, 200);
	// backward(0.8, 300);
	// forward(3, 100);
	// backward(1, 300);
	moveArm(1.4, 100);
	right(0.2, 600);

	pistonB1.set_value(true);
	pistonB2.set_value(true);

	forward(0.4, 600);
}


//simple
void soloWP() {
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	// backward(0.3, 600);
	moveArm(0.2, -100);

	//MAKE THIS ENOUGH FOR ONE RING TO GO
	dropOne();

	pros::delay(500);

	
	move_encoder(600, 300);
	moveArm(0.2, 100);
	turn(-90);
	move_encoder(900, -300);
	turn(90);

	forward(0.8, 500);

	inertial.tare_rotation();
	move_encoder(6900, -600);
	backward(0.2, 600);
	pistonB1.set_value(false);
	pistonB2.set_value(false);	

	ring.move_velocity(-600);

	moveArm(0.2, -100);
	move_encoder(2000, 600);

	pros::delay(400);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
}
void rightNeutral() {
	inertial.tare_rotation();
	move_encoder(2400, 600);
	pros::delay(50);
	pistonF.set_value(true);
	moveArm(0.2, -100);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	backward(3, 600);
	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
	pros::delay(1000);
}
void leftEasyWP() {
	//just wp
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	ring.move_velocity(-600);
	pros::delay(200);
	ring.brake();
	backward(0.5, 500);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	ring.move_velocity(-600);
	pros::delay(5000);
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	forward(0.4, 600);
}

void leftMiddle() {
	pistonB1.set_value(true);
	pistonB2.set_value(true);
	dropOne();
	pros::delay(30);
	backward(0.3, 500);
	pistonB1.set_value(false);
	pistonB2.set_value(false);
	ring.move_velocity(-600);

	pros::delay(1000);
	ring.brake();

	pistonB1.set_value(true);
	pistonB2.set_value(true);

	forward(0.7, 600);
	pros::delay(30);
	move_encoder(500, -300);

	turn(90);

	pistonB1.set_value(false);
	pistonB2.set_value(false);
	
	backward(0.4, 600);
	
	move_encoder(900, 300);

	moveArm(0.4, -100);
	turn(45);
	ring.move_velocity(-600);
	move_encoder(2200, 300);

	moveArm(0.6, 100);
	move_encoder(800, 400);
	pros::delay(30);
	pistonF.set_value(true);

	backward(2, 600);
}


void popHood() {
	ring.move_velocity(-400);
	pros::delay(100);
	ring.brake();
}

void dropOne() {
	ring.move_velocity(-600);
	pros::delay(200);
	ring.brake();
}