/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="autonomous MAIN", group="Auto")

public class main_auto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor armL = null; // left arm
    private DcMotor armR = null; // right arm
    private DcMotor witchfingersMotor = null;
    private DistanceSensor sensorDistance = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH_GOBUILDA for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV_GOBUILDA
    // For external drive gearing, set DRIVE_GEAR_REDUCTION_GOBUILDA as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 384.5;    // eg:
    static final double DRIVE_GEAR_REDUCTION_GOBUILDA = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES_GOBUILDA = 3.78;     // https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/
    static final double COUNTS_PER_INCH_GOBUILDA = (COUNTS_PER_MOTOR_REV_GOBUILDA * DRIVE_GEAR_REDUCTION_GOBUILDA) /
            (WHEEL_DIAMETER_INCHES_GOBUILDA * 3.1415);
    static final double COUNTS_PER_MOTOR_REV_WITCHFINGERS = 28;
    static final double DRIVE_GEAR_REDUCTION_WITCHFINGERS = 12;
    static final double SPOOL_DIAMETER_INCHES_WITCHFINGERS = 1.2;
    static final double COUNTS_PER_INCH_WITCHFINGERS = (COUNTS_PER_MOTOR_REV_WITCHFINGERS * DRIVE_GEAR_REDUCTION_WITCHFINGERS) /
            (SPOOL_DIAMETER_INCHES_WITCHFINGERS * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        witchfingersMotor = hardwareMap.get(DcMotor.class, "witchfingers");
        armL = hardwareMap.get(DcMotor.class, "armL");
        armR = hardwareMap.get(DcMotor.class, "armR");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");
         //the arm motor
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        //Below commented code works
        encoderMove(witchfingersMotor, COUNTS_PER_INCH_WITCHFINGERS, 0.5, 15, 5);
        sleep(500);
        driveUntilLimit(0.25,6, "right");
        sleep(500);
        encoderMove(witchfingersMotor, COUNTS_PER_INCH_WITCHFINGERS, 0.5, -15, 5);
        sleep(500);
        driveUntilLimit(0.25,30, "left");
        sleep(500);
        driveInDirection(0.25, 50, 5,"up");
        sleep(500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(500);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public void encoderMove(DcMotor motor, double counts_per_inch, double speed, double inches, double timeoutS) {
        int target;

        if (opModeIsActive()) {
            target = motor.getCurrentPosition() + (int) (inches * counts_per_inch);
            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && motor.isBusy()) {
                telemetry.addData("Running to", "%7d", target);
                telemetry.addData("Currently at", "%7d", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250); // optional pause
        }
    }

    public void driveInDirection(double speed, double inches, double timeouts, String direction) {
        int frontLL = 0;
        int backLL = 0;
        int backRL = 0;
        int frontRL = 0;
        boolean isMotorsBusy = false;

        if (opModeIsActive()) {
            // Determine motor power and target positions based on direction
            switch (direction) {
                case "right":
                    // Move right (strafe right)
                    frontLL = frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL = backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;

                case "left":
                    // Move left (strafe left)
                    frontLL = frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL = backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;

                case "up":
                    // Move up (forward)
                    frontLL = frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;

                case "down":
                    // Move down (backward)
                    frontLL = frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL = backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL = backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;
            }
            frontLeft.setTargetPosition(frontLL);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setPower(Math.abs(speed));
            frontRight.setTargetPosition(frontRL);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setPower(Math.abs(speed));
            backLeft.setTargetPosition(backLL);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setPower(Math.abs(speed));
            backRight.setTargetPosition(backRL);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setPower(Math.abs(speed));
            runtime.reset();
            isMotorsBusy = frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();

            while (opModeIsActive() && runtime.seconds() < timeouts && isMotorsBusy) {
                telemetry.addData("frontLeft is running to", "%7d", frontLL);
                telemetry.addData("frontLeft is currently at", "%7d", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight is running to", "%7d", frontRL);
                telemetry.addData("frontRight is currently at", "%7d", frontRight.getCurrentPosition());
                telemetry.addData("backLeft is running to", "%7d", backLL);
                telemetry.addData("backLeft is currently at", "%7d", backLeft.getCurrentPosition());
                telemetry.addData("backRight is running to", "%7d", backRL);
                telemetry.addData("backRight is currently at", "%7d", backRight.getCurrentPosition());
                telemetry.update();

                isMotorsBusy = frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
            }
        }
    }



    public void driveUntilLimit(double speed, double inches, String direction) {

        if (opModeIsActive()) {

            if (direction.equals("right")){
                while (opModeIsActive() && sensorDistance.getDistance(DistanceUnit.INCH) >= inches) {
                    // Move right
                    frontLeft.setPower(-speed);
                    frontRight.setPower(speed);
                    backLeft.setPower(speed);
                    backRight.setPower(-speed);
                    telemetry.addData("Distance (in)", sensorDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
            }
            else if (direction.equals("left")){
                while (opModeIsActive() && sensorDistance.getDistance(DistanceUnit.INCH) <= inches) {
                    // Move left
                    frontLeft.setPower(speed);
                    frontRight.setPower(-speed);
                    backLeft.setPower(-speed);
                    backRight.setPower(speed);
                    telemetry.addData("Distance (in)", sensorDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
            }

            // Stop all motors once the target distance is reached
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }



    public void driveMotors(double speed, double inches, double timeoutS) {
        int target;
        boolean isMotorsBusy = false;
        int frontR = 0;
        int frontL = 0;
        int backL = 0;
        int backR = 0;

        if (opModeIsActive()) {

            backR = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRight.setTargetPosition(backR);
                    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRight.setPower(Math.abs(speed));

            backL = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLeft.setTargetPosition(backL);
                    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeft.setPower(Math.abs(speed));

            frontR = frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
            frontRight.setTargetPosition(frontR);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setPower(Math.abs(speed));

            frontL = frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
            frontLeft.setTargetPosition(frontL);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            isMotorsBusy = frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
            while (opModeIsActive() && runtime.seconds() < timeoutS && isMotorsBusy) {
                    telemetry.addData("frontLeft is running to", "%7d", frontL);
                    telemetry.addData("frontLeft is currently at", "%7d", frontLeft.getCurrentPosition());
                    telemetry.addData("frontRight Running to", "%7d", frontR);
                    telemetry.addData("frontRight Currently at", "%7d", frontRight.getCurrentPosition());
                    telemetry.addData("backLeft is running to", "%7d", backL);
                    telemetry.addData("BackLeft is currently at", "%7d", backLeft.getCurrentPosition());
                    telemetry.addData("backRight Running to", "%7d", backR);
                    telemetry.addData("backRight Currently at", "%7d", backRight.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
    }
