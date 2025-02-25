/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Robot: Auto Drive By Gyro", group="Robot")
public class RobotAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    // Four drive motors: two on each side.
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private IMU     imu        = null;      // Control/Expansion Hub IMU

    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    static final double COUNTS_PER_MOTOR_REV  = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION  = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Define desired driving/control characteristics.
    static final double DRIVE_SPEED       = 0.4;  // Max driving speed for better distance accuracy.
    static final double TURN_SPEED        = 0.2;  // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;  // How close must the heading get to the target before moving to next step.
    static final double P_TURN_GAIN       = 0.02; // Proportional gain when turning.
    static final double P_DRIVE_GAIN      = 0.03; // Proportional gain when driving straight.

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions: left side motors reversed so that positive power moves both sides forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Define Hub orientation.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Reset the encoders and set motors to BRAKE mode.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (displaying the current gyro heading while waiting).
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Set motors for closed loop speed control and reset the heading.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // Step through each leg of the autonomous path.
        driveStraight(DRIVE_SPEED, 24.0, 0.0);         // Drive Forward 24"
        turnToHeading(TURN_SPEED, -45.0);                // Turn CW to -45 Degrees
        holdHeading(TURN_SPEED, -45.0, 0.5);             // Hold -45 Deg for 0.5 seconds

//        driveStraight(DRIVE_SPEED, 17.0, -45.0);         // Drive Forward 17" at -45 degrees
//        turnToHeading(TURN_SPEED, 45.0);                 // Turn CCW to 45 Degrees
//        holdHeading(TURN_SPEED, 45.0, 0.5);              // Hold 45 Deg for 0.5 seconds
//
//        driveStraight(DRIVE_SPEED, 17.0, 45.0);          // Drive Forward 17" at 45 degrees
//        turnToHeading(TURN_SPEED, 0.0);                  // Turn CW to 0 Degrees
//        holdHeading(TURN_SPEED, 0.0, 1.0);               // Hold 0 Deg for 1 second
//
//        driveStraight(DRIVE_SPEED, -48.0, 0.0);          // Drive in Reverse 48"

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display final telemetry.
    }

    // ********************************************************************
    // High-level driving functions.
    // ********************************************************************

    /**
     * Drive in a straight line on a fixed heading (using encoder counts).
     * @param maxDriveSpeed Maximum driving speed (0 to +1.0).
     * @param distance Distance (in inches) to move (negative means reverse).
     * @param heading Absolute heading (in degrees) relative to the last gyro reset.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        if (opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            // Use frontLeft and frontRight as reference for encoder positions.
            leftTarget = frontLeft.getCurrentPosition() + moveCounts;
            rightTarget = frontRight.getCurrentPosition() + moveCounts;

            // Set target positions for both motors on each side.
            frontLeft.setTargetPosition(leftTarget);
            backLeft.setTargetPosition(leftTarget);
            frontRight.setTargetPosition(rightTarget);
            backRight.setTargetPosition(rightTarget);

            // Set all four motors to RUN_TO_POSITION mode.
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // Loop until all motors reach their target.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Get the steering correction to stay on the desired heading.
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply driving and turning power.
                moveRobot(driveSpeed, turnSpeed);
                sendTelemetry(true);
            }

            // Stop motion and switch back to RUN_USING_ENCODER.
            moveRobot(0, 0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turn the robot to a specific heading.
     * @param maxTurnSpeed Maximum turn speed (0 to +1.0).
     * @param heading Desired absolute heading (in degrees).
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Pre-calculate current error.
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // Loop until the heading error is within the threshold.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }
        moveRobot(0, 0);
    }

    /**
     * Hold a heading for a specific duration.
     * @param maxTurnSpeed Maximum differential turn speed (0 to +1.0).
     * @param heading Desired heading (in degrees).
     * @param holdTime Duration to hold the heading (in seconds).
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }
        moveRobot(0, 0);
    }

    // ********************************************************************
    // Low-level driving functions.
    // ********************************************************************

    /**
     * Use a proportional controller to determine steering correction.
     * @param desiredHeading Desired heading (in degrees).
     * @param proportionalGain Gain factor for heading error.
     * @return Steering correction value (clipped to -1 to +1).
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();

        // Normalize error to +/- 180 degrees.
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Combine drive (forward/reverse) and turn (left/right) requests, and apply them to all motors.
     * @param drive Forward motor speed.
     * @param turn  Clockwise turning speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        // Apply the same power to both motors on each side.
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backRight.setPower(rightSpeed);
    }

    /**
     * Display telemetry data.
     * @param straight If true, include encoder positions.
     */
    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d",
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * Get the current heading from the IMU (in degrees).
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
