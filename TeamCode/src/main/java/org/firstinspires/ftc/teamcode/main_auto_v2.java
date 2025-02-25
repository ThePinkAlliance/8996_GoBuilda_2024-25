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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates two sets of driving methods:
 *   1) Traditional encoder-based moves (encoderMove, driveInDirection, driveUntilLimit, driveMotors)
 *   2) Gyro-based moves that use the IMU for heading control (driveStraightGyro, turnToHeading, holdHeading, etc.)
 *
 * You can choose which method to use in your autonomous routine.
 */

@Autonomous(name="autonomous MAIN V2", group="Auto")
public class main_auto_v2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor witchfingersMotor = null;
    private DistanceSensor sensorDistance = null;
    private IMU imu = null; // Control/Expansion Hub IMU
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for Gobilda drive (encoder conversion for the drive motors)
    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 384.5;
    static final double DRIVE_GEAR_REDUCTION_GOBUILDA = 1.0;
    static final double WHEEL_DIAMETER_INCHES_GOBUILDA = 3.78;
    static final double COUNTS_PER_INCH_GOBUILDA = (COUNTS_PER_MOTOR_REV_GOBUILDA * DRIVE_GEAR_REDUCTION_GOBUILDA) /
            (WHEEL_DIAMETER_INCHES_GOBUILDA * 3.1415);
    // Constants for witchfingers (if used)
    static final double COUNTS_PER_MOTOR_REV_WITCHFINGERS = 28;
    static final double DRIVE_GEAR_REDUCTION_WITCHFINGERS = 12;
    static final double SPOOL_DIAMETER_INCHES_WITCHFINGERS = 1.2;
    static final double COUNTS_PER_INCH_WITCHFINGERS = (COUNTS_PER_MOTOR_REV_WITCHFINGERS * DRIVE_GEAR_REDUCTION_WITCHFINGERS) /
            (SPOOL_DIAMETER_INCHES_WITCHFINGERS * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    // ----- Gyro-drive member variables (for our integrated gyro-based functions) -----
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeedGyro = 0;
    private double turnSpeedGyro = 0;
    private double leftSpeedGyro = 0;
    private double rightSpeedGyro = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        witchfingersMotor = hardwareMap.get(DcMotor.class, "witchfingers");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Define Hub orientation.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set motor directions.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Telemetry to show starting encoder positions.
        telemetry.addData("Starting at", "%7d : %7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // ---------------------------
        // Example sequence using encoder-based moves:
        // ---------------------------
        encoderMove(witchfingersMotor, COUNTS_PER_INCH_WITCHFINGERS, 0.5, 15, 5);
        sleep(500);
        driveUntilLimit(0.25, 6, "right");
        sleep(500);
        encoderMove(witchfingersMotor, COUNTS_PER_INCH_WITCHFINGERS, 0.5, -15, 5);
        sleep(500);
        driveUntilLimit(0.25, 30, "left");
        sleep(500);
        driveInDirection(0.25, 50, 5, "up");
        sleep(500);

        // ---------------------------
        // Example sequence using gyro-based moves:
        // (Uncomment the following lines if you wish to use the gyro methods.)
        // ---------------------------
        // driveStraightGyro(0.6, 24.0, 0.0);       // Drive forward 24 inches at 0° heading.
        // turnToHeading(0.5, -45.0);               // Turn to -45°.
        // holdHeading(0.5, -45.0, 0.5);            // Hold -45° for 0.5 seconds.
        // driveStraightGyro(0.6, 17.0, -45.0);      // Drive 17 inches while maintaining -45°.
        // turnToHeading(0.5, 45.0);                // Turn to 45°.
        // holdHeading(0.5, 45.0, 0.5);             // Hold 45° for 0.5 seconds.
        // driveStraightGyro(0.6, 17.0, 45.0);       // Drive 17 inches while maintaining 45°.
        // turnToHeading(0.5, 0.0);                 // Turn to 0°.
        // holdHeading(0.5, 0.0, 1.0);              // Hold 0° for 1 second.
        // driveStraightGyro(0.6, -48.0, 0.0);       // Drive backward 48 inches.

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(500);  // Pause to display final telemetry.
    }

    // =========================================================================
    // --- Original encoder-based drive methods (unchanged) ---
    // =========================================================================

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
            switch (direction) {
                case "right":
                    // Strafe right.
                    frontLL = frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL  = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL  = backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;
                case "left":
                    // Strafe left.
                    frontLL = frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL  = backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL  = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;
                case "up":
                    // Move forward.
                    frontLL = frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL  = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL  = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    break;
                case "down":
                    // Move backward.
                    frontLL = frontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    frontRL = frontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backLL  = backLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
                    backRL  = backRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_GOBUILDA);
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
                telemetry.addData("frontLeft target", "%7d", frontLL);
                telemetry.addData("frontLeft current", "%7d", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight target", "%7d", frontRL);
                telemetry.addData("frontRight current", "%7d", frontRight.getCurrentPosition());
                telemetry.addData("backLeft target", "%7d", backLL);
                telemetry.addData("backLeft current", "%7d", backLeft.getCurrentPosition());
                telemetry.addData("backRight target", "%7d", backRL);
                telemetry.addData("backRight current", "%7d", backRight.getCurrentPosition());
                telemetry.update();
                isMotorsBusy = frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
            }
        }
    }

    /**
     * Drives in a specified direction while using gyro feedback to maintain a desired heading.
     * This method combines the encoder-based target calculation from driveInDirection with the
     * gyro-based heading correction from driveStraightGyro. It uses a mecanum-style mix of translation
     * and rotation.
     *
     * @param maxDriveSpeed Maximum translational speed (0..1)
     * @param distance      Distance in inches to travel (always positive; direction determined by the "direction" parameter)
     * @param heading       Desired heading (in degrees) to hold during the move
     * @param direction     Movement direction: "up" (forward), "down" (backward), "right" (strafe right), or "left" (strafe left)
     */
    public void driveInDirectionGyro(double maxDriveSpeed, double distance, double heading, String direction) {
        // Compute target encoder counts for each motor based on the specified direction.
        int targetFL = 0, targetFR = 0, targetBL = 0, targetBR = 0;
        int counts = (int)(distance * COUNTS_PER_INCH_GOBUILDA);

        // The following switch uses the same idea as in driveInDirection.
        switch(direction.toLowerCase()) {
            case "right":
                // For strafing right, the target increments are different on each wheel.
                targetFL = frontLeft.getCurrentPosition() - counts;
                targetFR = frontRight.getCurrentPosition() + counts;
                targetBL = backLeft.getCurrentPosition() + counts;
                targetBR = backRight.getCurrentPosition() - counts;
                break;
            case "left":
                targetFL = frontLeft.getCurrentPosition() + counts;
                targetFR = frontRight.getCurrentPosition() - counts;
                targetBL = backLeft.getCurrentPosition() - counts;
                targetBR = backRight.getCurrentPosition() + counts;
                break;
            case "up":
                // For forward (up), all motors add the same count.
                targetFL = frontLeft.getCurrentPosition() + counts;
                targetFR = frontRight.getCurrentPosition() + counts;
                targetBL = backLeft.getCurrentPosition() + counts;
                targetBR = backRight.getCurrentPosition() + counts;
                break;
            case "down":
                targetFL = frontLeft.getCurrentPosition() - counts;
                targetFR = frontRight.getCurrentPosition() - counts;
                targetBL = backLeft.getCurrentPosition() - counts;
                targetBR = backRight.getCurrentPosition() - counts;
                break;
            default:
                // If an unknown direction is passed, default to forward.
                targetFL = frontLeft.getCurrentPosition() + counts;
                targetFR = frontRight.getCurrentPosition() + counts;
                targetBL = backLeft.getCurrentPosition() + counts;
                targetBR = backRight.getCurrentPosition() + counts;
                break;
        }

        // Set target positions and change modes.
        frontLeft.setTargetPosition(targetFL);
        frontRight.setTargetPosition(targetFR);
        backLeft.setTargetPosition(targetBL);
        backRight.setTargetPosition(targetBR);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine the base translational components.
        // For a mecanum drive, we use (vx, vy) to represent lateral and forward motion.
        double vx = 0, vy = 0;
        switch(direction.toLowerCase()) {
            case "right":   vx = maxDriveSpeed;  break;
            case "left":    vx = -maxDriveSpeed; break;
            case "up":      vy = maxDriveSpeed;  break;
            case "down":    vy = -maxDriveSpeed; break;
            default:        vy = maxDriveSpeed;  break;
        }

        // Loop until all motors have reached their targets.
        while (opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Compute rotational correction from gyro.
            double rotationCorrection = getSteeringCorrection(heading, 0.03);

            // For mecanum drive, motor powers combine translation and rotation.
            // The typical equations:
            //   frontLeft  = vy + vx + rotation
            //   frontRight = vy - vx - rotation
            //   backLeft   = vy - vx + rotation
            //   backRight  = vy + vx - rotation
            double powerFL = vy + vx + rotationCorrection;
            double powerFR = vy - vx - rotationCorrection;
            double powerBL = vy - vx + rotationCorrection;
            double powerBR = vy + vx - rotationCorrection;

            // Normalize the wheel powers if any is outside the [-1, 1] range.
            double maxPower = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                    Math.max(Math.abs(powerBL), Math.abs(powerBR))));
            if (maxPower > 1.0) {
                powerFL /= maxPower;
                powerFR /= maxPower;
                powerBL /= maxPower;
                powerBR /= maxPower;
            }

            // Set the motor powers.
            frontLeft.setPower(powerFL);
            frontRight.setPower(powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

            // Optionally display telemetry.
            telemetry.addData("Desired Heading", heading);
            telemetry.addData("Current Heading", getHeading());
            telemetry.addData("Rotation Correction", rotationCorrection);
            telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", powerFL, powerFR, powerBL, powerBR);
            telemetry.update();
        }

        // Stop all motion.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Switch back to RUN_USING_ENCODER.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void driveUntilLimit(double speed, double inches, String direction) {
        if (opModeIsActive()) {
            if (direction.equals("right")) {
                while (opModeIsActive() && sensorDistance.getDistance(DistanceUnit.INCH) >= inches) {
                    // Strafe right.
                    frontLeft.setPower(-speed);
                    frontRight.setPower(speed);
                    backLeft.setPower(speed);
                    backRight.setPower(-speed);
                    telemetry.addData("Distance (in)", sensorDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
            } else if (direction.equals("left")) {
                while (opModeIsActive() && sensorDistance.getDistance(DistanceUnit.INCH) <= inches) {
                    // Strafe left.
                    frontLeft.setPower(speed);
                    frontRight.setPower(-speed);
                    backLeft.setPower(-speed);
                    backRight.setPower(speed);
                    telemetry.addData("Distance (in)", sensorDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
            }
            // Stop all motors.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    public void driveMotors(double speed, double inches, double timeoutS) {
        int frontR = 0;
        int frontL = 0;
        int backL = 0;
        int backR = 0;
        boolean isMotorsBusy = false;
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
                telemetry.addData("frontLeft target", "%7d", frontL);
                telemetry.addData("frontLeft current", "%7d", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight target", "%7d", frontR);
                telemetry.addData("frontRight current", "%7d", frontRight.getCurrentPosition());
                telemetry.addData("backLeft target", "%7d", backL);
                telemetry.addData("backLeft current", "%7d", backLeft.getCurrentPosition());
                telemetry.addData("backRight target", "%7d", backR);
                telemetry.addData("backRight current", "%7d", backRight.getCurrentPosition());
                telemetry.update();
                isMotorsBusy = frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
            }
        }
    }

    // =========================================================================
    // --- Gyro-based drive functions (integrated from the other code) ---
    // =========================================================================

    /**
     * Drive straight a given distance (in inches) while maintaining a specific heading.
     * Uses RUN_TO_POSITION with gyro-based correction.
     *
     * @param maxDriveSpeed Maximum forward speed (0..1).
     * @param distance      Distance in inches to drive (negative for reverse).
     * @param heading       Desired heading (in degrees).
     */
    public void driveStraightGyro(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            // Calculate target counts using Gobilda constants.
            int moveCounts = (int)(distance * COUNTS_PER_INCH_GOBUILDA);
            leftTarget = frontLeft.getCurrentPosition() + moveCounts;
            rightTarget = frontRight.getCurrentPosition() + moveCounts;

            // Set target positions for both left and right side motors.
            frontLeft.setTargetPosition(leftTarget);
            backLeft.setTargetPosition(leftTarget);
            frontRight.setTargetPosition(rightTarget);
            backRight.setTargetPosition(rightTarget);

            // Switch to RUN_TO_POSITION mode.
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            driveSpeedGyro = Math.abs(maxDriveSpeed);
            moveRobot(driveSpeedGyro, 0);

            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                // Get steering correction using a proportional gain (e.g., 0.03).
                turnSpeedGyro = getSteeringCorrection(heading, 0.03);
                if (distance < 0)
                    turnSpeedGyro *= -1.0;
                moveRobot(driveSpeedGyro, turnSpeedGyro);
                sendTelemetryGyro(true);
            }

            // Stop motion and return motors to RUN_USING_ENCODER.
            moveRobot(0, 0);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turn the robot to a specified heading.
     *
     * @param maxTurnSpeed Maximum turn speed (0..1).
     * @param heading      Desired heading (in degrees).
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {
        // Pre-calculate steering correction.
        getSteeringCorrection(heading, 0.02);
        while (opModeIsActive() && (Math.abs(headingError) > 1.0)) {  // HEADING_THRESHOLD = 1.0
            turnSpeedGyro = getSteeringCorrection(heading, 0.02);
            turnSpeedGyro = Range.clip(turnSpeedGyro, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeedGyro);
            sendTelemetryGyro(false);
        }
        moveRobot(0, 0);
    }

    /**
     * Hold the current heading for a specified duration.
     *
     * @param maxTurnSpeed Maximum turn speed (0..1).
     * @param heading      Heading to hold (in degrees).
     * @param holdTime     Duration (in seconds) to hold the heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.seconds() < holdTime)) {
            turnSpeedGyro = getSteeringCorrection(heading, 0.02);
            turnSpeedGyro = Range.clip(turnSpeedGyro, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, turnSpeedGyro);
            sendTelemetryGyro(false);
        }
        moveRobot(0, 0);
    }

    /**
     * Compute the steering correction based on the desired heading.
     *
     * @param desiredHeading   The target heading (in degrees).
     * @param proportionalGain The gain factor.
     * @return A correction value between -1 and +1.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Combine drive and turn commands and set power to the four drive motors.
     *
     * @param drive Forward/reverse component.
     * @param turn  Turning component.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeedGyro = drive;
        turnSpeedGyro = turn;
        leftSpeedGyro = drive - turn;
        rightSpeedGyro = drive + turn;
        double max = Math.max(Math.abs(leftSpeedGyro), Math.abs(rightSpeedGyro));
        if (max > 1.0) {
            leftSpeedGyro /= max;
            rightSpeedGyro /= max;
        }
        frontLeft.setPower(leftSpeedGyro);
        backLeft.setPower(leftSpeedGyro);
        frontRight.setPower(rightSpeedGyro);
        backRight.setPower(rightSpeedGyro);
    }

    /**
     * Display telemetry data for gyro-based driving.
     *
     * @param straight True if in a straight-driving segment (displays encoder targets and positions).
     */
    public void sendTelemetryGyro(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d : %7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d : %7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }
        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeedGyro);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeedGyro, rightSpeedGyro);
        telemetry.update();
    }

    /**
     * Get the current heading (yaw) from the IMU.
     *
     * @return Heading in degrees.
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }
}
