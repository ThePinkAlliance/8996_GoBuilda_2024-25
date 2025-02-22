package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ExampleMultiThreadedOpMode", group="Auto")
//Extend ThreadOpMode rather than OpMode
public class ExampleOpMode extends ThreadOpMode {

    //Define global variables
    private DcMotor dcMotor;

    @Override
    public void mainInit() {
        //Perform your normal init
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm_lift");

        //Below is a new thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //The loop method should contain what to constantly run in the thread
                //For instance, this drives a single DcMotor
                armMotor.setPower(gamepad1.left_stick_y);
            }
        }));
    }
    public int new_var = 0;
    @Override
    public void mainLoop() {
        //Anything you want to constantly run in the MAIN thread goes here
        new_var += 1;
        telemetry.addData("", new_var);
        telemetry.update();
    }
}