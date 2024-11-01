package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="DriftCode")
public class DriftCode extends OpMode {

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor elevator;
    private DcMotor arm;

    public void moveRobot(){
        double vertical;
        double horizontal;
        double pivot;
        double speed = 0.75;

        vertical = speed * gamepad1.left_stick_y;
        horizontal = speed * -gamepad1.left_stick_x;
        pivot = speed * -gamepad1.right_stick_x;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void moveArm(){
        double speed = -gamepad2.right_stick_y;



        arm.setPower(speed);
    }

    public void moveLift() {
        // Define the limits
        double maxPosition = 6500; // Maximum position (top)
        double minPosition = 0; // Minimum position (bottom)

        // Allows for sensitive input
        double speed = Math.pow(gamepad2.left_stick_y, 3);
        telemetry.addData("Speed:", speed);

        // Determine if the elevator is within bounds
        if (elevator.getCurrentPosition() < maxPosition && speed > 0) {
            // Prevent moving up if at the max position
            telemetry.addData("Elevator going out", 1);
            elevator.setPower(speed);
        } else if (elevator.getCurrentPosition() > minPosition && speed < 0) {
            // Prevent moving down if at the min position
            telemetry.addData("Elevator going in", 2);
            elevator.setPower(speed);
        } else {
            // Set elevator power based on joystick input if within bounds
            telemetry.addData("Elevator stopped", 3);
            elevator.setPower(0);
        }
    }




    public void init(){
        // connect to hardware map
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // setting encoders
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();
        moveLift();
        moveArm();

        telemetry.addData("Elevator Position", elevator.getCurrentPosition());
        telemetry.update();
    }
}
