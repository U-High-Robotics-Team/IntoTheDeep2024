package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="DriftCode")
public class DriftCode extends OpMode {

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor elevator;
    private DcMotor arm;
    private Servo wrist;
    private Servo holder;
    private Servo wheel;


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

    public void intake(){

        if(gamepad2.a){
            // take in
            wheel.setPosition(1.0);
        }else if(gamepad2.b){
            // take out
            wheel.setPosition(0.0);
        }else{
            wheel.setPosition(0.5);
        }


    }

    public void moveArm() {
        double speed = gamepad2.right_stick_y;

        arm.setPower(speed);
//        double currentPosition = arm.getCurrentPosition();
//
//
//        double speedFactor = 10;
//        double targetPosition = currentPosition + (speed * speedFactor);
//
//
//        double maxPosition = 1000;
//        double minPosition = 0;
//
//
//        targetPosition = Math.max(minPosition, Math.min(maxPosition, targetPosition));
//
//
//        double error = targetPosition - currentPosition;
//
//        double power = 0.01 * error;
//
//        arm.setPower(Math.max(-1, Math.min(1, power)));
    }


    public void moveLift() {
        double maxPosition = 6500; // Maximum position (top)
        double minPosition = 0; // Minimum position (bottom)

        double speed = -Math.pow(gamepad2.left_stick_y, 3);
        telemetry.addData("Speed:", speed);

        // Determine if the elevator is within bounds
        if (elevator.getCurrentPosition() >= maxPosition && speed > 0) {
            telemetry.addData("Elevator at max", 1);
            elevator.setPower(0); // Stop power
        }
        else if (elevator.getCurrentPosition() <= minPosition && speed < 0) {
            telemetry.addData("Elevator at min", 2);
            elevator.setPower(0); // Stop power
        }else{
            elevator.setPower(speed);
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
        wheel = hardwareMap.get(Servo.class, "wheel");
        wrist = hardwareMap.get(Servo.class, "wrist");
        holder = hardwareMap.get(Servo.class, "holder");

        // setting encoders
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();
        moveLift();
        moveArm();
        intake();

        telemetry.addData("Elevator Position", elevator.getCurrentPosition());
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Wheel Position", wheel.getPosition());
        telemetry.update();
    }
}
