package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;





@TeleOp (name="PioneerOneTeleOp")

public class PioneerOneTeleOp extends OpMode {

    // Performance constants

    final int SHOULDER_MAX = 1000;

    final int SHOULDER_MIN = 0;

    final double SHOULDER_POWER = 0.5;

    final double WRIST_MAX = 0.7;

    final double WRIST_MIN = 0;

    final double CLAW_MAX = 0.6;

    final double CLAW_MIN = 0;



    // position targets

    int shoulderTarget = 0;

    double wristTarget = 0;

    double clawTarget = 0;



    // initalizing motors

    private DcMotor BLeft;

    private DcMotor BRight;

    private DcMotor FLeft;

    private DcMotor FRight;

    private DcMotor slide;

    private DcMotor shoulder;

    private Servo wrist;

    private Servo holder;

    private Servo claw;





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



    public void moveWrist(){

        double STEP = 0.02;

        if(gamepad2.a){

            wristTarget = Math.max(wristTarget - STEP, WRIST_MIN);

        }else if(gamepad2.y){

            wristTarget = Math.min(wristTarget + STEP, WRIST_MAX);

        }

        wrist.setPosition(wristTarget);

        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", -1.0,  wristTarget );

    }



    public void moveClaw(){

        double STEP = 0.02;

        if(gamepad2.x){

            clawTarget = Math.max(clawTarget - STEP, CLAW_MIN);

        }else if(gamepad2.b){

            clawTarget = Math.min(clawTarget + STEP, CLAW_MAX);

        }

        claw.setPosition(clawTarget);

        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", -1.0,  clawTarget );

    }



    public void moveShoulder() {

        final int STEP = 10;   // move increment

        // read control inputs and set target position, keeping within limits

        if (gamepad2.dpad_up){

            shoulderTarget = SHOULDER_MAX;

        } else if (gamepad2.dpad_down){

            shoulderTarget = SHOULDER_MIN;

        } else if (gamepad2.dpad_left){

            shoulderTarget = Math.max(shoulderTarget - STEP, SHOULDER_MIN);

        } else if (gamepad2.dpad_right){

            shoulderTarget = Math.min(shoulderTarget + STEP, SHOULDER_MAX);

        }

        shoulder.setTargetPosition(shoulderTarget);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(Math.abs(SHOULDER_POWER));

        telemetry.addData("Shoulder Current / Target ", "(%d, %d)", shoulder.getCurrentPosition(), shoulderTarget );

    }



    public void moveSlide() {

        double maxPosition = 6000; // Maximum position (top)

        double minPosition = 0; // Minimum position (bottom)



        double speed = Math.pow(gamepad2.left_stick_y, 3);

        telemetry.addData("Speed:", speed);



        // Determine if the elevator is within bounds

        if (slide.getCurrentPosition() >= maxPosition && speed > 0) {

            telemetry.addData("Slide at max", 1);

            slide.setPower(0); // Stop power

        } else if (slide.getCurrentPosition() <= minPosition && speed < 0) {

            telemetry.addData("Slide at min", 2);

            slide.setPower(0); // Stop power

        }else{

            slide.setPower(speed);

        }

    }



    public void init(){

        // connect to hardware map

        BLeft = hardwareMap.get(DcMotor.class, "backleft");

        BRight = hardwareMap.get(DcMotor.class, "backright");

        FLeft = hardwareMap.get(DcMotor.class, "frontleft");

        FRight = hardwareMap.get(DcMotor.class, "frontright");

        slide = hardwareMap.get(DcMotor.class, "elevator");

        shoulder = hardwareMap.get(DcMotor.class, "arm");

        claw = hardwareMap.get(Servo.class, "holder");

        wrist = hardwareMap.get(Servo.class, "wrist");

//        holder = hardwareMap.get(Servo.class, "holder");



        // setting encoders

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // brakes

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // reverse the motor directions

        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);



    }



    public void init_loop() {

    }





    public void loop(){

        moveRobot();

        moveShoulder();

        moveSlide();

        moveWrist();

        moveClaw();



        telemetry.addData("Slide Position", slide.getCurrentPosition());

        telemetry.update();

    }

}