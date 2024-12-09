package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Pioneer One TeleOp")
public class PioneerOneTeleop extends OpMode {

    // Performance constants

    //TODO change names for shoulder and wrist and claw max and min so its easier to understand
    final int SLIDE_Y_MAX = 2400;
    final int SLIDE_X_MAX = 1000; // Maximum position (top)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 1;
    final int SHOULDER_MAX = 1400;
    final int SHOULDER_MIN = 0;
    final double SHOULDER_POWER = 0.6;
    final double WRIST_UP = 0.6;
    final double WRIST_DOWN = 0.9;
    final double WRIST_CLIP = 0.3;
    final double CLAW_OPEN = 0.6;
    final double CLAW_CLOSED = 0;

    //TODO play around with the below value to find a good threshold to change from slower robot to faster robot
    final double SLIDE_POSITION_THRESHOLD = 1900;

    // position targets
    double shoulderTarget = 0;
    double wristTarget = 0;
    double clawTarget = 0;
    double slideTarget = 0;

    // wheel speeds
    double wheelSpeed = 1;

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;

    public void moveRobot() {

        if(slide.getCurrentPosition()>SLIDE_POSITION_THRESHOLD){
            this.wheelSpeed = 0.2;
        }else{
            this.wheelSpeed = 1;
        }

        double vertical;
        double horizontal;
        double pivot;

        // back to linear speeds
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        vertical = wheelSpeed * leftStickY;
        horizontal = wheelSpeed * -leftStickX;
        pivot = wheelSpeed * -rightStickX;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void gamepadInput(){
        // first checking overriding operations
        if(gamepad2.a){
            wristTarget = WRIST_DOWN;
        }
        if(gamepad2.y){
            wristTarget = WRIST_UP;
        }
        if(gamepad2.x){
            clawTarget = CLAW_OPEN;
        }
        if(gamepad2.b){
            clawTarget = CLAW_CLOSED;
        }
        if(gamepad2.left_bumper && slide.getCurrentPosition() < 500){
            shoulderTarget = SHOULDER_MIN;
        }
        if(gamepad2.right_bumper && slide.getCurrentPosition() < 500) {
            shoulderTarget = SHOULDER_MAX;
        }
        if(gamepad2.left_stick_y != 0){
            final int STEP = 50;
            double input = STEP * -gamepad2.left_stick_y;

            shoulderTarget = Math.max(SHOULDER_MIN, Math.min(shoulder.getCurrentPosition() + input,SHOULDER_MAX));
        }
        if(gamepad2.right_stick_y != 0){
            final int STEP = 50;
            double input = STEP * -gamepad2.right_stick_y;
            double max;

            if(shoulder.getCurrentPosition() > 500){
                max = SLIDE_Y_MAX;
            }else{
                max = SLIDE_X_MAX;
            }
            slideTarget = Math.max(SLIDE_MIN, Math.min(max, slide.getCurrentPosition() + input));
        } if (gamepad2.right_trigger>0.5 && shoulder.getCurrentPosition() < 500){
            // this code initates when right trigger is pressed and robot is ready to pick up sample
            clawTarget = CLAW_CLOSED;
            wristTarget = WRIST_UP;
            slideTarget = SLIDE_MIN;
        }
        if(gamepad2.right_trigger>0.5 && shoulder.getCurrentPosition() > 500){
            clawTarget = CLAW_OPEN;
            wristTarget = WRIST_DOWN;
            slideTarget = SLIDE_MIN;
        }
        if(gamepad2.left_trigger>0.5 && shoulder.getCurrentPosition() < 500){
            clawTarget = CLAW_OPEN;
            wristTarget = WRIST_DOWN;
            slideTarget = SLIDE_X_MAX;
            shoulderTarget = SHOULDER_MIN;
        }
        if(gamepad2.left_trigger>0.5 && shoulder.getCurrentPosition() > 500){
            shoulderTarget = SHOULDER_MAX;
            slideTarget = SLIDE_Y_MAX;
            wristTarget = WRIST_UP;
        }
    }


    public void moveWrist() {
        wrist.setPosition(wristTarget);
        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", 0.0, wristTarget);
    }


    public void moveClaw() {
        claw.setPosition(clawTarget);
        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", 0.0, clawTarget);
    }


    public void moveShoulder() {
        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
    }


    public void moveSlide() {
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
    }


    public void init() {
        // connect to hardware map
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");


        // setting encoders
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brakes
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

/*
    public void start(){
        shoulder.setTargetPosition(-700);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));

        // waits for shoulder to complete

        while(shoulder.isBusy()){
            telemetry.addData("Shoulder Going to Position", 0);
            telemetry.addData("Shoulder Current / Target ", "(%d, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);

            if(shoulder.getCurrentPosition() < -650){
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shoulder.setPower(0);
            }
        }
    }
*/


    public void init_loop() {

    }


    public void loop() {
        moveRobot();
        moveShoulder();
        moveSlide();
        moveWrist();
        moveClaw();
        gamepadInput();
    }

}