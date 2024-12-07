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
    final double WRIST_MAX = 0.9;
    final double WRIST_MIN = 0.6;
    final double WRIST_CLIP = 0.3;
    final double CLAW_MAX = 0.6;
    final double CLAW_MIN = 0;

    //TODO play around with the below value to find a good threshold to change from slower robot to faster robot
    final double SLIDE_POSITION_THRESHOLD = 1900;

    // position targets
    double shoulderTarget = 0;
    double wristTarget = 0;
    double clawTarget = 0;
    double slideTarget = 0;

    // wheel speeds
    double speed = 1;
    double activeIntakeSpeed = 0;

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private CRServo activeIntake;
    private Servo holder;

    public void moveRobot() {

        if(slide.getCurrentPosition()>SLIDE_POSITION_THRESHOLD){
            this.speed = 0.2;
        }else{
            this.speed = 1;
        }

        double vertical;
        double horizontal;
        double pivot;

        // back to linear speeds
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        vertical = speed * leftStickY;
        horizontal = speed * -leftStickX;
        pivot = speed * -rightStickX;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void gamepadInput(){
        // first checking overriding operations
        if(gamepad2.a){
            wristTarget = WRIST_MIN;
            moveWrist();
        } else if(gamepad2.y){
            wristTarget = WRIST_MAX;
            moveWrist();
        }else if(gamepad2.dpad_down) {
            activeIntakeSpeed = -0.5; // Reverse direction
        } else if (gamepad2.dpad_up) {
            activeIntakeSpeed = 0.5; // Forward direction
        }else if(gamepad2.x){
            clawTarget = CLAW_MIN;
            // moveClaw();
        }else if(gamepad2.b){
            clawTarget = CLAW_MAX;
            // moveClaw();
        }else if(gamepad2.left_bumper && slide.getCurrentPosition() < 500){
            shoulderTarget = SHOULDER_MIN;
            moveShoulder();
        }else if(gamepad2.right_bumper && slide.getCurrentPosition() < 500){
            shoulderTarget = SHOULDER_MAX;
            moveShoulder();
        }
        else if(gamepad2.left_stick_y != 0){
            final int STEP = 10;   // move increment
            double input = STEP * -gamepad2.left_stick_y;

            shoulderTarget = Math.max(SHOULDER_MIN, Math.min(shoulder.getCurrentPosition() + input,SHOULDER_MAX));

            moveShoulder();
        }else if(gamepad2.right_stick_y != 0){
            final int STEP = 30;
            double input = STEP * -gamepad2.right_stick_y;
            double max;

            if(shoulder.getCurrentPosition() > 500){
                max = SLIDE_Y_MAX;
            }else{
                max = SLIDE_X_MAX;
            }
            slideTarget = Math.max(SLIDE_MIN, Math.min(max, slide.getCurrentPosition() + input));

            moveSlide();
        }else if(gamepad2.right_trigger>0.5 && shoulder.getCurrentPosition() < 500){ // used 0.5 to add a threshold to allow for no mis hits
            clawTarget = CLAW_MIN;
            moveClaw(); // making sure claw happens at this instant and same with the other moves.
            wristTarget = WRIST_MIN;
            moveWrist();
            slideTarget = SLIDE_MIN;
            moveSlide();
        }else if(gamepad2.right_trigger>0.5 && shoulder.getCurrentPosition() > 500){
            clawTarget = CLAW_MAX;
            moveClaw(); // making sure claw happens at this instant and same with the other moves.
            wristTarget = WRIST_MAX;
            moveWrist();
            slideTarget = SLIDE_MIN;
            moveSlide();
        }else if(gamepad2.left_trigger>0.5 && shoulder.getCurrentPosition() < 500){
            clawTarget = CLAW_MAX;
            moveClaw(); // making sure claw happens at this instant and same with the other moves.
            wristTarget = WRIST_MAX;
            moveWrist();
            slideTarget = SLIDE_X_MAX;
            moveSlide();
            shoulderTarget = SHOULDER_MIN;
            moveShoulder();
        }else if(gamepad2.left_trigger>0.5 && shoulder.getCurrentPosition() > 500){
            clawTarget = CLAW_MAX;
            moveClaw(); // making sure claw happens at this instant and same with the other moves.
            wristTarget = WRIST_MAX;
            moveWrist();
            slideTarget = SLIDE_Y_MAX;
            moveSlide();
            shoulderTarget = SHOULDER_MAX;
            moveShoulder();
        }
    }


    public void moveWrist() {
        wrist.setPosition(wristTarget);
        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", 0.0, wristTarget);
    }


    public void moveClaw() {
        holder.setPosition(clawTarget);
        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", 0.0, clawTarget);
    }


    public void moveShoulder() {
        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
        telemetry.addData("Shoulder Current / Target ", "(%d, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
    }


    public void moveSlide() {
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        telemetry.addData("Slide Current / Target ", "(%d, %.2f)", slide.getCurrentPosition(), slideTarget);
    }


    public void init() {
        // connect to hardware map
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        holder = hardwareMap.get(Servo.class, "holder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");


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


    public void init_loop() {

    }


    public void loop() {
        moveRobot();
        // moveShoulder();
        // moveSlide();
        // moveWrist();
        moveClaw();
        gamepadInput();

        activeIntake.setPower(activeIntakeSpeed);

        // Display power for debugging
        telemetry.addData("Active Intake Power", activeIntakeSpeed);
        telemetry.update();
    }

}