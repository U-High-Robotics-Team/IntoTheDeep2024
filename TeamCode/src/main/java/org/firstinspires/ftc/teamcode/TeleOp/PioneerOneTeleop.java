package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Pioneer One TeleOp")
public class PioneerOneTeleop extends OpMode {

    // Performance constants
    final int SLIDE_Y_MAX = 2400;
    final int SLIDE_X_MAX = 1000; // Maximum position (top)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 1.0;
    final int SHOULDER_MAX = 1500;
    final int SHOULDER_MIN = 0;
    final double SHOULDER_POWER = 0.5;
    final double WRIST_MAX = 0.6;
    final double WRIST_MIN = 0.0;
    final double WRIST_CLIP = 0.3;
    final double CLAW_MAX = 0.6;
    final double CLAW_MIN = 0;

    // position targets
    double shoulderTarget = 0;
    double wristTarget = 0;
    double clawTarget = 0;
    double slideTarget = 0;

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

    public void moveRobot() {

        double vertical;
        double horizontal;
        double pivot;
        double speed = 0.5;


        double leftStickY = Math.pow(gamepad1.left_stick_y, 3);
        double leftStickX = Math.pow(gamepad1.left_stick_x, 3);
        double rightStickX = Math.pow(gamepad1.right_stick_x, 3);
        vertical = speed * leftStickY;
        horizontal = speed * -leftStickX;
        pivot = speed * -rightStickX;


        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }


    public void moveWrist() {
        double STEP = 0.02;

        if (gamepad2.a) {
            // TODO: Try to make it binary operations
            // wristTarget = Math.max(wristTarget - STEP, WRIST_MIN);

            wristTarget = WRIST_MIN;

        } else if (gamepad2.y) {
            // TODO: Try to make it binary operations
            // wristTarget = Math.min(wristTarget + STEP, WRIST_MAX);

            wristTarget = WRIST_MAX;

        } else if (gamepad2.right_bumper){
            wristTarget = WRIST_CLIP;
        }

        wrist.setPosition(wristTarget);
        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", 0.0, wristTarget);
    }


    public void moveClaw() {
        double STEP = 0.02;

        if (gamepad2.x) {
            // TODO: Try to make it binary operations
            // clawTarget = Math.max(clawTarget - STEP, CLAW_MIN);

            clawTarget = CLAW_MIN;

        } else if (gamepad2.b) {
            // TODO: Try to make it binary operations
            // clawTarget = Math.min(clawTarget + STEP, CLAW_MAX);

            clawTarget = CLAW_MAX;

        }

        claw.setPosition(clawTarget);
        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", 0.0, clawTarget);
    }


    public void moveShoulder() {
        final int STEP = 10;   // move increment
        double input = STEP * -Math.pow(gamepad2.left_stick_y, 3);

        shoulderTarget = Math.max(SHOULDER_MIN, Math.min(SHOULDER_MAX, shoulderTarget + input));

        // Previous DPad Controls

        // if (gamepad2.dpad_up) {
        //     shoulderTarget = SHOULDER_MAX;

        // } else if (gamepad2.dpad_down) {
        //     shoulderTarget = SHOULDER_MIN;

        // } else if (gamepad2.dpad_left) {
        //     shoulderTarget = Math.max(shoulderTarget - STEP, SHOULDER_MIN);

        // } else if (gamepad2.dpad_right) {
        //     shoulderTarget = Math.min(shoulderTarget + STEP, SHOULDER_MAX);

        // }

        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
        telemetry.addData("Shoulder Current / Target ", "(%d, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
    }


    public void moveSlide() {
        final int STEP = 30;
        double input = STEP * -Math.pow(gamepad2.right_stick_y, 3);
        double max;

        if(shoulder.getCurrentPosition() > 500){
            max = SLIDE_Y_MAX;
        }else{
            max = SLIDE_X_MAX;
        }
        slideTarget = Math.max(SLIDE_MIN, Math.min(max, slideTarget + input));

        //double power = -Math.pow(gamepad2.right_stick_y, 3);

        // Determine if the elevator is outside of bounds and limit target
        //if (power < 0) {
        //    slideTarget = Math.max(slideTarget - STEP, SLIDE_MIN);
        //} else if (power > 0) {
        //    slideTarget = Math.min(slideTarget + STEP, SLIDE_MAX);
        //}

        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        //slide.setPower(0);
        telemetry.addData("Slide Current / Target ", "(%d, %.2f)", slide.getCurrentPosition(), slideTarget);
        //telemetry.addData("GamePad2.RightStickY Power:", power);
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


    public void init_loop() {

    }


    public void loop() {
        moveRobot();
        moveShoulder();
        moveSlide();
        moveWrist();
        moveClaw();

        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.update();
    }

}