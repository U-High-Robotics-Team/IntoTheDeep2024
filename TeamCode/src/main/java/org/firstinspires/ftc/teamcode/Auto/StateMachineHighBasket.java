package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "State Machine High Basket")
public class StateMachineHighBasket extends LinearOpMode {

    // Variables for PID Controller

    double kP = 0.0024; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot
    double toleranceLevel = 40; // TODO Find appropriate values
    double bRError;
    double bLError;
    double fRError;
    double fLError;


    // Timer for Servos
    private final ElapsedTime presetTimer = new ElapsedTime();

    // Preset action states
    enum RobotState {
        NONE,
        HOME,
        SUB_1,
        SUB_2,
        SUB_3,
        BASKET_1,
        BASKET_2,
        BASKET_3,
        BASKET_4,
        UNKNOWN             // when moved manually into another pose
    }


    // Performance constants
    final int SLIDE_Y_MAX = 2400;
    final int SLIDE_X_MAX = 1000; // Maximum position (top)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 1;
    final int SHOULDER_MAX = 1400;
    final int SHOULDER_MIN = 0;
    final double SHOULDER_POWER = 0.6;
    final double WRIST_UP = 0;
    final double WRIST_DOWN = 0.65;
    final double WRIST_CLIP = 0.3; // unused currently
    final double CLAW_OPEN = 0.6;
    final double CLAW_CLOSED = 0.25;
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;

    // Thresholds
    final double SLIDE_POSITION_THRESHOLD = 700;
    final double SHOULDER_POSITION_THRESHOLD = 500;

    // Initial Targets
    RobotState currentState = RobotState.HOME;
    RobotState requestedState = RobotState.HOME;
    // TODO: this assumes we have a block at the start
    double shoulderTarget = SHOULDER_MIN;
    double wristTarget = WRIST_UP;
    double clawTarget = CLAW_CLOSED;
    double slideTarget = SLIDE_MIN;
    double wheelSpeed = WHEEL_SPEED_MAX;

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();

        // Strafe Left
        movePosition(480,-480,-480,480);

        // Move Backwards
        movePosition(-720, -720, -720, -720);

        // Rotate Right
        movePosition(550,-550,550,-550);

        // Move Backwards a Little
        movePosition(-150,-150,-150,-150);

        requestedState = RobotState.BASKET_1;

        presetTimer.reset();

        while(presetTimer.seconds()<3.0) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        requestedState = RobotState.BASKET_3;

        presetTimer.reset();

        while(presetTimer.seconds()<2.4) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        movePosition(550,-550,550,-550);
        movePosition(-300,300,300,-300);


        movePosition(350,350,350,350);

        requestedState = RobotState.SUB_1;

        presetTimer.reset();

        while(presetTimer.seconds()<1) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        requestedState = RobotState.SUB_2;

        presetTimer.reset();

        while(presetTimer.seconds()<0.7) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        requestedState = RobotState.HOME;

        presetTimer.reset();

        while(presetTimer.seconds()<2.0) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        movePosition(-350,-350,-350,-350);
        movePosition(300,-300,-300,300);

        movePosition(-550,550,-550,550);

        requestedState = RobotState.BASKET_1;

        presetTimer.reset();

        while(presetTimer.seconds()<3.0) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }

        requestedState = RobotState.BASKET_3;

        presetTimer.reset();

        while(presetTimer.seconds()<4.0) {
            stateMachine();
            moveShoulder();
            moveSlide();
            moveWrist();
            moveClaw();
        }
    }

    public void stateMachine() {
        switch (currentState) {
            case HOME:
                // immediate actions
                clawTarget = CLAW_CLOSED;

                if(presetTimer.seconds() > 0.6){
                    wristTarget = WRIST_UP;
                }
                // delayed actions
                if (presetTimer.seconds() > 1.1) {
                    shoulderTarget = SHOULDER_MIN;
                    slideTarget = SLIDE_MIN;
                }
                // allowed transistions from HOME: SUBMERSIBLE, BASKET_1
                if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.BASKET_1){
                    currentState = RobotState.BASKET_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_1:
                // immediate actions
                clawTarget = CLAW_OPEN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                wristTarget = WRIST_CLIP;
                // allowed transistions from SUB: HOME, SUB_2
                if (requestedState == RobotState.HOME){
                    currentState = RobotState.HOME;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_2){
                    currentState = RobotState.SUB_2;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_2:
                // immediate actions
                wristTarget = WRIST_DOWN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                // delayed actoins
                if(presetTimer.seconds() > 0.4){
                    clawTarget = CLAW_CLOSED;
                }
                if(presetTimer.seconds() > 0.8){
                    currentState = RobotState.SUB_3;
                    presetTimer.reset();
                }
                // allowed transition
                break;


            case SUB_3:
                // immediate actions
                wristTarget = WRIST_UP;
                // allowed transition
                if (requestedState == RobotState.HOME){
                    currentState = RobotState.HOME;
                    presetTimer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    presetTimer.reset();  // start delay timer for wrist movement
                }
                break;

            case BASKET_1:
                // immediate actions
                shoulderTarget = SHOULDER_MAX;
                wristTarget = WRIST_DOWN;
                // delayed actions
                // allowed transistions
                if (presetTimer.seconds() > 1.5) {
                    currentState = RobotState.BASKET_2;
                    presetTimer.reset();
                }
                break;

            case BASKET_2:
                slideTarget = SLIDE_Y_MAX;
                clawTarget = CLAW_CLOSED;

                if(presetTimer.seconds() > 1.4){
                    wristTarget = WRIST_UP;
                }

                if(requestedState == RobotState.BASKET_3){
                    currentState = RobotState.BASKET_3;
                    presetTimer.reset();
                }
                break;

            case BASKET_3:
                // immediate actions
                clawTarget = CLAW_OPEN;

                if(presetTimer.seconds()> 1.0){
                    currentState = RobotState.BASKET_4;
                    presetTimer.reset();
                }
                break;

            case BASKET_4:
                wristTarget = WRIST_DOWN;

                if(presetTimer.seconds()>1.0){
                    slideTarget = SLIDE_MIN;
                    shoulderTarget = SHOULDER_MAX;
                }

                if(presetTimer.seconds()>2.0){
                    currentState = RobotState.HOME;
                    presetTimer.reset();
                }

                break;

            default:
                currentState = RobotState.UNKNOWN;

                break;
        }
        telemetry.addData("State Machine", "Current state: %s", currentState);
    }

    public void moveWrist() {
        wrist.setPosition(wristTarget);
        // telemetry.addData("Wrist Current / Target ", "(%.2f)", wristTarget);
    }

    public void moveClaw() {
        claw.setPosition(clawTarget);
        // telemetry.addData("Claw Current / Target ", "(%.2f)", clawTarget);
    }

    public void moveShoulder() {
        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
        // telemetry.addData("Shoulder Current / Target ", "(%.2f, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
    }

    public void moveSlide() {
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        // telemetry.addData("Slide Current / Target ", "(%.2f, %.2f)", slide.getCurrentPosition(), slideTarget);
    }


    public void movePosition(int FRTargetSteps, int FLTargetSteps, int BRTargetSteps, int BLTargetSteps){
        double bRLastError = 0;
        double fRLastError = 0;
        double fLLastError = 0;
        double bLLastError = 0;
        ElapsedTime timer = new ElapsedTime();

        FRTargetSteps = FRTargetSteps + FRight.getCurrentPosition();
        FLTargetSteps = FLTargetSteps + FLeft.getCurrentPosition();
        BRTargetSteps = BRTargetSteps + BRight.getCurrentPosition();
        BLTargetSteps = BLTargetSteps + BLeft.getCurrentPosition();

        this.fRError = FRTargetSteps - FRight.getCurrentPosition();
        this.fLError = FLTargetSteps - FLeft.getCurrentPosition();
        this.bRError = BRTargetSteps - BRight.getCurrentPosition();
        this.bLError = BLTargetSteps - BLeft.getCurrentPosition();

        while(Math.abs(fRError) > toleranceLevel && opModeIsActive() && Math.abs(fLError) > toleranceLevel && Math.abs(bRError) > toleranceLevel && Math.abs(bLError) > toleranceLevel){

            this.fRError = FRTargetSteps - FRight.getCurrentPosition();
            this.fLError = FLTargetSteps - FLeft.getCurrentPosition();
            this.bRError = BRTargetSteps - BRight.getCurrentPosition();
            this.bLError = BLTargetSteps - BLeft.getCurrentPosition();

            FRight.setPower(Math.max(Math.min(findPIDPower(FRTargetSteps, FRight, timer, fRLastError), 1), -1));
            FLeft.setPower(Math.max(Math.min(findPIDPower(FLTargetSteps, FLeft, timer, fLLastError), 1), -1));
            BRight.setPower(Math.max(Math.min(findPIDPower(BRTargetSteps, BRight, timer, bRLastError), 1), -1));
            BLeft.setPower(Math.max(Math.min(findPIDPower(BLTargetSteps, BLeft, timer, bLLastError), 1), -1));

            // setting for next iteration
            bRLastError = bRError;
            fRLastError = fRError;
            fLLastError = fLError;
            bLLastError = bLError;
            timer.reset();

            telemetry.addData("Front Right Position", FRight.getCurrentPosition());
            telemetry.addData("Front Left Position", FLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", BRight.getCurrentPosition());
            telemetry.addData("Back Left Position", BLeft.getCurrentPosition());

            telemetry.addData("Front Right Error", fRError);
            telemetry.addData("Front Left Error", fLError);
            telemetry.addData("Back Right Error", bRError);
            telemetry.addData("Back Left Error", bLError);

            telemetry.update();
        }

        // stopping motors
        FRight.setPower(0);
        FLeft.setPower(0);
        BRight.setPower(0);
        BLeft.setPower(0);
    }

    public double findPIDPower(int targetSteps, DcMotor motor, ElapsedTime timer, double lastError){
        double derivative = 0;
        double integralSum = 0;
        double out = 0;
        double error = 0;

        error = targetSteps - motor.getCurrentPosition();

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        out = (kP * error) + (kI * integralSum) + (kD * derivative);

        return out;
    }
}
