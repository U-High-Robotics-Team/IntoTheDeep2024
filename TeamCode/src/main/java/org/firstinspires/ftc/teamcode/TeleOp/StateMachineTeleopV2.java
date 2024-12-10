package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "State Machine TeleOp V2")
public class StateMachineTeleopV2 extends OpMode {

    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    // Preset action states
    enum RobotState {
        NONE,
        HOME_STATE,
        SUBMERSIBLE_STATE,
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
    final double WRIST_UP = 0.6;
    final double WRIST_DOWN = 0.9;
    final double WRIST_CLIP = 0.3; // unused currently
    final double CLAW_OPEN = 0.6;
    final double CLAW_CLOSED = 0;
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMTED = 0.2;
    
    // Threshold where extended slide causes drive mode to be slower
    final double SLIDE_POSITION_THRESHOLD = 1900;
    // Threshold where lowered should limits maximum slide extension to comply with size rules
    final double SHOULDER_POSITION_THRESHOLD = 500;
    
    // Position targets
    private RobotState currentState = RobotState.HOME;
    private RobotState requestedState = RobotState.NONE;
    // TODO: ensure that robot is physically in the HOME state configuration
    double shoulderTarget = SHOULDER_MIN;
    double wristTarget = WRIST_UP;
    double clawTarget = CLAW_CLOSED;
    double slideTarget = SLIDE_MIN;

    // Movement speed
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

    public void moveRobot() {
        // response curve is set to linear - can be adjusted for more or less sensitivity
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        if(slide.getCurrentPosition()>SLIDE_POSITION_THRESHOLD){
            this.wheelSpeed = WHEEL_SPEED_LIMTED;
        }else{
            this.wheelSpeed = WHEEL_SPEED_MAX;
        }
        
        double vertical = wheelSpeed * leftStickY;
        double horizontal = wheelSpeed * -leftStickX; 
        double pivot = wheelSpeed * -rightStickX;
        
        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void gamepadInput(){
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

        if(gamepad2.left_bumper && slide.getCurrentPosition() < SHOULDER_POSITION_THRESHOLD){
            shoulderTarget = SHOULDER_MIN;
        }

        if(gamepad2.right_bumper && slide.getCurrentPosition() < SHOULDER_POSITION_THRESHOLD) {
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

            if(shoulder.getCurrentPosition() > SHOULDER_POSITION_THRESHOLD){
                max = SLIDE_Y_MAX;
            }else{
                max = SLIDE_X_MAX;
            }
            slideTarget = Math.max(SLIDE_MIN, Math.min(max, slide.getCurrentPosition() + input));
        }

        if(currentState == RobotState.NONE) { // makes sure no other movements are happening and no conflicting presets

            // PRESET: Grabbed sample and retracting
            if (gamepad2.right_trigger > 0.5 && shoulder.getCurrentPosition() < SHOULDER_POSITION_THRESHOLD) {
                timer.reset();
                clawTarget = CLAW_CLOSED;
                wristTarget = WRIST_UP;

                currentState = ServoState.CLAW_ACTION; // jumps into state machine

                slideTarget = SLIDE_MIN;
//
//                currentState = ServoState.CLAW_ACTION; // jumps into state machine
            }

            // PRESET: Releasing sample in basket and retracting
            if (gamepad2.right_trigger > 0.5 && shoulder.getCurrentPosition() > SHOULDER_POSITION_THRESHOLD) {
                timer.reset();
                clawTarget = CLAW_OPEN;
                wristTarget = WRIST_DOWN;
                slideTarget = SLIDE_MIN;

                currentState = ServoState.CLAW_ACTION; // jumps into state machine
            }

            // PRESET: Extends slide into Submersible for Sample pickup
            if (gamepad2.left_trigger > 0.5 && shoulder.getCurrentPosition() < SHOULDER_POSITION_THRESHOLD) {
                timer.reset();
                clawTarget = CLAW_OPEN;
                wristTarget = WRIST_DOWN;
                slideTarget = SLIDE_X_MAX;
                shoulderTarget = SHOULDER_MIN;

                currentState = ServoState.CLAW_ACTION; // jumps into state machine
            }

            // PRESET: Extending slide into basket with wrist over basket
            if (gamepad2.left_trigger > 0.5 && shoulder.getCurrentPosition() > SHOULDER_POSITION_THRESHOLD) {
                timer.reset();
                shoulderTarget = SHOULDER_MAX;
                slideTarget = SLIDE_Y_MAX;
                wristTarget = WRIST_UP;
                clawTarget = CLAW_CLOSED;

                currentState = ServoState.CLAW_ACTION;  // jumps into state machine
            }
        }
    }

    public void stateMachine() {
        telemetry.addData("State Machine", "Current state: %s", currentState);
        switch (currentState) {
            case CLAW_ACTION:
                // telemetry.addData("State Machine", "Claw is moving");
                moveClaw(); // move claw to the target
                if (timer.seconds() > 0.5) { // TODO find appropriate time for claw action
                    timer.reset();
                    currentState = ServoState.WRIST_ACTION; // jump to next state
                }
                break;

            case WRIST_ACTION:
                // telemetry.addData("State Machine", "Wrist is moving");
                moveWrist(); // move wrist to the target
                if (timer.seconds() > 1.0) { // TODO find appropriate time for wrist action
                    timer.reset();
                    currentState = ServoState.SLIDE_ACTION; // jump to next state
                }
                break;

            case SLIDE_ACTION:
                moveSlide();
                currentState = ServoState.COMPLETED;
                break;



            case COMPLETED:
                // telemetry.addData("State Machine", "Preset actions are complete");
                currentState = ServoState.NONE; // reset state for the next preset
                break;

            case NONE:
                // telemetry.addData("State Machine", "No presets currently");
            default:
                break;
        }
    }

    public void moveWrist() {
        wrist.setPosition(wristTarget);
        // telemetry.addData("Wrist Target ", "(%.2f)", wristTarget);
    }

    public void moveClaw() {
        claw.setPosition(clawTarget);
        // telemetry.addData("Claw Target ", "(%.2f)", clawTarget);
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

    public void loop() {
        gamepadInput();
        stateMachine();
        moveRobot();
        moveShoulder();
        moveSlide();
        moveClaw();
        telemetry.update();
    }
}