package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.StateMachineHighBasket;


@TeleOp(name = "State Machine TeleOp V2")
public class StateMachineTeleopV2 extends OpMode {

    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    // Preset action states
    enum RobotState {
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
    final double CLAW_CLOSED = 0;
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;

    // Threshold where speed is reduced when slide is extended
    final double SLIDE_POSITION_THRESHOLD = 700;
    // Threshold where slide estension is limited with should down to stay within size limits
    final double SHOULDER_POSITION_THRESHOLD = 500;

    // set initial position ...
    double shoulderTarget = SHOULDER_MIN;
    double wristTarget = WRIST_UP;
    double clawTarget = CLAW_CLOSED;
    double slideTarget = SLIDE_MIN;
    double wheelSpeed = WHEEL_SPEED_MAX;
    // ... then set current state to match above position
    RobotState currentState = RobotState.HOME;
    RobotState requestedState = RobotState.HOME;

    // Declare motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;

    public void moveRobot() {
        // input sensitivity is linear right now - this can be adjusted
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        if(slide.getCurrentPosition()>SLIDE_POSITION_THRESHOLD){
            wheelSpeed = WHEEL_SPEED_LIMITED;
        }else{
            wheelSpeed = WHEEL_SPEED_MAX;
        }

        double vertical = wheelSpeed * leftStickY;
        double horizontal = wheelSpeed * -leftStickX;
        double pivot = wheelSpeed * -rightStickX;

        // mecanum wheel power calculations
        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void gamepadInput(){
        // Preset States
        if (gamepad2.right_bumper){
            requestedState = RobotState.HOME;
        }
        if (gamepad2.right_trigger > 0.5){
            requestedState = RobotState.SUB_1;
        }
        if (gamepad2.left_bumper){
            requestedState = RobotState.BASKET_1;
        }
        if (gamepad2.left_trigger > 0.5){
            requestedState = RobotState.BASKET_2;
        }
        if(gamepad2.a){
            // Preset to grab block
            if(currentState == RobotState.SUB_1){
                requestedState = RobotState.SUB_2;
            }else{
                // clawTarget = WRIST_DOWN;
            }
        }
        if(gamepad2.x){
            // Preset to return to home
            if(currentState == RobotState.BASKET_2){
                requestedState = RobotState.BASKET_3; // Same as home but with systematic process (ordering movements)
            } else {
                //clawTarget = CLAW_OPEN;
            }
        }
        /*
        NOTE : manual modes have been removed (commented out) from this version of the OpMode
        if(gamepad2.y){
            wristTarget = WRIST_UP;
        }
        if(gamepad2.b){
            clawTarget = CLAW_CLOSED;
        }


        if(gamepad2.left_bumper && slide.getCurrentPosition() < SHOULDER_POSITION_THRESHOLD) {
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
        */

    }

    public void stateMachine() {
        switch (currentState) {
            case HOME:
                // immediate actions
                clawTarget = CLAW_CLOSED;

                if(timer.seconds() > 0.3){
                    wristTarget = WRIST_UP;
                }
                // delayed actions
                if (timer.seconds() > 0.4) {
                    shoulderTarget = SHOULDER_MIN;
                    slideTarget = SLIDE_MIN;
                }
                // allowed transistions from HOME: SUBMERSIBLE, BASKET_1
                if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.BASKET_1){
                    currentState = RobotState.BASKET_1;
                    timer.reset();  // start delay timer for wrist movement
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
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_2){
                    currentState = RobotState.SUB_2;
                    timer.reset();  // start delay timer for wrist movement
                }
                break;

            case SUB_2:
                // immediate actions
                wristTarget = WRIST_DOWN;
                shoulderTarget = SHOULDER_MIN;
                slideTarget = SLIDE_X_MAX;
                // delayed actoins
                if(timer.seconds() > 0.4){
                    clawTarget = CLAW_CLOSED;
                }
                if(timer.seconds() > 0.8){
                    currentState = RobotState.SUB_3;
                    timer.reset();
                }
                // allowed transition
                break;


            case SUB_3:
                // immediate actions
                wristTarget = WRIST_UP;
                // allowed transition
                if (requestedState == RobotState.HOME){
                    currentState = RobotState.HOME;
                    timer.reset();  // start delay timer for wrist movement
                } else if (requestedState == RobotState.SUB_1){
                    currentState = RobotState.SUB_1;
                    timer.reset();  // start delay timer for wrist movement
                }
                break;

            case BASKET_1:
                // immediate actions
                shoulderTarget = SHOULDER_MAX;
                wristTarget = WRIST_DOWN;
                // delayed actions
                // allowed transistions
                if (timer.seconds() > 1.5) {
                    currentState = RobotState.BASKET_2;
                    timer.reset();
                }
                break;

            case BASKET_2:
                slideTarget = SLIDE_Y_MAX;
                clawTarget = CLAW_CLOSED;

                if(timer.seconds() > 1.4){
                    wristTarget = WRIST_UP;
                }

                if(requestedState == RobotState.BASKET_3){
                    currentState = RobotState.BASKET_3;
                    timer.reset();
                }
                break;

            case BASKET_3:
                // immediate actions
                clawTarget = CLAW_OPEN;

                if(timer.seconds()> 1.0){
                    currentState = RobotState.BASKET_4;
                    timer.reset();
                }
                break;

            case BASKET_4:
                wristTarget = WRIST_DOWN;

                if(timer.seconds()>1.0){
                    slideTarget = SLIDE_MIN;
                    shoulderTarget = SHOULDER_MAX;
                }

                if(timer.seconds()>2.0){
                    currentState = RobotState.HOME;
                    timer.reset();
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

        // set encoders
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
        moveWrist();
        moveClaw();
        telemetry.update();
    }
}