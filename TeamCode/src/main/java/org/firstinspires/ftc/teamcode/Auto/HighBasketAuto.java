package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "HighBasketAuto")
public class HighBasketAuto extends LinearOpMode {

    // Performance constants
    final int SLIDE_Y_MAX = 2400;
    final int SLIDE_X_MAX = 1000; // Maximum position (for intake)
    final int SLIDE_MIN = 0; // Minimum position (bottom)
    final double SLIDE_POWER = 0.5;
    final double MOTOR_POWER = 0.2;
    final int SHOULDER_MAX = 1470; // Maximum release
    final int SHOULDER_MIN = 0; // Minimum intake (top)
    final double SHOULDER_POWER = 0.5;
    final double WRIST_MAX = 0.6; // Intake position (top)
    final double WRIST_MIN = 0; // Release position (top)
    final double CLAW_MAX = 0.6;  // Open
    final double CLAW_MIN = 0;// Close

   // robot.leftClaw.setPosition(1.0); 
   // robot.rightClaw.setPosition(0.0);


    // position targets
    double shoulderTarget = 0;
    double wristTarget = 0;
    double clawTarget = 0;
    double slideTarget = 0;
    double motorTarget = 0;

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
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brakes
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // open claw
        clawPosition(CLAW_MIN);


        strafe(400);   // strafe left

        sleep(1000);  // wait till strafe is complete

        // these will happen at the same time
        movePosition(-720);  // move backward

        liftShoulder(SHOULDER_MAX); // lift shoulder to release position

        while (opModeIsActive() && (BLeft.isBusy() || BRight.isBusy() || FLeft.isBusy() || FRight.isBusy() || shoulder.isBusy())) {
            telemetry.addData("Driving / Lifting", "Driving forward and lifting arm concurrently");
            telemetry.addData("Motor Current / Target ", "(%d, %.2f)", FLeft.getCurrentPosition(), motorTarget);
            telemetry.update();
        }

        rotate(-380); // rotate to align with basket

        sleep(2000); // wait for rotate to complete

        extendSlide(SLIDE_Y_MAX); // extend the slide to basket position

        while (opModeIsActive() && (slide.isBusy())) {
            telemetry.addData("Extending / Releasing", "Slide, wrist, and claw actions in progress");
            telemetry.update();
        }

        wristPosition(WRIST_MIN); // flick claw back

        movePosition(-80); // move back to get above the basket

        sleep(2000); // waiat for the movement to com

        clawPosition(CLAW_MAX);  // open the claw to release the object

        movePosition(400); // move forward to leave the basket

        sleep(2000); //  wait until all the way out

        extendSlide(SLIDE_MIN); // bring back the slide

        sleep(4000); // wait untill the slide is all the way back

        liftShoulder(SHOULDER_MIN); // bring shoulder back to start position

        sleep(2000);

        telemetry.addData("Task Complete", "All movements finished.");
        telemetry.update();
    }

    public void extendSlide(double position){
        slideTarget = position;
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        telemetry.addData("Slide Current / Target ", "(%d, %.2f)", slide.getCurrentPosition(), slideTarget);
    }

    public void clawPosition(double position){
        clawTarget = position;
        claw.setPosition(clawTarget);
        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", -1.0, clawTarget);
    }

    public void movePosition(double encoderTicks) {
        motorTarget = encoderTicks;

        BLeft.setTargetPosition(BLeft.getCurrentPosition() + (int)motorTarget);
        BRight.setTargetPosition(BRight.getCurrentPosition() + (int)motorTarget);
        FLeft.setTargetPosition(FLeft.getCurrentPosition() + (int)motorTarget);
        FRight.setTargetPosition(FRight.getCurrentPosition() + (int)motorTarget);

        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLeft.setPower(Math.abs(MOTOR_POWER));
        BRight.setPower(Math.abs(MOTOR_POWER));
        FLeft.setPower(Math.abs(MOTOR_POWER));
        FRight.setPower(Math.abs(MOTOR_POWER));

        telemetry.addData("Motor Current / Target ", "(%d, %.2f)", FLeft.getCurrentPosition(), motorTarget);
    }

    public void rotate(double encoderTicks) {
        motorTarget = encoderTicks;

        BLeft.setTargetPosition(BLeft.getCurrentPosition() + (int)motorTarget);
        BRight.setTargetPosition(BRight.getCurrentPosition() + -(int)motorTarget);
        FLeft.setTargetPosition(FLeft.getCurrentPosition() + (int)motorTarget);
        FRight.setTargetPosition(FRight.getCurrentPosition() + -(int)motorTarget);

        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLeft.setPower(Math.abs(MOTOR_POWER));
        BRight.setPower(Math.abs(MOTOR_POWER));
        FLeft.setPower(Math.abs(MOTOR_POWER));
        FRight.setPower(Math.abs(MOTOR_POWER));


        telemetry.addData("Motor Current / Target ", "(%d, %.2f)", FLeft.getCurrentPosition(), motorTarget);
    }

    public void strafe(double encoderTicks) {
        motorTarget = encoderTicks;

        BLeft.setTargetPosition(BLeft.getCurrentPosition() + (int)motorTarget);
        BRight.setTargetPosition(BRight.getCurrentPosition() + -(int)motorTarget);
        FLeft.setTargetPosition(FLeft.getCurrentPosition() + -(int)motorTarget);
        FRight.setTargetPosition(FRight.getCurrentPosition() + (int)motorTarget);

        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLeft.setPower(Math.abs(MOTOR_POWER));
        BRight.setPower(Math.abs(MOTOR_POWER));
        FLeft.setPower(Math.abs(MOTOR_POWER));
        FRight.setPower(Math.abs(MOTOR_POWER));


        telemetry.addData("Motor Current / Target ", "(%d, %.2f)", FLeft.getCurrentPosition(), motorTarget);
    }

    public void liftShoulder(double position) {
        shoulderTarget = position;

        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));

        telemetry.addData("Shoulder Current / Target ", "(%d, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
    }

    public void wristPosition(double position){
        wristTarget = position;
        wrist.setPosition((int)wristTarget);
        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", -1.0, wristTarget);
    }

    public void stopMotors() {
        BLeft.setPower(0);
        BRight.setPower(0);
        FLeft.setPower(0);
        FRight.setPower(0);
        shoulder.setPower(0);
        slide.setPower(0);
    }


}
