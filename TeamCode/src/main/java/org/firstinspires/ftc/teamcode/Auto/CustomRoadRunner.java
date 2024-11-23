package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//TODO: Notes for additions in projects
// 1. Current coordinate system is relative to the robot and not the field so we
// need to store x and y values and convert given and x and y into field coordinate system (trig)
// 2. Going to add PID Controllers to handle errors (first isolating system within TestingPIDController.java
// 3. Soon will be utilizing online player to see code run without robot in hand

@Autonomous(name = "CustomRoadRunner")
public class CustomRoadRunner extends LinearOpMode {

    // motors
    DcMotorEx fRight;
    DcMotorEx fLeft;
    DcMotorEx bRight;
    DcMotorEx bLeft;

    double kP = 0.003; // bigger the error the faster we will fix it
    double kI = 0.0001; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot
    double toleranceLevel = 5;
    double bRError;
    double bLError;
    double fRError;
    double fLError;

    // constants
    private static final double WHEEL_RADIUS = 0.104;
    private static final double PI = Math.PI;
    private static final double REVS_PER_METER = 1 / (2 * PI * WHEEL_RADIUS);
    private static final double MAX_SPEED = 2; // meters per second
    private static final double TIMESTEP = 0.1; // seconds
    private static final double LENGTH_CONSTANT = 0.3380; // half-width + half-length in meters
    private static final double INV_WHEEL_RADIUS = 1 / WHEEL_RADIUS;
    private static final double STEPS_PER_REV = 751.8;
    private static final double STEPS_PER_RAD = STEPS_PER_REV/(2*PI);
    private static final double MAX_ROT_SPEED = 0.1; // radians per second
    private static final double POSITION_TOLERANCE = 0.05; // meters
    private static final double ANGLE_TOLERANCE = 0.1; // radians
    private static final double STEPS_PER_METER = STEPS_PER_REV * REVS_PER_METER;
    private static final double AXLE_CONSTANT = 0.35;          //(width + height) / 2;



    // unused constants
//    double invWheelRadius = 19.685;
//    double axleConst = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        bLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        bRight = hardwareMap.get(DcMotorEx.class, "backright");
        fLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        fRight = hardwareMap.get(DcMotorEx.class, "frontright");

        // setting encoders
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // reverse the motor directions
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // move one meter at 45 degrees with speed 1 with no rotation
            telemetry.addData("Status: Begin Path", 1);
            telemetry.update();
            roboGo(1.0, 0, 0, 0.2);
            telemetry.addData("Status: Completed Path", 0);
            telemetry.update();
        }
    }

    public void roboGo(double x, double y, double deg, double speed) {
        double velo = 2;  // arbitrary - just for calculating steps
        double dist = Math.sqrt((x * x) + (y * y)); // finds vector distance (pythagorean theorem)
        double time = dist / velo; // physics (t=d/v)
        double xVelo;
        double yVelo;

        if (dist == 0) {
            time = velo;
            xVelo = 0;
            yVelo = 0;
        } else {
            xVelo = (x / dist) * velo;
            yVelo = (y / dist) * velo;

        }

        telemetry.addData("X Velocity", xVelo);
        telemetry.addData("Y Velocity", yVelo);
        telemetry.update();

        sleep(5000);


        double rotVelo = (deg / time) * (PI / 180);

        // angular velocity (physics)
        // w = 1/r * (xvelo + yvelo - (distance from center * rotational velocity)
        // unit = rad/sec
        double wFL = WHEEL_RADIUS * ((xVelo + yVelo) - (LENGTH_CONSTANT * rotVelo));
        double wFR = WHEEL_RADIUS * ((-xVelo + yVelo) + (LENGTH_CONSTANT * rotVelo));
        double wBL = WHEEL_RADIUS * ((-xVelo + yVelo) - (LENGTH_CONSTANT * rotVelo));
        double wBR = WHEEL_RADIUS * ((xVelo + yVelo) + (LENGTH_CONSTANT * rotVelo));

        // finding the number of steps to satisfy the number of radians of rotations
        // units = (rad/sec * steps/rad) = steps/sec
        double stepSpeedFL = wFL * STEPS_PER_RAD;
        double stepSpeedFR = wFR * STEPS_PER_RAD;
        double stepSpeedBL = wBL * STEPS_PER_RAD;
        double stepSpeedBR = wBR * STEPS_PER_RAD;

        // units = (sec * steps/sec) = steps
        // calculating the final steps needed to get desire angular velocity
        double stepsFL = (long) (stepSpeedFL * time);
        double stepsFR = (long) (stepSpeedFR * time);
        double stepsBL = (long) (stepSpeedBL * time);
        double stepsBR = (long) (stepSpeedBR * time);

        telemetry.addData("Front Left Target Steps", stepsFL);
        telemetry.addData("Front Right Target Steps", stepsFR);
        telemetry.addData("Back Left Target Steps", stepsBL);
        telemetry.addData("Back Right Target Steps", stepsBR);
        telemetry.update();
        sleep(5000);

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()){
            movePosition((int)stepsFR, (int)stepsFL, (int)stepsBR, (int)stepsBL);
        }

    }


    public void movePosition(int fRTargetSteps, int fLTargetSteps, int bRTargetSteps, int bLTargetSteps){
        double bRLastError = 0;
        double fRLastError = 0;
        double fLLastError = 0;
        double bLLastError = 0;
        ElapsedTime timer = new ElapsedTime();

        this.fRError = fRTargetSteps - fRight.getCurrentPosition();
        this.fLError = fLTargetSteps - fLeft.getCurrentPosition();
        this.bRError = bRTargetSteps - bRight.getCurrentPosition();
        this.bLError = bLTargetSteps - bLeft.getCurrentPosition();

        while(Math.abs(fRError) > toleranceLevel && opModeIsActive() && Math.abs(fLError) > toleranceLevel && Math.abs(bRError) > toleranceLevel && Math.abs(bLError) > toleranceLevel){

            this.fRError = fRTargetSteps - fRight.getCurrentPosition();
            this.fLError = fLTargetSteps - fLeft.getCurrentPosition();
            this.bRError = bRTargetSteps - bRight.getCurrentPosition();
            this.bLError = bLTargetSteps - bLeft.getCurrentPosition();

            fRight.setPower(Math.max(Math.min(findPIDPower(fRTargetSteps, fRight, timer, fRLastError), 1), -1));
            fLeft.setPower(Math.max(Math.min(findPIDPower(fLTargetSteps, fLeft, timer, fLLastError), 1), -1));
            bRight.setPower(Math.max(Math.min(findPIDPower(bRTargetSteps, bRight, timer, bRLastError), 1), -1));
            bLeft.setPower(Math.max(Math.min(findPIDPower(bLTargetSteps, bLeft, timer, bLLastError), 1), -1));

            // setting for next iteration
            bRLastError = bRError;
            fRLastError = fRError;
            fLLastError = fLError;
            bLLastError = bLError;
            timer.reset();

            telemetry.addData("Front Right Position", fRight.getCurrentPosition());
            telemetry.addData("Front Left Position", fLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", bRight.getCurrentPosition());
            telemetry.addData("Back Left Position", bLeft.getCurrentPosition());

            telemetry.addData("Front Right Power", Math.max(Math.min(findPIDPower(fRTargetSteps, fRight, timer, fRLastError), 1), -1));

            telemetry.addData("Front Right Error", fRError);
            telemetry.addData("Front Left Error", fLError);
            telemetry.addData("Back Right Error", bRError);
            telemetry.addData("Back Left Error", bLError);

            telemetry.update();
        }

        // stopping motors
        fRight.setPower(0);
        fLeft.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
    }

    public double findPIDPower(int targetSteps, DcMotorEx motor, ElapsedTime timer, double lastError){
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

    private void stopMotors() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    private void wheelSpeeds(double xV, double yV, double rV) {
        // calculate angular velocity for each motor (physics)
        double wFL = ((xV + yV) - (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wFR = ((-xV + yV) + (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wBL = ((-xV + yV) - (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wBR = ((xV + yV) + (AXLE_CONSTANT * rV)) * STEPS_PER_METER;

        // set the velocities for each motor
        fLeft.setPower(wFL);
        fRight.setPower(wFR);
        bLeft.setPower(wBL);
        bRight.setPower(wBR);
    }


    public void moveX(double meters) {
        double moveTo = meters * STEPS_PER_METER;

        fLeft.setTargetPosition((int)-moveTo);
        fRight.setTargetPosition((int)moveTo);
        bLeft.setTargetPosition((int)-moveTo);
        bRight.setTargetPosition((int)moveTo);

        go();
    }


    public void spin(double degrees) {
        double moveTo = degrees * 7.291666;  //2625 for 360

        fLeft.setTargetPosition((int)moveTo);
        fRight.setTargetPosition((int)moveTo);
        bLeft.setTargetPosition((int)moveTo);
        bRight.setTargetPosition((int)moveTo);

        go();
    }

    public void moveY(double meters) {
        double moveTo = meters * STEPS_PER_METER;

        fLeft.setTargetPosition((int)-moveTo);
        fRight.setTargetPosition((int)-moveTo);
        bLeft.setTargetPosition((int)moveTo);
        bRight.setTargetPosition((int)moveTo);

        go();
    }

    public void go() {
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(1);
        fRight.setPower(1);
        bLeft.setPower(1);
        bRight.setPower(1);

        // wait for the motors to be finished
        while(opModeIsActive() && (fLeft.isBusy() || fRight.isBusy() || bLeft.isBusy() || bRight.isBusy())) {
        }

        stopMotors();
    }

    public void off() {
        fLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
    }
}

