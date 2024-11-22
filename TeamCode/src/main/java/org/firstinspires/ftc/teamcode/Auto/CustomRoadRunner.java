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

//TODO: Notes for additions in projects
// 1. Current coordinate system is relative to the robot and not the field so we
// need to store x and y values and convert given and x and y into field coordinate system (trig)
// 2. Going to add PID Controllers to handle errors (first isolating system within TestingPIDController.java
// 3. Soon will be utilizing online player to see code run without robot in hand

@Disabled // using this so it won't cause errors when running other codes (hopefully works not tested)
@Autonomous(name = "CustomRoadRunner")
public class CustomRoadRunner extends LinearOpMode {

    // motors
    private DcMotor BLeft, BRight, FLeft, FRight;

    // constants
    private static final double WHEEL_RADIUS = 0.05;
    private static final double PI = Math.PI;
    private static final double REVS_PER_METER = 1 / (2 * PI * WHEEL_RADIUS);
    private static final double MAX_SPEED = 2; // meters per second
    private static final double TIMESTEP = 0.1; // seconds
    private static final double LENGTH_CONSTANT = 0.3380; // half-width + half-length in meters
    private static final double INV_WHEEL_RADIUS = 1 / WHEEL_RADIUS;
    private static final double STEPS_PER_RAD = 62.832;
    private static final double MAX_ROT_SPEED = 0.1; // radians per second
    private static final double POSITION_TOLERANCE = 0.05; // meters
    private static final double ANGLE_TOLERANCE = 0.1; // radians
    private static final double STEPS_PER_REV = 1440;
    private static final double STEPS_PER_METER = STEPS_PER_REV * REVS_PER_METER;
    private static final double AXLE_CONSTANT = 0.35;          //(width + height) / 2;



    // unused constants
//    double invWheelRadius = 19.685;
//    double axleConst = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        // setting encoders
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set brakes
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // move one meter at 45 degrees with speed 1 with no rotation
            polarGo(1.0, Math.PI / 4, 1.0, 0);
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

        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLeft.setTargetPosition((int)stepsFL);
        FRight.setTargetPosition((int)stepsFR);
        BLeft.setTargetPosition((int)stepsBL);
        BRight.setTargetPosition((int)stepsBR);

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLeft.setPower(Math.abs(speed));
        FRight.setPower(Math.abs(speed));
        BLeft.setPower(Math.abs(speed));
        BRight.setPower(Math.abs(speed));
    }



    public void polarGo(double r, double theta, double speed, double rot) {
        //TODO: replaced milis() with System.currentTimeMillis() but not sure if this is correct
        long timeStart = System.currentTimeMillis();

        // this loop helps find when we reach target position
        while (r > POSITION_TOLERANCE || Math.abs(rot) > ANGLE_TOLERANCE) {
            if (System.currentTimeMillis() - timeStart > TIMESTEP * 1000) {
                timeStart = System.currentTimeMillis();

                // recompute velocites for the next timestep
                double travelTime = r / speed; //(t=d/v)
                double xVelo = speed * Math.cos(theta);
                double yVelo = speed * Math.sin(theta);

                double rotVelo = speed; // worrying about rotation later
                wheelSpeeds(xVelo, yVelo, rotVelo);

                // update targets for the next timestep
                r -= speed * TIMESTEP; // x = vt
                rot -= rotVelo * TIMESTEP; // worrying about rotation later
                theta -= rotVelo * TIMESTEP;// theta = wt
            }
        }

        off();
    }

    private void stopMotors() {
        FLeft.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
        BRight.setPower(0);
    }

    private void wheelSpeeds(double xV, double yV, double rV) {
        // calculate angular velocity for each motor (physics)
        double wFL = ((xV + yV) - (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wFR = ((-xV + yV) + (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wBL = ((-xV + yV) - (AXLE_CONSTANT * rV)) * STEPS_PER_METER;
        double wBR = ((xV + yV) + (AXLE_CONSTANT * rV)) * STEPS_PER_METER;

        // set the velocities for each motor
        FLeft.setPower(wFL);
        FRight.setPower(wFR);
        BLeft.setPower(wBL);
        BRight.setPower(wBR);
    }


    public void moveX(double meters) {
        double moveTo = meters * STEPS_PER_METER;

        FLeft.setTargetPosition((int)-moveTo);
        FRight.setTargetPosition((int)moveTo);
        BLeft.setTargetPosition((int)-moveTo);
        BRight.setTargetPosition((int)moveTo);

        go();
    }


    public void spin(double degrees) {
        double moveTo = degrees * 7.291666;  //2625 for 360

        FLeft.setTargetPosition((int)moveTo);
        FRight.setTargetPosition((int)moveTo);
        BLeft.setTargetPosition((int)moveTo);
        BRight.setTargetPosition((int)moveTo);

        go();
    }

    public void moveY(double meters) {
        double moveTo = meters * STEPS_PER_METER;

        FLeft.setTargetPosition((int)-moveTo);
        FRight.setTargetPosition((int)-moveTo);
        BLeft.setTargetPosition((int)moveTo);
        BRight.setTargetPosition((int)moveTo);

        go();
    }

    public void go() {
        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLeft.setPower(1);
        FRight.setPower(1);
        BLeft.setPower(1);
        BRight.setPower(1);

        // wait for the motors to be finished
        while (opModeIsActive() && (FLeft.isBusy() || FRight.isBusy() || BLeft.isBusy() || BRight.isBusy())) {
        }

        stopMotors();
    }

    public void off() {
        FLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
    }
}

