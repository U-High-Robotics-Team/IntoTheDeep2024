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


@Disabled // using this so it won't cause errors when running other codes (hopefully works not tested)
@Autonomous(name = "CustomRoadRunner")
public class CustomRoadRunner extends LinearOpMode {

    double STEPS_PER_REV = 400;
    double WHEEL_RAD = 0.05;
    double PI = Math.PI;
    double REVS_PER_METER = 1 / (2 * PI * WHEEL_RAD);
    double MAX_STEP_SPEED = 1500;

    double stepsPerRad = 62.832;
    double STEPS_PER_METER = STEPS_PER_REV / REVS_PER_METER;
    double invWheelRadius = 19.685;   // in meters
    double lengthCon = 0.3380;        //0.3975 half-width + half-length in meters
    double maxSpeed = 2;              //meters/sec
    double positionTolerance = 0.05;  //in meters
    double angleTolerance = 0.1;      //radians
    double maxRotSpeed = 0.1;        //radians / second
    double axleConst = 0.35;          //(width + height) / 2;
    double wheelRadius = 0.05;
    double TIMESTEP = 0.1;

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo claw;

    // TODO: not sure what this does
    char input[64]="";
    int inputIdx = 0;

    // TODO: need to figure out what setMinPulseWith is and corresponding methods for each
    void setup() {

        int moveTo = 0;
        int minPulseWidth = 200;

        stepperFL.setMaxSpeed(MAX_STEP_SPEED);
        // stepperFL.setAcceleration(accel);
        stepperFL.setMinPulseWidth(minPulseWidth);
        steppers.addStepper(stepperFL);

        stepperFR.setMaxSpeed(MAX_STEP_SPEED);
        // stepperFR.setAcceleration(accel);
        stepperFR.setMinPulseWidth(minPulseWidth);
        steppers.addStepper(stepperFR);

        stepperBL.setMaxSpeed(MAX_STEP_SPEED);
        // stepperBL.setAcceleration(accel);
        stepperBL.setMinPulseWidth(minPulseWidth);
        steppers.addStepper(stepperBL);


        stepperBR.setMaxSpeed(MAX_STEP_SPEED);
        // stepperBR.setAcceleration(accel);
        stepperBR.setMinPulseWidth(minPulseWidth);
        steppers.addStepper(stepperBR);

        //polarGo(3, 0, 0.5, -PI/2);
    }


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
    }

    void roboGo(double x, double y, double deg, double speed) {

        double velo = 2;  // arbitrary - just for calculating steps

        double dist = Math.sqrt((x * x) + (y * y));

        double time = dist / velo;

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

        // Serial.println(x / dist) ;


        //what type of var

        double wFL = invWheelRadius * ((xVelo + yVelo) - (lengthCon * rotVelo));
        double wFR = invWheelRadius * ((-xVelo + yVelo) + (lengthCon * rotVelo));
        double wBL = invWheelRadius * ((-xVelo + yVelo) - (lengthCon * rotVelo));
        double wBR = invWheelRadius * ((xVelo + yVelo) + (lengthCon * rotVelo));

        double stepSpeedFL = wFL * stepsPerRad;
        double stepSpeedFR = wFR * stepsPerRad;
        double stepSpeedBL = wBL * stepsPerRad;
        double stepSpeedBR = wBR * stepsPerRad;

        // instead of stuff below, set wheel speeds in multistepper and runSpeed

        double stepsFL = (long) (stepSpeedFL * time);
        double stepsFR = (long) (stepSpeedFR * time);
        double stepsBL = (long) (stepSpeedBL * time);
        double stepsBR = (long) (stepSpeedBR * time);

        // TODO: move vs moveto?
        stepperFR.move(stepsFR);
        stepperFL.move(-stepsFL);
        stepperBR.move(stepsBR);
        stepperBL.move(-stepsBL);


        // looks like this just resets positions?
//        stepperFR.setCurrentPosition(0);
//        stepperFL.setCurrentPosition(0);
//        stepperBR.setCurrentPosition(0);
//        stepperBL.setCurrentPosition(0);

        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // TODO: need to understand this block of code
        long positions[ 4];
        positions[0] = -stepsFL;
        positions[1] = stepsFR;
        positions[2] = -stepsBL;
        positions[3] = stepsBR;
        steppers.moveTo(positions);

    }


    void polarGo(double r, double theta, double speed, double rot) {

        if (speed <= 0) {

            speed = Math.min(speed, maxSpeed);

        }

        //TODO: what is millis
        long timeStart = millis();

        while (r > positionTolerance || Math.abs(rot) > angleTolerance) {


            if (millis() - timeStart > TIMESTEP * 1000) {
                timeStart = millis();

                // recompute velocites for the next timestep

                double travelTime = r / speed;
                double xVelo = speed * Math.cos(theta);
                double yVelo = speed * Math.sin(theta);

                //double rotVelo = max(-maxRotSpeed, (min(maxRotSpeed, rot / (r / speed))));

                double rotVelo = speed;
                wheelSpeeds(xVelo, yVelo, rotVelo);

                // update targets for the next timestep

                r -= speed * TIMESTEP;
                rot -= rotVelo * TIMESTEP;
                theta -= rotVelo * TIMESTEP;
            }

            stepperFL.runSpeed();
            stepperFR.runSpeed();
            stepperBL.runSpeed();
            stepperBR.runSpeed();
        }
    }

    void wheelSpeeds(double xV, double yV, double rV) {
//        stepperFL.setSpeed(-((xV - yV - axleConst * rV) * STEPS_PER_METER));  // +-- fl
//        stepperFR.setSpeed((xV + yV + axleConst * rV) * STEPS_PER_METER);     // +++ fr
//        stepperBL.setSpeed(-((xV + yV - axleConst * rV) * STEPS_PER_METER));  // ++-; bl
//        stepperBR.setSpeed((xV - yV + axleConst * rV) * STEPS_PER_METER);     // +-+; br
//
        FLeft.setPower(-((xV - yV - axleConst * rV) * STEPS_PER_METER));
        FRight.setPower((xV + yV + axleConst * rV) * STEPS_PER_METER);
        BLeft.setPower(-((xV + yV - axleConst * rV) * STEPS_PER_METER));
        BRight.setPower((xV - yV + axleConst * rV) * STEPS_PER_METER);
    }

    // TODO: understand what is serial
    void processSerialIn() {

        Serial.println("Data received:");

        double r = Serial.parsedouble();
        double th = Serial.parsedouble();
        double spd = Serial.parsedouble();
        double rot = Serial.parsedouble();

        Serial.readStringUntil('\n');

        if (spd != 0 && (r != 0 || rot != 0)) {
            Serial.println("Command sent.");

            polarGo(r, th, spd, rot);
        }

    }

    void moveX(double meters) {
        double moveTo = meters * 1250;
//
//        stepperFL.moveTo(-moveTo);
//        stepperFR.moveTo(moveTo);
//        stepperBL.moveTo(-moveTo);
//        stepperBR.moveTo(moveTo);

        FLeft.setTargetPosition((int)-moveTo);
        FRight.setTargetPosition((int)moveTo);
        BLeft.setTargetPosition((int)-moveTo);
        BRight.setTargetPosition((int)moveTo);

        go();
    }


    void spin(double degrees) {
        double moveTo = degrees * 7.291666;  //2625 for 360

//        stepperFL.moveTo(moveTo);
//        stepperFR.moveTo(moveTo);
//        stepperBL.moveTo(moveTo);
//        stepperBR.moveTo(moveTo);

        FLeft.setTargetPosition((int)moveTo);
        FRight.setTargetPosition((int)moveTo);
        BLeft.setTargetPosition((int)moveTo);
        BRight.setTargetPosition((int)moveTo);
        go();
    }

    void moveY(double meters) {
        double moveTo = meters * 1250;

//        stepperFL.moveTo(-moveTo);
//        stepperFR.moveTo(-moveTo);
//        stepperBL.moveTo(moveTo);
//        stepperBR.moveTo(moveTo);

        FLeft.setTargetPosition((int)-moveTo);
        FRight.setTargetPosition((int)-moveTo);
        BLeft.setTargetPosition((int)moveTo);
        BRight.setTargetPosition((int)moveTo);

        go();
    }


    void go() {
//        stepperFL.run();
//        stepperBR.run();
//        stepperBL.run();
//        stepperFR.run();

        FLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void off() {
//        stepperFL.setSpeed(0);
//        stepperFR.setSpeed(0);
//        stepperBL.setSpeed(0);
//        steeperBR.setSpeed(0);

        FLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
    }
}

