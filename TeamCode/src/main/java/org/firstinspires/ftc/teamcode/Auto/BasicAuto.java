package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name="BasicAuto")
public class BasicAuto extends LinearOpMode {

    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor BRight;
    private DcMotor BLeft;

    @Override
    public void runOpMode () {
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        // reverse the motors
        BRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        // this is what runs on the robot
        while (opModeIsActive()) {

            // goes straight at 1 speed for 1000 milliseconds(1 second)
            runMotorsTime(1.0, 1000);

        }
    }

    public void brakeMotors(){
        BLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotorsTime(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(power * 0.95);
        FRight.setPower(power * 0.95);
        FLeft.setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void strafeMotorsRight(double power, long motorTime) {
        BLeft.setPower(-power);
        BRight.setPower(power);
        FRight.setPower(-power);
        FLeft.setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void strafeMotorsLeft(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(-power);
        FRight.setPower(power);
        FLeft.setPower(-power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }

    public void turnRight(double power, long motorTime) {
        BLeft.setPower(power);
        BRight.setPower(-power);
        FRight.setPower(-power);
        FLeft .setPower(power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);

    }

    public void turnLeft(double power, long motorTime) {
        BLeft.setPower(-power);
        BRight.setPower(power);
        FRight.setPower(power);
        FLeft.setPower(-power);

        sleep(motorTime);

        BLeft.setPower(0);
        BRight.setPower(0);
        FRight.setPower(0);
        FLeft.setPower(0);
    }
}