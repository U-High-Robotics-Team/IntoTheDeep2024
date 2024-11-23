package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class TestingPIDController extends LinearOpMode {

    DcMotorEx fRight;
    DcMotorEx fLeft;
    DcMotorEx bRight;
    DcMotorEx bLeft;

    double kP = 0.0024; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot
    double toleranceLevel = 1;
    double bRError;
    double bLError;
    double fRError;
    double fLError;

    @Override
    public void runOpMode() throws InterruptedException {
        fRight = hardwareMap.get(DcMotorEx.class, "frontright");
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bRight = hardwareMap.get(DcMotorEx.class, "backright");
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            movePosition(2000);
            telemetry.addData("Moving to Position", fRight.getCurrentPosition());
            telemetry.addData("Current Error", 0);
            telemetry.update();
        }
    }

    public void movePosition(int targetSteps){
        double bRLastError = 0;
        double fRLastError = 0;
        double fLLastError = 0;
        double bLLastError = 0;
        ElapsedTime timer = new ElapsedTime();

        this.fRError = targetSteps - fRight.getCurrentPosition();
        this.fLError = targetSteps - fLeft.getCurrentPosition();
        this.bRError = targetSteps - bRight.getCurrentPosition();
        this.bLError = targetSteps - bLeft.getCurrentPosition();

        while(Math.abs(fRError) > toleranceLevel && opModeIsActive() && Math.abs(fLError) > toleranceLevel && Math.abs(bRError) > toleranceLevel && Math.abs(bLError) > toleranceLevel){

            this.fRError = targetSteps - fRight.getCurrentPosition();
            this.fLError = targetSteps - fLeft.getCurrentPosition();
            this.bRError = targetSteps - bRight.getCurrentPosition();
            this.bLError = targetSteps - bLeft.getCurrentPosition();

            fRight.setPower(Math.max(Math.min(findPIDPower(targetSteps, fRight, timer, fRLastError), 1), -1));
            fLeft.setPower(Math.max(Math.min(findPIDPower(targetSteps, fLeft, timer, fLLastError), 1), -1));
            bRight.setPower(Math.max(Math.min(findPIDPower(targetSteps, bRight, timer, bRLastError), 1), -1));
            bLeft.setPower(Math.max(Math.min(findPIDPower(targetSteps, bLeft, timer, bLLastError), 1), -1));

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
}