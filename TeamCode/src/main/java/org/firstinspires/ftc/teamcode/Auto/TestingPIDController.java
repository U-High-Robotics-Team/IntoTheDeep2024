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

    double kP = 0.003; // bigger the error the faster we will fix it
    double kI = 0.00006; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot
    double toleranceLevel = 1;
    double error;

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
            telemetry.addData("Current Error", error);
            telemetry.update();
        }
    }

    public void movePosition(int targetSteps){
        double lastError = 0;
        double derivative = 0;
        double integralSum = 0;
        double out = 0;
        ElapsedTime timer = new ElapsedTime();

        this.error = targetSteps - fRight.getCurrentPosition();


        while(Math.abs(error) > toleranceLevel && opModeIsActive()){
            this.error = targetSteps - fRight.getCurrentPosition();

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (kP * error) + (kI * integralSum) + (kD * derivative);

            fRight.setPower(Math.max(Math.min(out, 1), -1));
            fLeft.setPower(Math.max(Math.min(out, 1), -1));
            bRight.setPower(Math.max(Math.min(out, 1), -1));
            bLeft.setPower(Math.max(Math.min(out, 1), -1));



            // setting for next iteration
            lastError = error;
            timer.reset();

            telemetry.addData("Moving to Position", fRight.getCurrentPosition());
            telemetry.addData("Current Error", error);
            telemetry.addData("Power", out);
            telemetry.update();
        }

        // stopping motors
        fRight.setPower(0);
        fLeft.setPower(0);
        bRight.setPower(0);
        bLeft.setPower(0);
    }
}