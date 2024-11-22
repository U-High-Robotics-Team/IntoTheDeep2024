package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class TestingPIDController extends LinearOpMode {

    DcMotorEx motor;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotorEx motor4;

    double kP = 0.003; // bigger the error the faster we will fix it
    double kI = 0.00006; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot
    double toleranceLevel = 1;
    double error;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "frontright");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, "frontleft");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor3 = hardwareMap.get(DcMotorEx.class, "backright");
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor4 = hardwareMap.get(DcMotorEx.class, "backleft");
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            movePosition(2000);
            telemetry.addData("Moving to Position", motor.getCurrentPosition());
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

        this.error = targetSteps - motor.getCurrentPosition();


        while(Math.abs(error) > toleranceLevel && opModeIsActive()){
            this.error = targetSteps - motor.getCurrentPosition();

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            out = (kP * error) + (kI * integralSum) + (kD * derivative);

            motor.setPower(Math.max(Math.min(out, 1), -1));
            motor2.setPower(Math.max(Math.min(out, 1), -1));
            motor3.setPower(Math.max(Math.min(out, 1), -1));
            motor4.setPower(Math.max(Math.min(out, 1), -1));



            // setting for next iteration
            lastError = error;
            timer.reset();

            telemetry.addData("Moving to Position", motor.getCurrentPosition());
            telemetry.addData("Current Error", error);
            telemetry.addData("Power", out);
            telemetry.update();
        }

        // stopping motors
        motor.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}