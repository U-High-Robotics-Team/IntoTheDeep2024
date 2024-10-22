package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "EncoderCode")
public class EncoderCode extends LinearOpMode {

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    // Encoder counts per revolution
    private static final int COUNTS_PER_REVOLUTION = 1440; // Adjust based on your motor
    private static final double WHEEL_DIAMETER = 4.0; // In inches
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() {
        // Initialize the motors
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");



        // reset encoders and start

        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // Move forward for 24 inches
        driveForward(12);
    }

    private void driveForward(double inches) {
        int targetPosition = (int)(inches * COUNTS_PER_INCH);

        BLeft.setTargetPosition(targetPosition);
        BRight.setTargetPosition(targetPosition);
        BLeft.setTargetPosition(targetPosition);
        BRight.setTargetPosition(targetPosition);

        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLeft.setTargetPosition(targetPosition);
        BRight.setTargetPosition(targetPosition);
        BLeft.setTargetPosition(targetPosition);
        BRight.setTargetPosition(targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(1.0); // Full power
        rightDrive.setPower(1.0); // Full power

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Reset motor modes for future movements
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
