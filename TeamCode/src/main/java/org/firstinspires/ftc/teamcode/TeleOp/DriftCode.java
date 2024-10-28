package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="DriftCode")
public class DriftCode extends OpMode {

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor elevator;
    private DcMotor arm;

    public void moveRobot(){
        double vertical;
        double horizontal;
        double pivot;
        double speed = 0.75;

        vertical = speed * gamepad1.left_stick_y;
        horizontal = speed * -gamepad1.left_stick_x;
        pivot = speed * -gamepad1.right_stick_x;

        FRight.setPower((pivot + (-vertical + horizontal)));
        BRight.setPower(pivot + (-vertical - horizontal));
        FLeft.setPower((-pivot + (-vertical - horizontal)));
        BLeft.setPower((-pivot + (-vertical + horizontal)));
    }

    public void moveArm() {

        //TODO: Find these values with debugging
        double maxPosition = 100;
        double minPosition = 0;

        double speed = gamepad2.left_stick_y;

        if(arm.getCurrentPosition() <= maxPosition && speed > 0){
            arm.setPower(gamepad2.left_stick_y);
        }else if(arm.getCurrentPosition() >= minPosition && speed < 0){
            arm.setPower(gamepad2.left_stick_y);
        }else{
            arm.setPower(0);
        }
    }

    public void init(){
        // connect to hardware map
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // setting encoders
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();
        moveArm();

        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.update();
    }
}
