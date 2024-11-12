package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="DriftCode")
public class DriftCode extends OpMode {

    // initalizing motors
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

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

    // creating gamepadInput
    public void gamepadInput(){

    }

    public void init(){
        // connect to hardware map
        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop() {
    }


    public void loop(){
        moveRobot();

        telemetry.update();
    }
}
