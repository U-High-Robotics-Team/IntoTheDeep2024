package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Pioneer One TeleOp")
public class TestingActiveIntake extends OpMode {

    CRServo activeIntake;

    @Override
    public void init() {
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
    }

    @Override
    public void loop() {
        while(gamepad2.dpad_down){
            activeIntake.setPower(-1.0);
        }

        while(gamepad2.dpad_up){
            activeIntake.setPower(1.0);
        }

        activeIntake.setPower(0);
    }
}
