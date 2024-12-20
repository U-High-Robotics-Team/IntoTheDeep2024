package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="OperatorControls")
public class OperatorControls extends OpMode {

    // initalizing motors
    private DcMotor elevator;
    private DcMotor arm;

    public void moveArm() {
        //TODO: Find these values with debugging
        double maxPosition = 2000;
        double minPosition = 0;

        double speed = gamepad2.right_stick_y;

        if(arm.getCurrentPosition() <= maxPosition && speed > 0){
            arm.setPower(gamepad2.left_stick_y);
        }else if(arm.getCurrentPosition() >= minPosition && speed < 0){
            arm.setPower(gamepad2.left_stick_y);
        }else{
            arm.setPower(0);
        }

        telemetry.addData("Arm Position", arm.getCurrentPosition());
    }

    public void moveLift() {
        //TODO: Find these values with debugging
        double maxPosition = 0;
        double minPosition = -6100;


        double speed = gamepad2.left_stick_y;

        if(elevator.getCurrentPosition() <= maxPosition && speed >= 0){
            elevator.setPower(gamepad2.left_stick_y);
        }else if(elevator.getCurrentPosition() >= minPosition && speed <= 0){
            elevator.setPower(gamepad2.left_stick_y);
        }else{
            elevator.setPower(0);
        }

        telemetry.addData("Elevator Position", elevator.getCurrentPosition());
    }

    public void init(){
        // connect to hardware map
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        arm = hardwareMap.get(DcMotor.class, "arm");

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // if needed reverse the motor directions
//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init_loop() {
    }


    public void loop(){
        moveArm();
        moveLift();

        telemetry.addData("GamePad2.RightStickY", gamepad2.right_stick_y);
        telemetry.addData("GamePad2.LeftStickY", gamepad2.left_stick_y);
        telemetry.update();
    }
}

