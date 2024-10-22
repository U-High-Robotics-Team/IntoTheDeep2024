package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Final ServoTest")
public class ServoTest extends OpMode {

    // initalizing servo
    Servo myServo;

    public void init(){

        myServo = hardwareMap.get(Servo.class, "Servo");

    }

    public void init_loop() {
    }


    @Override
    public void loop() {
        double joystickValue = gamepad1.left_stick_y; // Invert if necessary

        double servoPosition = (joystickValue + 1) / 2;

        // Set the servo position
        myServo.setPosition(servoPosition);
    }
}
