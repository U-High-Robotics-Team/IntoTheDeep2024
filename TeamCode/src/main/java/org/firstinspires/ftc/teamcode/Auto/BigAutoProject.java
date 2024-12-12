package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/* This is a long-term project to create a very powerful and accurate codebase
 * for auto mode that can also be used in teleop. The core structure is a set of 
 * targets for each robot axis and a loop that quickly cycles and attempts to move
 * each axis closer to its target.
 * 
 * This structure allows for a modular and flexible robot controller, where each
 * axis can be swapped out or tweaked individually. For example, some axes can use
 * default control commands and other can use custom PID control.
 * 
 * Similarly, control inputs can come from  multiple sources - an array of presets, 
 * a file, or the gamepad. Postion tracking can also be swapped from odometry pods, 
 * encoders, or just using time & distance. 
*/

@Autonomous(name = "Big Auto Project")
public class BigAutoProject extends OpMode {

    // motors
    private DcMotorEx motorFR;
    private DcMotorEx motorFL;
    private DcMotorEx motorBR;
    private DcMotorEx motorBL;
    private DcMotor slide;
    private DcMotor shoulder;
    private Servo wrist;
    private Servo holder;
    private Servo claw;

    // robot position in world coordinates
    double worldX = 0;
    double worldY = 0;
    double worldRot = 0;
    double currMoveTime = 0;  // in sec - time since start of current move

    // position targets
    double worldXTarget = 0;
    double worldYTarget = 0;
    double worldRotTarget = 0;
    double shoulderTarget = 0;
    double wristTarget = 0;
    double clawTarget = 0;
    double slideTarget = 0;
    double timeRemaining = 0;   // in sec - used to calculate velocities
    
    // PID state
    double error = 0;
    double lastError = 0;
    double integralSum = 0;
    double prevTime = 0;

    // TODO make a Robot class and instantiate a bot object
    // robot geometry constants
    private static final double WHEEL_RADIUS = 0.104 / 2; // meters
    private static final double AXLE_CONSTANT = 0.3380; // (width + length) / 2;
    // TODO goBilda 5203 312rpm should be 527.7 steps per rev
    private static final double STEPS_PER_REV = 751.8;
    private static final double REVS_PER_METER = 1 / (2 * Math.PI * WHEEL_RADIUS);
    private static final double STEPS_PER_METER = STEPS_PER_REV * REVS_PER_METER;
    private static final double STEPS_PER_RAD = STEPS_PER_REV / (2 * Math.PI);
    private final int SLIDE_Y_MAX = 2400;
    private final int SLIDE_X_MAX = 1000; // Maximum position (top)
    private final int SLIDE_MIN = 0; // Minimum position (bottom)
    // TODO play around with the below value to find a good threshold to change from slower robot to faster robot
    private final double SLIDE_POSITION_THRESHOLD = 1900;
    private final int SHOULDER_MAX = 1500;
    private final int SHOULDER_MIN = 0;
    private final double WRIST_MAX = 0.6;
    private final double WRIST_MIN = 0.0;
    private final double WRIST_CLIP = 0.3;
    private final double CLAW_MAX = 0.6;
    private final double CLAW_MIN = 0;
    private final double TIMEOUT = 2;  // seconds: how long to wait after allowed time before going on to next move

    // control parameters
    private static final double KP = 0.003; // bigger the error the faster we will fix it
    private static final double KI = 0.0001; // provides extra boost when you get close to the target
    private static final double KD = 0.00015; // dampens overshoot
    private static final double ERROR_TOLERANCE = 0.05; // meters
    private static final double MAX_SPEED = 2; // meters per second
    private static final double MAX_ROT_SPEED = 0.1; // radians per second
    private final double SLIDE_POWER = 1.0;
    private final double SHOULDER_POWER = 0.5;
    // TODO (long-term) create a Controller class that holds state for each motor and does PID control - much cleaner code

    /**
     * The init method is called once when the DS INIT button is pressed. It does hardware configuration and
     * sets state variables to their initial values. 
     */ 
    @Override
    public void init() {
        // connect to hardware map
        motorBL = hardwareMap.get(DcMotorEx.class, "backleft");
        motorBR = hardwareMap.get(DcMotorEx.class, "backright");
        motorFL = hardwareMap.get(DcMotorEx.class, "frontleft");
        motorFR = hardwareMap.get(DcMotorEx.class, "frontright");
        slide = hardwareMap.get(DcMotor.class, "elevator");
        shoulder = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // set encoders
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // TODO - do we need these lines? What do they do?
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // correct some motor directions
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // set brakes
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * The loop method is called repeatedly when the DS START buttton is pressed. It it called after start(), if present.
     * This is the core of the OpMode behavior. The code structure in this particular case is to have targets for 
     * each axis, then call methods that move each axis a tiny bit closer to its target each time through the loop.
     *  Do NOT have any invoked method run until complete. This will block the other methods from doing their jobs.
     * The targets are set using the getInput() call, which can use a data array, file input, gamepad input, etc. 
     */
    @Override
    public void loop() {
        getCurrState();
        getInput();
        // TODO implement a "busy" counter to see if any of the following moves are still in process
        moveChassis();
        moveShoulder();
        moveSlide();
        moveWrist();
        moveClaw();
        // if not busy OR timeout, then . . .
        telemetry.update();
    }

    private void getInput(){
        // TODO read a list of desired actions using a counter to keep track of which step we're on
        // each action should set the target x, y, rot, arm, wrist, etc.
        // each action should also set a time target and contingency timeout
        // move to next action if (time > time target && within tolerances) or (time > time target + timeout) 
    }

    private void getCurrState(){
        // TODO use odometry pod (or IMU or encoder steps) to update worldX, worldY, worldRot
        // TODO update timeRemaining
        // update other states as needed
    }

    public int moveChassis() {
        lastError = error;
        double dX = worldXTarget - worldX;
        double dY = worldYTarget - worldY;
        double dRot = worldRotTarget - worldRot;
        error = Math.sqrt(dX * dX + dY * dY +  (dRot  * dRot ) / (AXLE_CONSTANT * AXLE_CONSTANT));  // this is an attempt to give postion error and rotation error equal weight
        
        telemetry.addData("X Current / Target ", "(%.2f, %.2f)", worldX, worldXTarget);
        telemetry.addData("Y Current / Target ", "(%.2f, %.2f)", worldY, worldYTarget);
        telemetry.addData("Rot Current / Target ", "(%.2f, %.2f)", worldRot, worldRotTarget);

        moveWheels(dX, dY, dRot);

        if (error > ERROR_TOLERANCE){
            return 1;   // still moving
        } else {
            return 0;   // not busy     
        }
    }
     
    private void moveWheels(double deltaX, double deltaY, double deltaRot) {
        // cordinate transform into motion in robot's local coordinate system
        // TODO double check this!
        double deltaX_bot = deltaX * Math.cos(worldRot) - deltaY * Math.sin(worldRot);
        double deltaY_bot = deltaX * Math.sin(worldRot) + deltaY * Math.cos(worldRot)
        // deltaRot is the same

        // velocities are based on travel distance and user-specified travel time
        double xVel = (deltaX_bot) / timeRemaining;
        double yVel = (deltaY_bot) / timeRemaining;
        double rotVel = (worldRotTarget - worldRot) / timeRemaining;
        
        // calculate angular velocity for each motor (physics)
        // TODO - do you want angular velocites in radians/sec or encoder steps/sec? It only affects PID constants
        // w = 1/wheelradius * (xvel + yvel - (distance from center * rotational velocity)) * steps/rad
        // units = (rad/sec * steps/rad) = steps/sec
        double wFL = ((xVel + yVel) + (AXLE_CONSTANT * rotVel)) * STEPS_PER_RAD / WHEEL_RADIUS;// + + +
        double wFR = ((xVel - yVel) - (AXLE_CONSTANT * rotVel)) * STEPS_PER_RAD / WHEEL_RADIUS;// + - -
        double wBL = ((xVel - yVel) + (AXLE_CONSTANT * rotVel)) * STEPS_PER_RAD/ WHEEL_RADIUS;// + - +
        double wBR = ((xVel + yVel) - (AXLE_CONSTANT * rotVel)) * STEPS_PER_RAD/ WHEEL_RADIUS;// + + -

        motorFL.setPower(findPIDPower(wFL));
        motorFR.setPower(findPIDPower(wFR));
        motorBL.setPower(findPIDPower(wBR));
        motorBR.setPower(findPIDPower(wBR));
    }

    public double findPIDPower(double angVel) {
        // TODO this uses the overall position error to calculate PID values. Tracking individual wheel errors may be more precise, but more complex and may not be needed.
       
        // track time step
        double currTime = this.getRuntime();
        double timestep = currTime - prevTime;
        prevTime = currTime;

        // sum of all error over time
        integralSum += error * timestep;

        // rate of change of the error
        double derivative = (error - lastError) / timestep;

        double out = (KP * error) + (KI * integralSum) + (KD * derivative);
        out = Math.max(-1, Math.min(1, out));
        return out;
    }

    public int moveWrist() {
        wrist.setPosition(wristTarget);
        telemetry.addData("Wrist Current / Target ", "(%.2f, %.2f)", 0.0, wristTarget);
        return 0;  // assume move succeeded (for now - maybe improve later)
    }
    
    public int moveClaw() {
        claw.setPosition(clawTarget);
        telemetry.addData("Claw Current / Target ", "(%.2f, %.2f)", 0.0, clawTarget);
        return 0;  // assume move succeeded (for now - maybe improve later)
    }
    
    public int moveShoulder() {
        shoulder.setTargetPosition((int)shoulderTarget);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(Math.abs(SHOULDER_POWER));
        telemetry.addData("Shoulder Current / Target ", "(%d, %.2f)", shoulder.getCurrentPosition(), shoulderTarget);
        return 0;  // assume move succeeded (for now - maybe improve later)
    }
    
    public int moveSlide() {
        slide.setTargetPosition((int)slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(Math.abs(SLIDE_POWER));
        telemetry.addData("Slide Current / Target ", "(%d, %.2f)", slide.getCurrentPosition(), slideTarget);
        return 0;  // assume move succeeded (for now - maybe improve later)
    }

}
