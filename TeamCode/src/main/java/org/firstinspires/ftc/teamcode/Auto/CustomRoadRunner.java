#include <AccelStepper.h>

        #include <MultiStepper.h>



        #define STEPS_PER_REV 400

        #define WHEEL_RAD 0.05

        #define PI 3.14159

        #define REVS_PER_METER 1 / (2 * PI * WHEEL_RAD)



//front left, 1

        #define STEPPER1_DIR_PIN 2

        #define STEPPER1_STEP_PIN 7



//back left, 2

        #define STEPPER2_DIR_PIN 3

        #define STEPPER2_STEP_PIN 8



//middle, 3

        #define STEPPER3_DIR_PIN 4

        #define STEPPER3_STEP_PIN 9



//back right, 4

        #define STEPPER4_DIR_PIN 5

        #define STEPPER4_STEP_PIN 10



//front right, 5

        #define STEPPER5_DIR_PIN 6

        #define STEPPER5_STEP_PIN 11



        #define MAX_STEP_SPEED 1500  // from library, 1000 steps/sec max



        AccelStepper stepperFL(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);

        AccelStepper stepperBL(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

        AccelStepper stepper3(AccelStepper::DRIVER, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN);

        AccelStepper stepperBR(AccelStepper::DRIVER, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

        AccelStepper stepperFR(AccelStepper::DRIVER, STEPPER5_STEP_PIN, STEPPER5_DIR_PIN);

        MultiStepper steppers;



        const float stepsPerRad = 62.832;

        const float STEPS_PER_METER = STEPS_PER_REV / REVS_PER_METER;

        const float invWheelRadius = 19.685;   // in meters

        const float lengthCon = 0.3380;        //0.3975 half-width + half-length in meters

        const float maxSpeed = 2;              //meters/sec

        const float positionTolerance = 0.05;  //in meters

        const float angleTolerance = 0.1;      //radians

        const float maxRotSpeed = 0.1;        //radians / second

        const float axleConst = 0.35;          //(width + height) / 2;

        const float wheelRadius = 0.05;

        const float TIMESTEP = 0.1;

        char input[64] = "";

        int inputIdx = 0;



        void setup() {

        Serial.begin(115200);

        Serial.println("Serial started.");



        int moveTo = 0;

        int minPulseWidth = 200;



        stepperFL.setMaxSpeed(MAX_STEP_SPEED);

        // stepperFL.setAcceleration(accel);

        stepperFL.setMinPulseWidth(minPulseWidth);

        steppers.addStepper(stepperFL);



        stepperFR.setMaxSpeed(MAX_STEP_SPEED);

        // stepperFR.setAcceleration(accel);

        stepperFR.setMinPulseWidth(minPulseWidth);

        steppers.addStepper(stepperFR);



        stepperBL.setMaxSpeed(MAX_STEP_SPEED);

        // stepperBL.setAcceleration(accel);

        stepperBL.setMinPulseWidth(minPulseWidth);

        steppers.addStepper(stepperBL);



        stepperBR.setMaxSpeed(MAX_STEP_SPEED);

        // stepperBR.setAcceleration(accel);

        stepperBR.setMinPulseWidth(minPulseWidth);

        steppers.addStepper(stepperBR);



        Serial.println("Motors initialized.");



        //polarGo(3, 0, 0.5, -PI/2);

        }



        void loop() {

        if (Serial.available()) {

        processSerialIn();

        }

        delay(1);

        }



        void roboGo(float x, float y, float deg, float speed) {

        const float velo = 2;  // arbitrary - just for calculating steps

        float dist = sqrt((x * x) + (y * y));

        float time = dist / velo;

        float xVelo;

        float yVelo;

        if (dist == 0) {

        time = velo;

        xVelo = 0;

        yVelo = 0;

        } else {

        xVelo = (x / dist) * velo;

        yVelo = (y / dist) * velo;

        }

        float rotVelo = (deg / time) * (M_PI / 180);

        // Serial.println(x / dist) ;



        //what type of var

        float wFL = invWheelRadius * ((xVelo + yVelo) - (lengthCon * rotVelo));

        float wFR = invWheelRadius * ((-xVelo + yVelo) + (lengthCon * rotVelo));

        float wBL = invWheelRadius * ((-xVelo + yVelo) - (lengthCon * rotVelo));

        float wBR = invWheelRadius * ((xVelo + yVelo) + (lengthCon * rotVelo));

        // Serial.println(wFL);



        float stepSpeedFL = wFL * stepsPerRad;

        float stepSpeedFR = wFR * stepsPerRad;

        float stepSpeedBL = wBL * stepsPerRad;

        float stepSpeedBR = wBR * stepsPerRad;



        // instead of stuff below, set wheel speeds in multistepper and runSpeed



        float stepsFL = (long)(stepSpeedFL * time);

        float stepsFR = (long)(stepSpeedFR * time);

        float stepsBL = (long)(stepSpeedBL * time);

        float stepsBR = (long)(stepSpeedBR * time);



        stepperFR.move(stepsFR);

        stepperFL.move(-stepsFL);

        stepperBR.move(stepsBR);

        stepperBL.move(-stepsBL);



        stepperFR.setCurrentPosition(0);

        stepperFL.setCurrentPosition(0);

        stepperBR.setCurrentPosition(0);

        stepperBL.setCurrentPosition(0);



        long positions[4];

        positions[0] = -stepsFL;

        positions[1] = stepsFR;

        positions[2] = -stepsBL;

        positions[3] = stepsBR;

        steppers.moveTo(positions);

        }



        void polarGo(float r, float theta, float speed, float rot) {

        if (speed <= 0) {

        speed = min(speed, maxSpeed);

        }

        long timeStart = millis();

        while (r > positionTolerance || abs(rot) > angleTolerance) {

        Serial.print("rot");

        Serial.println(rot);



        if (millis() - timeStart > TIMESTEP * 1000) {

        timeStart = millis();

        // recompute velocites for the next timestep

        float travelTime = r / speed;

        float xVelo = speed * cos(theta);

        float yVelo = speed * sin(theta);

        //float rotVelo = max(-maxRotSpeed, (min(maxRotSpeed, rot / (r / speed))));

        float rotVelo = speed;

        Serial.print("rotvelo=");

        Serial.println(rotVelo);

        wheelSpeeds(xVelo, yVelo, rotVelo);

        // update targets for the next timestep

        r -= speed * TIMESTEP;

        rot -= rotVelo * TIMESTEP;

        theta -= rotVelo * TIMESTEP;

        }

        stepperFL.runSpeed();

        stepperFR.runSpeed();

        stepperBL.runSpeed();

        stepperBR.runSpeed();

        }

        Serial.println("Finished move.");

        }



        void wheelSpeeds(double xV, double yV, double rV) {

        //

        stepperFL.setSpeed(-((xV - yV - axleConst * rV) * STEPS_PER_METER));  // +-- fl

        //Serial.print("wFl=");

        //Serial.println(stepperFL.speed());

        stepperFR.setSpeed((xV + yV + axleConst * rV)  * STEPS_PER_METER);     // +++ fr

        stepperBL.setSpeed(-((xV + yV - axleConst * rV) * STEPS_PER_METER));  // ++-; bl

        stepperBR.setSpeed((xV - yV + axleConst * rV) * STEPS_PER_METER);     // +-+; br

        }



        void processSerialIn() {

        Serial.println("Data received:");

        float r = Serial.parseFloat();

        float th = Serial.parseFloat();

        float spd = Serial.parseFloat();

        float rot = Serial.parseFloat();

        Serial.readStringUntil('\n');

        if (spd != 0 && (r != 0 || rot != 0)) {

        Serial.println("Command sent.");

        polarGo(r, th, spd, rot);

        }

        }



        void moveX(double meters) {

        double moveTo = meters * 1250;

        stepperFL.moveTo(-moveTo);

        stepperFR.moveTo(moveTo);

        stepperBL.moveTo(-moveTo);

        stepperBR.moveTo(moveTo);

        go();

        }



        void spin(double degrees) {

        double moveTo = degrees * 7.291666;  //2625 for 360

        stepperFL.moveTo(moveTo);

        stepperFR.moveTo(moveTo);

        stepperBL.moveTo(moveTo);

        stepperBR.moveTo(moveTo);

        go();

        // + = cc

        }



        void moveY(double meters) {

        double moveTo = meters * 1250;

        stepperFL.moveTo(-moveTo);

        stepperFR.moveTo(-moveTo);

        stepperBL.moveTo(moveTo);

        stepperBR.moveTo(moveTo);

        go();

        }



        void go() {

        stepperFL.run();

        stepperBR.run();

        stepperBL.run();

        stepperFR.run();

        }



        void off(){

        stepperFL.setSpeed(0);

        stepperFR.setSpeed(0);

        stepperBL.setSpeed(0);

        }