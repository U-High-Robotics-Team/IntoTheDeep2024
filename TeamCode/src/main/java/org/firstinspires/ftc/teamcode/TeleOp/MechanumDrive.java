//System settings --> ODOMETRY REQUIREMENT

//IMU SETUP [Intertial Measuring Unit]

imu = hardwareMap.get(BNO055IMU.class, "imu");
BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
imu.initialize(parameters);

//Road Runner Perspective

@Override
public Double getExternalHeadingVelocity() {
    // TODO: This must be changed to match your configuration
    //                           | Z axis
    //                           |
    //     (Motor Port Side)     |   / X axis
    //                       ____|__/____
    //          Y axis     / *   | /    /|   (IO Side)
    //          _________ /______|/    //      I2C
    //                   /___________ //     Digital
    //                  |____________|/      Analog
    //
    //                 (Servo Port Side)
    //
    // The positive x axis points toward the USB port(s)
    //
    // Adjust the axis rotation rate as necessary
    // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
    // flat on a surface

    return (double) imu.getAngularVelocity().zRotationRate;
}
