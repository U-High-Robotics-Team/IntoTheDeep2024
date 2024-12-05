//Credits: RoadRunner INDEX

public static double TICKS_PER_REV = 0;
public static double WHEEL_RADIUS = 2; // in
public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

public static double PARALLEL_X = 0; // X is the forward and back direction
public static double PARALLEL_Y = 0; // Y is the strafe direction

public static double PERPENDICULAR_X = 0; // X is the forward and back direction
public static double PERPENDICULAR_Y = 0; // Y is the strafe direction

parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder")); //Change name
perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder")); //Change name

// TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

// If you need to reverse the perpendicular encoder:
// Vice-versa for the other encoder
perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

//What accuracy?
public static double X_MULTIPLIER = 1; // Multiplier in the X direction
public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

@NonNull
@Override

//Wheel posits
  
public List<Double> getWheelPositions() {
    return Arrays.asList(
            encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
    );
}

@NonNull
@Override
public List<Double> getWheelVelocities() {
    //Check for odometry velocities

    return Arrays.asList(
            encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
    );
}
