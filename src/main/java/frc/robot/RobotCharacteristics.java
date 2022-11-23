package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RobotCharacteristics {

    // DRIVE TRAIN
    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 8.4;
    public static final double DRIVE_WHEEL_RADIUS_M = Units.inchesToMeters(2.5);
    public static final int DRIVE_SIDE_MOTOR_COUNT = 2;
    public static final DCMotor DRIVE_MOTORS = DCMotor.getCIM(DRIVE_SIDE_MOTOR_COUNT);
    public static final double WEIGHT_KG = 20;
    public static final double DRIVE_TRACK_WIDTH_M = 0.9;
    public static final double ROBOT_WIDTH_M = DRIVE_TRACK_WIDTH_M;
    public static final double ROBOT_LENGTH_M = 5;
    // for rectangle -> I = m * (a^2 + b^2) / 12, where b = width, a = length
    public static final double DRIVE_MOMENT_OF_INERTIA = WEIGHT_KG * (Math.pow(ROBOT_LENGTH_M, 2) + Math.pow(ROBOT_WIDTH_M, 2)) / 12;


    // SHOOTER
    public static final DCMotor SHOOTER_MOTORS = DCMotor.getCIM(2);
    public static final double SHOOTER_WHEEL_MASS_KG = 0.2;
    public static final double SHOOTER_MOTOR_TO_WHEEL_GEAR_RATIO = Constants.SHOOTER_GEAR_RATIO;
    public static final double SHOOTER_WHEEL_RADIUS_M = 2;
    // for circle -> I = M * r^2
    public static final double SHOOTER_MOMENT_OF_INERTIA = SHOOTER_WHEEL_MASS_KG * Math.pow(SHOOTER_WHEEL_RADIUS_M, 2);


    public static final double ksVolts = 0.7604;
    public static final double kvVoltSecondsPerMeter = 1.8355;
    public static final double kaVoltSecondsSquaredPerMeter = 1.156;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 2.5;
    public static final double kTrackwidthMeters = DRIVE_TRACK_WIDTH_M;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;    
}
