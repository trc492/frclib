package frclib.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frclib.sensor.FrcAHRSGyro;
import trclib.drivebase.TrcSwerveDriveBase;  // From trclib
import trclib.drivebase.TrcSwerveModule;  // Assuming your module type
import trclib.sensor.TrcGyro;

public class FrcSwerveDriveBase extends TrcSwerveDriveBase
{
    private final TrcSwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final double maxDriveSpeed;  // e.g., 4.0 m/s; adjust to your robot
    private final double maxTurnSpeed;
    private final SwerveDriveOdometry odometry;
    private Pose2d currentPose = new Pose2d();

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param flModule specifies the left front swerve module of the drive base.
     * @param blModule specifies the left back swerve module of the drive base.
     * @param frModule specifies the right front swerve module of the drive base.
     * @param brModule specifies the right back swerve module of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     * @param maxDriveSpeed specifies the robot's max translational velocity.
     * @param maxTurnSpeed specifies the robot's max rotational veloicty.
     */
    public FrcSwerveDriveBase(
        TrcSwerveModule flModule, TrcSwerveModule blModule, TrcSwerveModule frModule, TrcSwerveModule brModule,
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength, double maxDriveSpeed, double maxTurnSpeed)
    {
        super(flModule, blModule, frModule, brModule, gyro, wheelBaseWidth, wheelBaseLength);
        swerveModules = new TrcSwerveModule[4];
        swerveModules[0] = flModule;
        swerveModules[1] = frModule;
        swerveModules[2] = blModule;
        swerveModules[3] = brModule;
        this.maxDriveSpeed = Units.inchesToMeters(maxDriveSpeed);
        this.maxTurnSpeed = Units.degreesToRadians(maxTurnSpeed);

        // Define kinematics (order: FL, FR, BL, BR)
        double halfWheelbase = Units.inchesToMeters(wheelBaseLength / 2.0);
        double halfTrackwidth = Units.inchesToMeters(wheelBaseWidth / 2.0);
        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(halfWheelbase, halfTrackwidth),
            new Translation2d(halfWheelbase, -halfTrackwidth),
            new Translation2d(-halfWheelbase, halfTrackwidth),
            new Translation2d(-halfWheelbase, -halfTrackwidth));

        // Initial module positions (distance in meters, angle as Rotation2d)
        SwerveModulePosition[] initialPositions = getModulePositions();
        // Init odometry (gyro angle as Rotation2d; convert from TrcLib degrees)
        Rotation2d initialGyro = new Rotation2d(Math.toRadians(((FrcAHRSGyro) gyro).ahrs.getYaw()));
        this.odometry = new SwerveDriveOdometry(kinematics, initialGyro, initialPositions, currentPose);
    }

    @Override
    public void holonomicDrive(double xPower, double yPower, double turnPower)
    {
        // Scale powers to m/s
        double xSpeed = xPower * maxDriveSpeed;
        double ySpeed = yPower * maxDriveSpeed;
        double turnSpeed = turnPower * maxTurnSpeed;  // Angular speed in rad/s? Adjust if needed (e.g., * wheelbase)

        // Create ChassisSpeeds (field-relative if desired; add gyro here for that)
        // For now, robot-relative; extend for field-relative via pose (see odometry section)
        ChassisSpeeds speeds = new ChassisSpeeds(ySpeed, xSpeed, turnSpeed);

        // WPILib kinematics
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        // Desaturate to prevent exceeding max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

        // Apply to modules (TrcLib interface; assumes set(speed, angle) in m/s and radians/degrees—match units)
        for (int i = 0; i < swerveModules.length; i++)
        {
            swerveModules[i].driveMotor.setVelocity(Units.metersToInches(states[i].speedMetersPerSecond));
            swerveModules[i].setSteerAngle(states[i].angle.getDegrees());
        }
    }

    // Expose kinematics for other uses (e.g., trajectories)
    public SwerveDriveKinematics getKinematics()
    {
        return kinematics;
    }

    private SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++)
        {
            double distanceMeters = Units.inchesToMeters(swerveModules[i].driveMotor.getPosition());//WHEEL_CIRCUMFERENCE / GEAR_RATIO;  // Adjust conversion
            double steerAngleRad = Math.toRadians(swerveModules[i].getSteerAngle());
            positions[i] = new SwerveModulePosition(distanceMeters, new Rotation2d(steerAngleRad));
        }
        return positions;
    }
}
