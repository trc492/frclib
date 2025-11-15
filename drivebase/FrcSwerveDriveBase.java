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
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcSwerveDriveBase;  // From trclib
import trclib.drivebase.TrcSwerveModule;  // Assuming your module type
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcGyro;

public class FrcSwerveDriveBase extends TrcSwerveDriveBase
{
    private static final String moduleName = FrcSwerveDrive.class.getSimpleName();
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

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param xPower    specifies the x power.
     * @param yPower    specifies the y power.
     * @param turnPower specifies the rotating power.
     * @param inverted  specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    public void holonomicDrive(
        String owner, double xPower, double yPower, double turnPower, boolean inverted, Double gyroAngle,
        double driveTime, TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ", x=" + xPower +
            ", y=" + yPower +
            ", turn=" + turnPower +
            ", inverted=" + inverted +
            ", gyroAngle=" + gyroAngle +
            ", driveTime=" + driveTime +
            ", event=" + event);

        if (validateOwnership(owner))
        {
            if (inverted)
            {
                xPower = -xPower;
                yPower = -yPower;
            }

            if (gyroAngle != null)
            {
                if (inverted)
                {
                    tracer.traceWarn(
                        moduleName, "You should not be using inverted and field reference frame at the same time!");
                }

                // double gyroRadians = Math.toRadians(gyroAngle);
                // double temp = yPower * Math.cos(gyroRadians) + xPower * Math.sin(gyroRadians);
                // xPower = -yPower * Math.sin(gyroRadians) + xPower * Math.cos(gyroRadians);
                // yPower = temp;
            }
            else if (isGyroAssistEnabled())
            {
                turnPower += getGyroAssistPower(turnPower);
            }

            if (isAntiTippingEnabled())
            {
                xPower += getAntiTippingPower(true);
                yPower += getAntiTippingPower(false);
            }

            xPower = TrcUtil.clipRange(xPower);
            yPower = TrcUtil.clipRange(yPower);
            turnPower = TrcUtil.clipRange(turnPower);

            // Scale powers to speed in m/s
            double xSpeed = yPower * maxDriveSpeed;
            double ySpeed = -xPower * maxDriveSpeed;
            double turnSpeed = -turnPower * maxTurnSpeed;
            ChassisSpeeds targetSpeeds;
            if (gyroAngle != null)
            {
                Rotation2d robotAngle = new Rotation2d(Units.degreesToRadians(-gyroAngle));
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, robotAngle);
            }
            else
            {
                targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            }
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);

            // Desaturate to prevent exceeding max speed
            SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

            // Apply to modules (TrcLib interface; assumes set(speed, angle) in m/s and radians/degrees—match units)
            for (int i = 0; i < swerveModules.length; i++)
            {
                swerveModules[i].driveMotor.setVelocity(Units.metersToInches(states[i].speedMetersPerSecond));
                swerveModules[i].setSteerAngle(states[i].angle.getDegrees());
            }
            setDriveTime(owner, driveTime, event);
        }
    }   //holonomicDrive

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
            double distanceMeters = Units.inchesToMeters(swerveModules[i].driveMotor.getPosition());
            double steerAngleRad = Math.toRadians(swerveModules[i].getSteerAngle());
            positions[i] = new SwerveModulePosition(distanceMeters, new Rotation2d(steerAngleRad));
        }
        return positions;
    }
}
