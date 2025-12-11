/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.drivebase.TrcSwerveModule;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcGyro;

public class FrcSwerveDriveBase extends TrcSwerveDriveBase implements TrcDriveBaseOdometry
{
    private static final String moduleName = FrcSwerveDrive.class.getSimpleName();
    private final TrcSwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final double maxDriveSpeed; // in m/s
    private final double maxTurnSpeed;  // in rad/s
    private final SwerveDriveOdometry odometry;
    private Pose2d currentPose = new Pose2d();
    private TrcPose2D trcPose = new TrcPose2D();

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
     * @param maxDriveSpeed specifies the robot's max translational velocity in inches/sec.
     * @param maxTurnSpeed specifies the robot's max rotational veloicty in degrees/sec.
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
        this.maxTurnSpeed = Math.toRadians(maxTurnSpeed);

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
        Rotation2d initialGyro = Rotation2d.fromDegrees(gyro != null? -gyro.getZHeading().value: 0.0);
        this.odometry = new SwerveDriveOdometry(kinematics, initialGyro, initialPositions, currentPose);
        updateCache();
    }   //FrcSwerveDriveBase

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
            boolean fieldRelative = gyroAngle != null;

            if (inverted)
            {
                xPower = -xPower;
                yPower = -yPower;
            }

            if (fieldRelative)
            {
                if (inverted)
                {
                    tracer.traceWarn(
                        moduleName, "You should not be using inverted and field reference frame at the same time!");
                }
            }
            else if (isGyroAssistEnabled())
            {
                // Apply assist features (only in robot-relative mode)
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

            // Scale powers to speed in m/s and convert TrcLib → WPILib convention
            double xSpeedMps = yPower * maxDriveSpeed;
            double ySpeedMps = -xPower * maxDriveSpeed;
            double omegaRadPerSec = -turnPower * maxTurnSpeed;
            // Build ChassisSpeeds
            ChassisSpeeds targetSpeeds;
            if (fieldRelative)
            {
                Rotation2d fieldHeading = Rotation2d.fromDegrees(-gyroAngle);
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedMps, ySpeedMps, omegaRadPerSec, fieldHeading);
            }
            else
            {
                targetSpeeds = new ChassisSpeeds(xSpeedMps, ySpeedMps, omegaRadPerSec);
            }
            // Kinematics → module states
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);
            // Desaturate to prevent exceeding max speed
            SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);
            // Send to modules
            for (int i = 0; i < swerveModules.length; i++)
            {
                states[i].optimize(Rotation2d.fromDegrees(-swerveModules[i].getSteerAngle()));
                swerveModules[i].driveMotor.setVelocity(Units.metersToInches(states[i].speedMetersPerSecond));
                swerveModules[i].setSteerAngle(-states[i].angle.getDegrees(), false, true);
            }
            // Do Timed Drive if necessary
            setDriveTime(owner, driveTime, event);
        }
    }   //holonomicDrive

    public SwerveDriveKinematics getKinematics()
    {
        return kinematics;
    }   //getKinematics

    private SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++)
        {
            double distanceMeters = Units.inchesToMeters(swerveModules[i].driveMotor.getPosition());
            double steerAngleRad = Math.toRadians(-swerveModules[i].getSteerAngle());
            positions[i] = new SwerveModulePosition(distanceMeters, new Rotation2d(steerAngleRad));
        }
        return positions;
    }   //getModulePositions

    //
    // Implements TrcDriveBaseOdometry interface.
    //

    /**
     * This method is called once at the beginning of the INPUT_TASK loop. Odometry device can update their cache
     * at this time.
     */
    @Override
    public void updateCache()
    {
        double gyroYawDeg = gyro != null? -gyro.getZHeading().value: 0.0;
        Rotation2d gyroRot = Rotation2d.fromDegrees(gyroYawDeg);
        SwerveModulePosition[] positions = getModulePositions();

        currentPose = odometry.update(gyroRot, positions);
        trcPose.x = Units.metersToInches(-currentPose.getY());
        trcPose.y = Units.metersToInches(currentPose.getX());
        trcPose.angle = -currentPose.getRotation().getDegrees();
        tracer.traceDebug(moduleName, "Odometry: " + trcPose + ", gyro=" + gyroYawDeg);
    }   //updateCache

    /**
     * This method resets the DriveBase position.
     */
    @Override
    public void reset()
    {
        setPosition(new TrcPose2D(0.0, 0.0, 0.0));
    }   //reset

    /**
     * This method returns the DriveBase position.
     *
     * @return DriveBase position.
     */
    @Override
    public TrcPose2D getPosition()
    {
        return trcPose.clone();
    }   //getPosition

    /**
     * This method sets the DriveBase position.
     *
     * @param pose specifies the DriveBase position.
     */
    @Override
    public void setPosition(TrcPose2D pose)
    {
        Pose2d pose2d = new Pose2d(
            Units.inchesToMeters(pose.y), Units.inchesToMeters(-pose.x), Rotation2d.fromDegrees(-pose.angle));
        SwerveModulePosition[] pos = getModulePositions();
        Rotation2d gyroRot = Rotation2d.fromDegrees(gyro != null? -gyro.getZHeading().value: 0.0);

        odometry.resetPosition(gyroRot, pos, pose2d);
        // Update cached TrcLib pose
        trcPose = pose.clone();
    }   //setPosition

    /**
     * This method returns the DriveBase velocity.
     *
     * @return DriveBase velocity.
     */
    @Override
    public TrcPose2D getVelocity()
    {
        // Sample current module states (drive velocity in m/s, steer angle in radians)
        SwerveModuleState[] currentStates = new SwerveModuleState[4];
        for (int i = 0; i < currentStates.length; i++)
        {
            TrcSwerveModule module = swerveModules[i];
            double driveVelMps = Units.inchesToMeters(module.driveMotor.getVelocity());
            currentStates[i] = new SwerveModuleState(driveVelMps, Rotation2d.fromDegrees(-module.getSteerAngle()));
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(currentStates);
        double xVel = Units.metersToInches(-speeds.vyMetersPerSecond);
        double yVel = Units.metersToInches(speeds.vxMetersPerSecond);
        double turnVel = Math.toDegrees(-speeds.omegaRadiansPerSecond);

        return new TrcPose2D(xVel, yVel, turnVel);
    }   //getVelocity

}   //class FrcSwerveDriveBase
