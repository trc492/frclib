/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frclib.driverio.FrcDashboard;
import frclib.motor.FrcMotorActuator;
import frclib.sensor.FrcAHRSGyro;
import frclib.sensor.FrcEncoder;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.drivebase.TrcSwerveModule;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcEncoder;

/**
 * This class creates the FrcSwerve drive base subsystem that consists of wheel motors and related objects for
 * driving a swerve robot.
 */
public class FrcSwerveDrive extends FrcRobotDrive
{
    /**
     * This class contains Swerve Robot Info.
     */
    public static class SwerveInfo extends FrcRobotDrive.RobotInfo
    {
        // Steer Encoder parameters.
        public FrcEncoder.EncoderType steerEncoderType = null;
        public String[] steerEncoderNames = null;
        public int steerEncoderIds[] = null;
        public boolean[] steerEncoderInverted = null;
        public double[] steerEncoderZeros = null;
        public String steerZerosFilePath = null;
        // Steer Motor parameters.
        public FrcMotorActuator.MotorType steerMotorType = null;
        public boolean steerMotorBrushless = false;
        public boolean steerMotorAbsEnc = false;
        public String[] steerMotorNames = null;
        public int[] steerMotorIds = null;
        public boolean[] steerMotorInverted = null;
        public TrcPidController.PidCoefficients steerMotorPidCoeffs = null;
        public double steerMotorPidTolerance = 0.0;
        // Swerve Module parameters.
        public String[] swerveModuleNames = null;
        public double wheelBaseWidth = 0.0;
        public double wheelBaseLength = 0.0;
        public double driveGearRatio = 0.0;
        public double steerGearRatio = 1.0;
        public double steerPositionScale = 360.0 / steerGearRatio;
        //
        // WPILib Parameters.
        //
        public boolean invertGyro = true;   // Always ensure Gyro is CCW+ CW-
        // Drivetrain Constants
        public double wheelBase = 0.0;
        public double trackWidth = 0.0;
        public double wheelCircumference = 0.0;
        // Swerve Kinematics
        // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
        public SwerveDriveKinematics swerveKinematics  = null;
        // Meters per Second
        public double maxSpeed = 0.0;
        // Radians per Second
        public double maxAngularVelocity = 0.0;
        public double driveKs = 0.0;
        public double driveKv = 0.0;
        public double driveKa = 0.0;
    }   //class SwerveInfo

    private static final String moduleName = FrcSwerveDrive.class.getSimpleName();

    public final TrcDbgTrace tracer;
    public final SwerveInfo swerveInfo;
    public final TrcEncoder[] steerEncoders;
    public final TrcMotor[] steerMotors;
    public final TrcSwerveModule[] swerveModules;
    private final FrcDashboard dashboard;
    // WPILib support.
    private final SwerveDriveOdometry swerveOdometry;
    private final SimpleMotorFeedforward driveFeedForward;

    private final double[] steerZeros = new double[4];
    private int steerZeroCalibrationCount = 0;
    private String xModeOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param swerveInfo specifies the Swerve Robot Info.
     */
    public FrcSwerveDrive(SwerveInfo swerveInfo)
    {
        super(swerveInfo);
        this.tracer = new TrcDbgTrace();
        this.swerveInfo = swerveInfo;
        // The parent class FrcRobotDrive is creating all the drive motors with generic parameters (e.g. Brake mode
        // on with VoltageComp).
        steerEncoders = createSteerEncoders();
        steerMotors = createSteerMotors();
        swerveModules = createSwerveModules();
        // WPILib support.
        swerveOdometry = new SwerveDriveOdometry(swerveInfo.swerveKinematics, getGyroAngle(), getModulePositions());
        driveFeedForward = new SimpleMotorFeedforward(swerveInfo.driveKs, swerveInfo.driveKv, swerveInfo.driveKa);

        FrcSwerveDriveBase driveBase = new FrcSwerveDriveBase(
            swerveModules[INDEX_LEFT_FRONT], swerveModules[INDEX_LEFT_BACK],
            swerveModules[INDEX_RIGHT_FRONT], swerveModules[INDEX_RIGHT_BACK],
            imu, swerveInfo.wheelBaseWidth, swerveInfo.wheelBaseLength, swerveInfo.robotMaxVelocity, swerveInfo.robotMaxTurnRate);
        if (swerveInfo.odometryType == null)
        {
            driveBase.setDriveBaseOdometry(driveBase, null, null);
        }
        super.configDriveBase(driveBase);
        this.dashboard = FrcDashboard.getInstance();
    }   //FrcSwerveDrive

    /**
     * This method creates an array of steer encoders for each steer motor and configure them.
     *
     * @return an array of created steer encoder.
     */
    private TrcEncoder[] createSteerEncoders()
    {
        TrcEncoder[] encoders = new TrcEncoder[swerveInfo.steerEncoderNames.length];

        for (int i = 0; i < encoders.length; i++)
        {
            encoders[i] = FrcEncoder.createEncoder(
                swerveInfo.steerEncoderNames[i], swerveInfo.steerEncoderIds[i],
                swerveInfo.steerEncoderType, swerveInfo.steerEncoderInverted[i]);
        }

        return encoders;
    }   //createSteerEncoders

    /**
     * This method create an array of steer motors and configure them.
     *
     * @return created array of motors.
     */
    private TrcMotor[] createSteerMotors()
    {
        TrcMotor[] motors = new TrcMotor[swerveInfo.steerMotorNames.length];

        for (int i = 0; i < motors.length; i++)
        {
            FrcMotorActuator.Params motorParams= new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    swerveInfo.steerMotorNames[i], swerveInfo.steerMotorIds[i], swerveInfo.steerMotorType,
                    swerveInfo.steerMotorBrushless, swerveInfo.steerMotorAbsEnc, swerveInfo.steerMotorInverted[i]);
            motors[i] = new FrcMotorActuator(motorParams).getMotor();

            motors[i].setBrakeModeEnabled(false);
            motors[i].setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
            motors[i].setPositionSensorScaleAndOffset(swerveInfo.steerPositionScale, 0.0);
            motors[i].setPositionPidParameters(
                swerveInfo.steerMotorPidCoeffs, null, swerveInfo.steerMotorPidTolerance, false, false, null);
        }

        return motors;
    }   //createSteerMotors

    /**
     * This method creates and configures all swerve modules.
     *
     * @return an array of created swerve modules.
     */
    private TrcSwerveModule[] createSwerveModules()
    {
        TrcSwerveModule[] modules = new TrcSwerveModule[swerveInfo.swerveModuleNames.length];

        for (int i = 0; i < modules.length; i++)
        {
            modules[i] = new TrcSwerveModule(swerveInfo.swerveModuleNames[i], driveMotors[i], steerMotors[i]);
        }

        return modules;
    }   //createSwerveModules

    /**
     * This method sets the steering angle of all swerve modules.
     *
     * @param angle specifies the steer angle.
     * @param optimize specifies true to optimize (only turns within +/- 90 degrees), false otherwse.
     * @param hold specifies true to hold the angle, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize, boolean hold)
    {
        for (TrcSwerveModule module: swerveModules)
        {
            module.setSteerAngle(angle, optimize, hold);
        }
    }   //setSteerAngle

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position. If owner
     * is specifies, it will acquire execlusive ownership of the drivebase on behalf of the specified owner. On
     * disable, it will release the ownership.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setXModeEnabled(String owner, boolean enabled)
    {
        if (enabled)
        {
            if (owner != null && !driveBase.hasOwnership(owner) && driveBase.acquireExclusiveAccess(owner))
            {
                xModeOwner = owner;
            }

            ((TrcSwerveDriveBase) driveBase).setXMode(owner);
        }
        else if (xModeOwner != null)
        {
            driveBase.releaseExclusiveAccess(xModeOwner);
            xModeOwner = null;
        }
    }   //setXModeEnabled

    /**
     * This method displays the steer zero calibration progress to the dashboard.
     *
     * @param lineNum specifies the starting line number to display the info on the dashboard.
     * @return updated line number to the next available line on the dashboard.
     */
    public int displaySteerZeroCalibration(int lineNum)
    {
        if (steerZeroCalibrationCount > 0)
        {
            dashboard.displayPrintf(
                lineNum++, "Count = %d", steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: lf=%.3f/%f",
                steerEncoders[FrcSwerveDrive.INDEX_LEFT_FRONT].getRawPosition(),
                steerZeros[FrcSwerveDrive.INDEX_LEFT_FRONT] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: rf=%.3f/%f",
                steerEncoders[FrcSwerveDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                steerZeros[FrcSwerveDrive.INDEX_RIGHT_FRONT] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: lb=%.3f/%f",
                steerEncoders[FrcSwerveDrive.INDEX_LEFT_BACK].getRawPosition(),
                steerZeros[FrcSwerveDrive.INDEX_LEFT_BACK] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: rb=%.3f/%f",
                steerEncoders[FrcSwerveDrive.INDEX_RIGHT_BACK].getRawPosition(),
                steerZeros[FrcSwerveDrive.INDEX_RIGHT_BACK] / steerZeroCalibrationCount);
        }

        return lineNum;
    }   //displaySteerZeroCalibration

    /**
     * This method starts the steering calibration.
     */
    public void startSteeringCalibration()
    {
        steerZeroCalibrationCount = 0;
        Arrays.fill(steerZeros, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     */
    public void stopSteeringCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] /= steerZeroCalibrationCount;
        }
        steerZeroCalibrationCount = 0;
        saveSteeringCalibrationData(steerZeros);
    }   //stopSteeringCalibration

    /**
     * This method is called periodically to sample the steer encoders for averaging the zero position data.
     */
    public void runSteeringCalibration()
    {
        for (int i = 0; i < steerZeros.length; i++)
        {
            steerZeros[i] += steerEncoders[i].getRawPosition();
        }
        steerZeroCalibrationCount++;
    }   //runSteeringCalibration

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     *
     * @param zeros specifies the steering zero calibration data to be saved.
     */
    public void saveSteeringCalibrationData(double[] zeros)
    {
        try (PrintStream out = new PrintStream(new FileOutputStream(swerveInfo.steerZerosFilePath)))
        {
            for (int i = 0; i < swerveInfo.steerMotorNames.length; i++)
            {
                out.println(swerveInfo.steerMotorNames[i] + ": " + zeros[i]);
            }
            out.close();
            tracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerMotorNames) +
                "=" + Arrays.toString(zeros));
        }
        catch (FileNotFoundException e)
        {
            TrcDbgTrace.printThreadStack();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    public double[] readSteeringCalibrationData()
    {
        double[] zeros;
        String line = null;

        try (Scanner in = new Scanner(new FileReader(swerveInfo.steerZerosFilePath)))
        {
            zeros = new double[steerMotors.length];

            for (int i = 0; i < steerMotors.length; i++)
            {
                line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(swerveInfo.steerMotorNames[i]))
                {
                    throw new RuntimeException("Invalid steer motor name in line " + line);
                }

                zeros[i] = Double.parseDouble(line.substring(colonPos + 1));
            }
            tracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerMotorNames) +
                "=" + Arrays.toString(zeros));
        }
        catch (FileNotFoundException e)
        {
            tracer.traceWarn(moduleName, "Steering calibration data file not found, using built-in defaults.");
            zeros = swerveInfo.steerEncoderZeros.clone();
        }
        catch (NumberFormatException e)
        {
            throw new RuntimeException("Invalid zero position value: " + line);
        }
        catch (RuntimeException e)
        {
            throw new RuntimeException("Invalid steer motor name: " + line);
        }

        return zeros;
    }   //readSteeringCalibrationData

    //
    // WPILib required methods.
    //

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, swerveInfo.maxSpeed);
        for (int i = 0; i < desiredStates.length; i++)
        {
            // Set steer angle.
            desiredStates[i].optimize(Rotation2d.fromRotations(steerMotors[i].getMotorPosition()));
            steerMotors[i].setMotorPosition(desiredStates[i].angle.getRotations(), null, 0.0, 0.0);
            // Set drive wheel speed.
            if (isOpenLoop)
            {
                double dutyCycle = desiredStates[i].speedMetersPerSecond / swerveInfo.maxSpeed;
                driveMotors[i].setMotorPower(dutyCycle);
                TrcDbgTrace.globalTraceInfo(
                    "SwerveMod" + i, "DriveSpeedOpenLoop: speed=%.3f, dutyCycle=%.3f, SteerAngle=%.3f",
                    desiredStates[i].speedMetersPerSecond, dutyCycle, desiredStates[i].angle.getRotations());
            }
            else
            {
                double velocity = Conversions.MPSToRPS(
                    desiredStates[i].speedMetersPerSecond, swerveInfo.wheelCircumference) /
                    swerveInfo.driveGearRatio;
                double feedForward = driveFeedForward.calculate(desiredStates[i].speedMetersPerSecond);
                driveMotors[i].setMotorVelocity(velocity, 0.0, feedForward);
                TrcDbgTrace.globalTraceInfo(
                    "SwerveMod" + i,
                    "DriveSpeedClosedLoop: speed=%.3f, motorVel=%.3f, feedForward=%.3f, SteerAngle=%.3f",
                    desiredStates[i].speedMetersPerSecond, velocity, feedForward, desiredStates[i].angle.getRotations());
            }
        }
    }   //setModuleStates

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        setModuleStates(desiredStates, false);
    }   //setModuleStates

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < positions.length; i++)
        {
            positions[i] = new SwerveModulePosition(
                Conversions.rotationsToMeters(driveMotors[i].getMotorPosition(), swerveInfo.wheelCircumference),
                Rotation2d.fromRotations(steerMotors[i].getMotorPosition()));
        }

        return positions;
    }   //getModulePositions

    public Pose2d getPose()
    {
        return swerveOdometry.getPoseMeters();
    }   //getPose

    public void setPose(Pose2d pose)
    {
        swerveOdometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }   //setPose

    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }   //getHeading

    public void setHeading(Rotation2d heading)
    {
        swerveOdometry.resetPosition(
            getGyroAngle(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }   //setHeading

    public void zeroHeading()
    {
        swerveOdometry.resetPosition(
            getGyroAngle(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }   //zeroHeading

    public Rotation2d getGyroAngle()
    {
        double gyroYaw = ((FrcAHRSGyro) imu).ahrs.getYaw();
        return (swerveInfo.invertGyro) ? Rotation2d.fromDegrees(360 - gyroYaw) : Rotation2d.fromDegrees(gyroYaw);
    }   //getGyroYaw

    @Override
    public void periodic()
    {
        swerveOdometry.update(getGyroAngle(), getModulePositions());
    }   //periodic

}   //class FrcSwerveDrive
