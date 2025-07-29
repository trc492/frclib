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

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frclib.motor.FrcMotorActuator;
import frclib.sensor.FrcAHRSGyro;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPose3D;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcGyro;
import trclib.sensor.TrcOdometryWheels;
import trclib.vision.TrcHomographyMapper;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class FrcRobotDrive extends SubsystemBase
{
    public static final int INDEX_LEFT_FRONT = 0;
    public static final int INDEX_RIGHT_FRONT = 1;
    public static final int INDEX_LEFT_BACK = 2;
    public static final int INDEX_RIGHT_BACK = 3;
    public static final int INDEX_LEFT_CENTER = 4;
    public static final int INDEX_RIGHT_CENTER = 5;

    public enum ImuType
    {
        NavX
    }   //enum ImuType

    /**
     * This class contains Vision parameters of a camera.
     */
    public static class VisionInfo
    {
        public String camName = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public Double camHFov = null, camVFov = null;
        public double camXOffset = 0.0, camYOffset = 0.0, camZOffset = 0.0;
        public double camYaw = 0.0, camPitch = 0.0, camRoll = 0.0;
        public Transform3d robotToCam = null;
        public TrcPose3D camPose = null;
        // The following parameters are for OpenCvVision.
        public TrcHomographyMapper.Rectangle cameraRect = null;
        public TrcHomographyMapper.Rectangle worldRect = null;
        public double camFx = 0.0, camFy = 0.0, camCx = 0.0, camCy = 0.0;
        public double aprilTagSize = 0.0;   // in inches
        public double targetZOffset = 0.0;
    }   //class VisionInfo

    /**
     * This class contains the Common Robot Info.
     */
    public static class RobotInfo
    {
        public String robotName = null;
        // Robot Dimensions
        public double robotLength = 0.0, robotWidth = 0.0;
        public double wheelBaseLength = 0.0, wheelBaseWidth = 0.0;
        // Gyro parameters.
        public String imuName = null;
        public ImuType imuType = null;
        public NavXComType imuPort = null;
        // Drive Motor parameters.
        public FrcMotorActuator.MotorType driveMotorType = null;
        public boolean driveMotorBrushless = false;
        public boolean driveMotorAbsEnc = false;
        public String[] driveMotorNames = null;
        public int[] driveMotorIds = null;
        public boolean[] driveMotorInverted = null;
        public TrcDriveBase.OdometryType odometryType = null;
        // Odometry Wheels
        public Double odWheelXScale = null;
        public Double odWheelYScale = null;
        public int[] xOdWheelIndices = null;
        public double[] xOdWheelXOffsets = null;
        public double[] xOdWheelYOffsets = null;
        public int[] yOdWheelIndices = null;
        public double[] yOdWheelXOffsets = null;
        public double[] yOdWheelYOffsets = null;
        // Absolute Odometry
        public TrcDriveBaseOdometry absoluteOdometry = null;
        public Double headingWrapRangeLow = null, headingWrapRangeHigh = null;
        // Drive Motor Odometry
        public double xDrivePosScale = 1.0, yDrivePosScale = 1.0;
        // Robot Drive Characteristics
        public Double driveMotorMaxVelocity = null;
        public Double robotMaxVelocity = null;
        public Double robotMaxAcceleration = null;
        public Double robotMaxDeceleration = null;
        public Double robotMaxTurnRate = null;
        public Double profiledMaxVelocity = robotMaxVelocity;
        public Double profiledMaxAcceleration = robotMaxAcceleration;
        public Double profiledMaxDeceleration = robotMaxDeceleration;
        public Double profiledMaxTurnRate = robotMaxTurnRate;
        // DriveBase PID Parameters
        public double drivePidTolerance = 0.0, turnPidTolerance = 0.0;
        public TrcPidController.PidCoefficients xDrivePidCoeffs = null;
        public double xDrivePidPowerLimit = 1.0;
        public Double xDriveMaxPidRampRate = null;
        public TrcPidController.PidCoefficients yDrivePidCoeffs = null;
        public double yDrivePidPowerLimit = 1.0;
        public Double yDriveMaxPidRampRate = null;
        public TrcPidController.PidCoefficients turnPidCoeffs = null;
        public double turnPidPowerLimit = 1.0;
        public Double turnMaxPidRampRate = null;
        // PID Stall Detection
        public boolean pidStallDetectionEnabled = false;
        // PidDrive Parameters
        public boolean usePidDrive = false;
        public boolean enablePidDriveSquareRootPid = false;
        // PurePursuit Parameters
        public boolean usePurePursuitDrive = false;
        public boolean enablePurePursuitDriveSquareRootPid = false;
        public double ppdFollowingDistance = 0.0;
        public TrcPidController.PidCoefficients velPidCoeffs = null;
        public boolean fastModeEnabled = true;
        // Vision
        public VisionInfo cam1 = null;
        public VisionInfo cam2 = null;
        // Miscellaneous
        public String ledName = null;
        public int ledChannel = -1;
        public int numLEDs = 0;
    }   //class RobotInfo

    public final RobotInfo robotInfo;
    public final TrcGyro imu;
    public final TrcMotor[] driveMotors;
    public TrcDriveBase driveBase = null;
    public TrcPidDrive pidDrive = null;
    public TrcPurePursuitDrive purePursuitDrive = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotInfo specifies the Robot Info.
     */
    public FrcRobotDrive(RobotInfo robotInfo)
    {
        super();
        this.robotInfo = robotInfo;
        imu = robotInfo.imuName != null? new FrcAHRSGyro(robotInfo.imuName, robotInfo.imuPort) : null;
        driveMotors = new TrcMotor[robotInfo.driveMotorNames.length];
        for (int i = 0; i < driveMotors.length; i++)
        {
            FrcMotorActuator.Params motorParams= new FrcMotorActuator.Params()
                .setPrimaryMotor(robotInfo.driveMotorNames[i], robotInfo.driveMotorIds[i],
                                 robotInfo.driveMotorType, robotInfo.driveMotorBrushless,
                                 robotInfo.driveMotorAbsEnc, robotInfo.driveMotorInverted[i]);
            driveMotors[i] = new FrcMotorActuator(motorParams).getMotor();
        }
    }   //FrcRobotDrive

    /**
     * This method cancels any drive operation still in progress.
     *
     * @param owner specifies the owner that requested the cancel.
     */
    public void cancel(String owner)
    {
        if (pidDrive != null && pidDrive.isActive())
        {
            pidDrive.cancel(owner);
        }

        if (purePursuitDrive != null && purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel(owner);
        }

        driveBase.stop(owner);
    }   //cancel

    /**
     * This method cancels any drive operation still in progress.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method configures the rest of drive base after it has been created.
     *
     * @param driveBase specifies the created drive base.
     */
    protected void configDriveBase(TrcDriveBase driveBase)
    {
        boolean supportHolonomic = driveBase.supportsHolonomicDrive();
        TrcPidController pidCtrl;

        this.driveBase = driveBase;
        driveBase.setMotorVelocityControlEnabled(robotInfo.driveMotorMaxVelocity);
        // Create and initialize PID controllers.
        if (robotInfo.usePidDrive)
        {
            if (supportHolonomic)
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.xDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getXPosition,
                    robotInfo.yDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getYPosition,
                    robotInfo.turnPidCoeffs, robotInfo.turnPidTolerance, driveBase::getHeading);
                pidCtrl = pidDrive.getXPidCtrl();
                pidCtrl.setOutputLimit(robotInfo.xDrivePidPowerLimit);
                pidCtrl.setRampRate(robotInfo.xDriveMaxPidRampRate);
            }
            else
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.yDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getYPosition,
                    robotInfo.turnPidCoeffs, robotInfo.turnPidTolerance, driveBase::getHeading);
            }

            pidCtrl = pidDrive.getYPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.yDrivePidPowerLimit);
            pidCtrl.setRampRate(robotInfo.yDriveMaxPidRampRate);

            pidCtrl = pidDrive.getTurnPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.turnPidPowerLimit);
            pidCtrl.setRampRate(robotInfo.turnMaxPidRampRate);
            pidCtrl.setAbsoluteSetPoint(true);
            // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
            // of the absolute target position.
            pidDrive.setAbsoluteTargetModeEnabled(true);
            pidDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            pidDrive.setSquidModeEnabled(robotInfo.enablePidDriveSquareRootPid);
        }

        if (robotInfo.usePurePursuitDrive)
        {
            purePursuitDrive = new TrcPurePursuitDrive(
                "purePursuitDrive", driveBase,
                robotInfo.ppdFollowingDistance, robotInfo.drivePidTolerance, robotInfo.turnPidTolerance,
                robotInfo.xDrivePidCoeffs, robotInfo.yDrivePidCoeffs, robotInfo.turnPidCoeffs, robotInfo.velPidCoeffs);
            purePursuitDrive.setMoveOutputLimit(robotInfo.yDrivePidPowerLimit);
            purePursuitDrive.setRotOutputLimit(robotInfo.turnPidPowerLimit);
            purePursuitDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            purePursuitDrive.setSquidModeEnabled(robotInfo.enablePurePursuitDriveSquareRootPid);
            purePursuitDrive.setFastModeEnabled(robotInfo.fastModeEnabled);
        }

        if (robotInfo.odometryType == TrcDriveBase.OdometryType.OdometryWheels)
        {
            TrcOdometryWheels.AxisSensor[] xOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.xOdWheelIndices.length];
            for (int i = 0; i < robotInfo.xOdWheelIndices.length; i++)
            {
                xOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                    driveMotors[robotInfo.xOdWheelIndices[i]],
                    robotInfo.xOdWheelXOffsets[i], robotInfo.xOdWheelYOffsets[i]);
            }
            TrcOdometryWheels.AxisSensor[] yOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.yOdWheelIndices.length];
            for (int i = 0; i < robotInfo.yOdWheelIndices.length; i++)
            {
                yOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                    driveMotors[robotInfo.yOdWheelIndices[i]],
                    robotInfo.yOdWheelXOffsets[i], robotInfo.yOdWheelYOffsets[i]);
            }
            // Set the drive base to use the external odometry device overriding the built-in one.
            driveBase.setDriveBaseOdometry(new TrcOdometryWheels(xOdWheelSensors, yOdWheelSensors, imu));
            driveBase.setOdometryScales(robotInfo.odWheelXScale, robotInfo.odWheelYScale);
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
        {
            // SparkFun OTOS scales are already set when it was created.
            driveBase.setDriveBaseOdometry(
                robotInfo.absoluteOdometry, robotInfo.headingWrapRangeLow, robotInfo.headingWrapRangeHigh);
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.MotorOdometry)
        {
            // Set drive base odometry to built-in motor odometry and reset their encoders.
            driveBase.setDriveBaseOdometry(true);
            driveBase.setOdometryScales(robotInfo.xDrivePosScale, robotInfo.yDrivePosScale);
        }

        // if (robotInfo.xTippingPidCoeffs != null && robotInfo.yTippingPidCoeffs)
        // {
        //     driveBase.enableAntiTipping(
        //         robotInfo.xTippingPidCoeffs, robotInfo.xTippingPidTolerance, this::getGyroRoll,
        //         robotInfo.yTippingPidCoeffs, robotInfo.yTippingPidTolerance, this::getGyroPitch);
        // }
        // else if (robotInfo.yTippingPidCoeffs)
        // {
        //     driveBase.enableAntiTipping(
        //         robotInfo.yTippingPidCoeffs, robotInfo.yTippingPidTolerance, this::getGyroPitch);
        // }
        // if (robot.pdp != null)
        // {
        //     robot.pdp.registerEnergyUsed(
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LFDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LBDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RFDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RBDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LFSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LBSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RFSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RBSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_BACK]));
        // }
    }   //configDriveBase

    /**
     * This method returns the gyro pitch.
     *
     * @return gyro pitch.
     */
    public double getGyroPitch()
    {
        return imu != null? imu.getXHeading().value: 0.0;
    }   //getGyroPitch

    /**
     * This method returns the gyro roll.
     *
     * @return gyro roll.
     */
    public double getGyroRoll()
    {
        return imu != null? imu.getYHeading().value: 0.0;
    }   //getGyroRoll

    /**
     * This method returns the gyro yaw.
     *
     * @return gyro yaw.
     */
    public double getGyroYaw()
    {
        return imu != null? imu.getZHeading().value: 0.0;
    }   //getGyroYaw

}   //class FrcRobotDrive
