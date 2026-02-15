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

 package frclib.motor;

import java.util.ArrayList;
import java.util.Arrays;

import frclib.sensor.FrcDigitalInput;
import frclib.sensor.FrcEncoder;
import frclib.sensor.FrcEncoder.EncoderType;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcDigitalInput;
import trclib.sensor.TrcEncoder;

/**
 * This class creates an FRC platform specific motor with the specified parameters.
 */
public class FrcMotorActuator
{
    public enum MotorType
    {
        CanTalonFx,
        CanTalonSrx,
        CanSparkMax,
        PwmTalonFx,
        PwmTalonSrx,
        PwmSparkMax,
        PwmVictorSpx,
        CRServo
    }   //enum MotorType

    /**
     * This class contains all the extra parameters for SparkMax motors.
     */
    public static class SparkMaxMotorParams
    {
        public boolean brushless;
        public boolean absEnc;

        public SparkMaxMotorParams(boolean brushless, boolean absEnc)
        {
            this.brushless = brushless;
            this.absEnc = absEnc;
        }   //SparkMaxMotorParams

        @Override
        public String toString()
        {
            return "(brushless=" + brushless + ", absEnc=" + absEnc + ")";
        }   //toString

    }   //SparkMaxMotorParams

    public static class MotorInfo
    {
        public String name = null;
        public MotorType motorType = null;
        public boolean inverted = false;
        public boolean voltageCompEnabled = false;
        public Boolean brakeModeEnabled = null;
        public int motorId = -1;
        public String canBusName = null;
        public SparkMaxMotorParams sparkMaxParams = null;

        public MotorInfo(
            String name, MotorType motorType, boolean inverted, boolean voltageCompEnabled, Boolean brakeModeEnabled,
            int motorId, String canBusName, SparkMaxMotorParams sparkMaxParams)
        {
            this.name = name;
            this.motorType = motorType;
            this.inverted = inverted;
            this.voltageCompEnabled = voltageCompEnabled;
            this.brakeModeEnabled = brakeModeEnabled;
            this.motorId = motorId;
            this.canBusName = canBusName;
            this.sparkMaxParams = sparkMaxParams;
        }   //MotorInfo

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "name=" + name +
                   ", type=" + motorType +
                   ", inverted=" + inverted +
                   ", voltageComp=" + voltageCompEnabled +
                   ", brakeMode=" + brakeModeEnabled +
                   ", motorId=" + motorId +
                   ", canBus=" + canBusName +
                   ", sparkMaxParams=" + sparkMaxParams;
        }   //toString
    }   //class MotorInfo

    /**
     * This class contains all the parameters for creating the motor.
     */
    public static class Params
    {
        public MotorInfo primaryMotor = null;
        public ArrayList<MotorInfo> followerMotors = null;

        public String lowerLimitSwitchName = null;
        public int lowerLimitSwitchChannel = -1;
        public boolean lowerLimitSwitchInverted = false;

        public String upperLimitSwitchName = null;
        public int upperLimitSwitchChannel = -1;
        public boolean upperLimitSwitchInverted = false;

        public TrcEncoder externalEncoder = null;
        public String externalEncoderName = null;
        public boolean externalEncoderInverted = false;
        public String externalEncoderCanBusName = null;
        public EncoderType externalEncoderType = null;
        public int externalEncoderChannel = -1;
        public boolean externalEncoderWrapped = true;

        public double positionScale = 1.0;
        public double positionOffset = 0.0;
        public double positionZeroOffset = 0.0;

        public double[] positionPresets = null;
        public double positionPresetTolerance = 0.0;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "primaryMotor=(" + primaryMotor +
                   ")\nfollowerMotors=" + followerMotors +
                   "\nlowerLimitName=" + lowerLimitSwitchName +
                   ",lowerLimitChannel=" + lowerLimitSwitchChannel +
                   ",lowerLimitInverted=" + lowerLimitSwitchInverted +
                   "\nupperLimitName=" + upperLimitSwitchName +
                   ",upperLimitChannel=" + upperLimitSwitchChannel +
                   ",upperLimitInverted=" + upperLimitSwitchInverted +
                   "\nencoderName=" + externalEncoderName +
                   ",encoderInverted=" + externalEncoderInverted +
                   ",encoderType=" + externalEncoderType +
                   ",encoderChannel=" + externalEncoderChannel +
                   ",encoderWrapped=" + externalEncoderWrapped +
                   "\nposScale=" + positionScale +
                   ",posOffset=" + positionOffset +
                   ",posZeroOffset=" + positionZeroOffset +
                   "\nposPresets=" + Arrays.toString(positionPresets) +
                   ",posPresetTolerance=" + positionPresetTolerance;
        }   //toString

        /**
         * This method sets the parameters of the primary motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param voltageCompEnabled specifies true to enable voltage compensation, false otherwise.
         * @param brakeModeEnabled specifies true to enable brake mode, false for coast mode. Can be null if motor
         *        does not support brake mode.
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param sparkMaxParams specifies extra parameters for SparkMax motor, null if motor type is not SparkMax.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(
            String name, MotorType motorType, boolean inverted, boolean voltageCompEnabled, Boolean brakeModeEnabled,
            int motorId, String canBusName, SparkMaxMotorParams sparkMaxParams)
        {
            if (motorId == -1)
            {
                throw new IllegalArgumentException("Must provide a valid primary motor ID.");
            }

            if (primaryMotor != null)
            {
                throw new IllegalStateException("Primary motor is already set.");
            }

            primaryMotor = new MotorInfo(
                name, motorType, inverted, voltageCompEnabled, brakeModeEnabled, motorId, canBusName, sparkMaxParams);
            return this;
        }   //setPrimaryMotor

        /**
         * This method sets the parameters of the follower motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param sparkMaxParams specifies extra parameters for SparkMax motor, null if motor type is not SparkMax.
         * @return this object for chaining.
         */
        public Params addFollowerMotor(
            String name, MotorType motorType, boolean inverted, int motorId, String canBusName,
            SparkMaxMotorParams sparkMaxParams)
        {
            if (primaryMotor == null)
            {
                throw new IllegalStateException("Must set the primary motor first.");
            }

            if (followerMotors == null)
            {
                followerMotors = new ArrayList<>();
            }

            followerMotors.add(
                new MotorInfo(name, motorType, inverted, false, null, motorId, canBusName, sparkMaxParams));
            return this;
        }   //addFollowerMotor

        /**
         * This method sets the lower limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param channel specifies the digital input channel if there is a lower limit switch, -1 otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(String name, int channel, boolean inverted)
        {
            this.lowerLimitSwitchName = name;
            this.lowerLimitSwitchChannel = channel;
            this.lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param channel specifies the digital input channel if there is a lower limit switch, -1 otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(String name, int channel, boolean inverted)
        {
            this.upperLimitSwitchName = name;
            this.upperLimitSwitchChannel = channel;
            this.upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitch

        /**
         * This method sets the external encoder parameters.
         *
         * @param encoder specifies the external analog encoder.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(TrcEncoder encoder)
        {
            if (this.externalEncoderName != null)
            {
                throw new IllegalStateException("Can only specify encoder or encode name but not both.");
            }
            this.externalEncoder = encoder;
            return this;
        }   //setExternalEncoder

        /**
         * This method sets the external encoder parameters.
         *
         * @param name specifies the name of the encoder.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @param type specifies the encoder type.
         * @param channel specifies channel/ID if there is an external encoder, -1 otherwise.
         * @param wrapped specifies true if the encoder value is wrapped, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(String name, boolean inverted, EncoderType type, int channel, boolean wrapped)
        {
            if (this.externalEncoder != null)
            {
                throw new IllegalStateException("Can only specify encoder or encode name but not both.");
            }
            this.externalEncoderName = name;
            this.externalEncoderInverted = inverted;
            this.externalEncoderType = type;
            this.externalEncoderChannel = channel;
            this.externalEncoderWrapped = wrapped;
            return this;
        }   //setExternalEncoder

        /**
         * This method sets the external encoder parameters.
         *
         * @param name specifies the name of the encoder.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @param type specifies the encoder type.
         * @param channel specifies channel/ID if there is an external encoder, -1 otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(String name, boolean inverted, EncoderType type, int channel)
        {
            return setExternalEncoder(name, inverted, type, channel, true);
        }   //setExternalEncoder

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @param zeroOffset specifies the zero offset for absolute encoder.
         * @return this object for chaining.
         */
        public Params setPositionScaleAndOffset(double scale, double offset, double zeroOffset)
        {
            positionScale = scale;
            positionOffset = offset;
            positionZeroOffset = zeroOffset;
            return this;
        }   //setPositionScaleAndOffset

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @return this object for chaining.
         */
        public Params setPositionScaleAndOffset(double scale, double offset)
        {
            return setPositionScaleAndOffset(scale, offset, 0.0);
        }   //setPositionScaleAndOffset

        /**
         * This method sets an array of preset positions.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public Params setPositionPresets(double tolerance, double... posPresets)
        {
            positionPresets = posPresets;
            positionPresetTolerance = tolerance;
            return this;
        }   //setPositionPresets

    }   //class Params

    private final TrcMotor motor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the motor parameters.
     */
    public FrcMotorActuator(Params params)
    {
        TrcDigitalInput lowerLimitSwitch =
            params.lowerLimitSwitchChannel != -1?
                new FrcDigitalInput(params.lowerLimitSwitchName, params.lowerLimitSwitchChannel): null;
        TrcDigitalInput upperLimitSwitch =
            params.upperLimitSwitchChannel != -1?
                new FrcDigitalInput(params.upperLimitSwitchName, params.upperLimitSwitchChannel): null;
        TrcEncoder encoder =
            params.externalEncoderChannel == -1?
                params.externalEncoder:
                FrcEncoder.createEncoder(
                    params.externalEncoderName, params.externalEncoderChannel, params.externalEncoderType,
                    params.externalEncoderInverted, params.externalEncoderCanBusName);

        TrcMotor.ExternalSensors sensors = null;
        if (lowerLimitSwitch != null || upperLimitSwitch != null || encoder != null)
        {
            sensors = new TrcMotor.ExternalSensors();

            if (lowerLimitSwitch != null)
            {
                sensors.setLowerLimitSwitch(lowerLimitSwitch, params.lowerLimitSwitchInverted);
            }

            if (upperLimitSwitch != null)
            {
                sensors.setUpperLimitSwitch(upperLimitSwitch, params.upperLimitSwitchInverted);
            }

            if (encoder != null)
            {
                sensors.setEncoder(encoder, params.externalEncoderInverted);
            }
        }

        motor = createMotor(params.primaryMotor, sensors);

        if (params.followerMotors != null)
        {
            for (MotorInfo motorInfo: params.followerMotors)
            {
                TrcMotor followerMotor = createMotor(motorInfo, null);
                // If motor natively supports following, it will override the follow method.
                followerMotor.follow(motor, params.primaryMotor.inverted != motorInfo.inverted);
            }
        }

        motor.setPositionSensorScaleAndOffset(params.positionScale, params.positionOffset, params.positionZeroOffset);

        if (params.positionPresets != null)
        {
            motor.setPresets(false, params.positionPresetTolerance, params.positionPresets);
        }
    }   //FrcMotorActuator

    /**
     * This method returns the created primary motor.
     *
     * @return primary motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method creates a motor with the specified parameters and initializes it.
     *
     * @param motorInfo specifies the motor info.
     * @param sensors specifies external sensors, can be null if none.
     * @return created motor.
     */
    public TrcMotor createMotor(MotorInfo motorInfo, TrcMotor.ExternalSensors sensors)
    {
        TrcMotor motor;

        switch (motorInfo.motorType)
        {
            case CanTalonFx:
                motor = new FrcCANTalonFX(motorInfo.name, motorInfo.motorId, motorInfo.canBusName, sensors);
                motor.resetFactoryDefault();
                break;
            
            case CanTalonSrx:
                motor = new FrcCANTalonSRX(motorInfo.name, motorInfo.motorId, sensors);
                motor.resetFactoryDefault();
                break;

            case CanSparkMax:
                motor = new FrcCANSparkMax(
                    motorInfo.name, motorInfo.motorId,
                    motorInfo.sparkMaxParams != null && motorInfo.sparkMaxParams.brushless,
                    motorInfo.sparkMaxParams != null && motorInfo.sparkMaxParams.absEnc, sensors);
                motor.resetFactoryDefault();
                break;

            case PwmTalonFx:
                motor = new FrcPWMTalonFX(motorInfo.name, motorInfo.motorId, sensors);
                break;

            case PwmTalonSrx:
                motor = new FrcPWMTalonSRX(motorInfo.name, motorInfo.motorId, sensors);
                break;

            case PwmSparkMax:
                motor = new FrcPWMSparkMax(motorInfo.name, motorInfo.motorId, sensors);
                break;

            case PwmVictorSpx:
                motor = new FrcPWMVictorSPX(motorInfo.name, motorInfo.motorId, sensors);
                break;

            case CRServo:
                motor = new FrcCRServo(motorInfo.name, motorInfo.motorId, sensors);
                break;

            default:
                motor = null;
                break;
        }

        if (motor != null)
        {
            motor.setMotorInverted(motorInfo.inverted);

            if (motorInfo.voltageCompEnabled)
            {
                motor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
            }

            if (motorInfo.brakeModeEnabled != null)
            {
                motor.setBrakeModeEnabled(motorInfo.brakeModeEnabled);
            }
        }

        return motor;
    }   //createMotor

}   //class FrcMotorActuator
