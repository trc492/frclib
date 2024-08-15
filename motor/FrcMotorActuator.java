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

import java.util.Arrays;

import frclib.sensor.FrcAnalogEncoder;
import frclib.sensor.FrcDigitalInput;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;

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
     * This class contains all the parameters for creating the motor.
     */
    public static class Params
    {
        public int primaryMotorId = -1;
        public MotorType primaryMotorType = null;
        public boolean primaryMotorBrushless = false;
        public boolean primaryMotorAbsEnc = false;
        public boolean primaryMotorInverted = false;

        public int followerMotorId = -1;
        public MotorType followerMotorType = null;
        public boolean followerMotorBrushless = false;
        public boolean followerMotorAbsEnc = false;
        public boolean followerMotorInverted = false;

        public int lowerLimitSwitchChannel = -1;
        public boolean lowerLimitSwitchInverted = false;

        public int upperLimitSwitchChannel = -1;
        public boolean upperLimitSwitchInverted = false;

        public int externalEncoderChannel = -1;
        public boolean externalEncoderInverted = false;

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
            return "primaryMotorId=" + primaryMotorId +
                   ",primaryMotorType=" + primaryMotorType +
                   ",primaryMotorBrushless=" + primaryMotorBrushless +
                   ",primaryMotorAbsEnc=" + primaryMotorAbsEnc +
                   ",primaryMotorInverted=" + primaryMotorInverted +
                   "\nfollowerMotorId=" + primaryMotorId +
                   ",followerMotorType=" + primaryMotorType +
                   ",followerMotorBrushless=" + primaryMotorBrushless +
                   ",followerMotorAbsEnc=" + primaryMotorAbsEnc +
                   ",followerMotorInverted=" + primaryMotorInverted +
                   "\nlowerLimitChannel=" + lowerLimitSwitchChannel +
                   ",lowerLimitInverted=" + lowerLimitSwitchInverted +
                   "\nupperLimitChannel=" + upperLimitSwitchChannel +
                   ",upperLimitInverted=" + upperLimitSwitchInverted +
                   "\nencoderChannel=" + externalEncoderChannel +
                   ",encoderInverted=" + externalEncoderInverted +
                   "\nposScale=" + positionScale +
                   ",posOffset=" + positionOffset +
                   ",posZeroOffset=" + positionZeroOffset +
                   "\nposPresets=" + Arrays.toString(positionPresets) +
                   ",posPresetTolerance=" + positionPresetTolerance;
        }   //toString

        /**
         * This method sets the parameters of the primary motor.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(
            int motorId, MotorType motorType, boolean brushless, boolean absEnc, boolean inverted)
        {
            if (motorId == -1)
            {
                throw new IllegalArgumentException("Must provide a valid primary motor ID.");
            }

            this.primaryMotorId = motorId;
            this.primaryMotorType = motorType;
            this.primaryMotorBrushless = brushless;
            this.primaryMotorAbsEnc = absEnc;
            this.primaryMotorInverted = inverted;
            return this;
        }   //setPrimaryMotor

        /**
         * This method sets the parameters of the follower motor.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(
            int motorId, MotorType motorType, boolean brushless, boolean absEnc, boolean inverted)
        {
            this.followerMotorId = motorId;
            this.followerMotorType = motorType;
            this.followerMotorBrushless = brushless;
            this.followerMotorAbsEnc = absEnc;
            this.followerMotorInverted = inverted;
            return this;
        }   //setFollowerMotor

        /**
         * This method sets the lower limit switch parameters.
         *
         * @param channel specifies the digital input channel if there is a lower limit switch, -1 otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(int channel, boolean inverted)
        {
            lowerLimitSwitchChannel = channel;
            lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch parameters.
         *
         * @param channel specifies the digital input channel if there is a lower limit switch, -1 otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(int channel, boolean inverted)
        {
            upperLimitSwitchChannel = channel;
            upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitch

        /**
         * This method sets the external encoder parameters.
         *
         * @param channel specifies analog channel if there is an external encoder, -1 otherwise.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(int channel, boolean inverted)
        {
            externalEncoderChannel = channel;
            externalEncoderInverted = inverted;
            return this;
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

    private final TrcMotor primaryMotor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the motor parameters.
     */
    public FrcMotorActuator(String instanceName, Params params)
    {
        FrcDigitalInput lowerLimitSwitch =
            params.lowerLimitSwitchChannel != -1?
                new FrcDigitalInput(instanceName + ".lowerLimit", params.lowerLimitSwitchChannel): null;
        FrcDigitalInput upperLimitSwitch =
            params.upperLimitSwitchChannel != -1?
                new FrcDigitalInput(instanceName + ".upperLimit", params.upperLimitSwitchChannel): null;
        FrcAnalogEncoder encoder =
            params.externalEncoderChannel != -1?
                new FrcAnalogEncoder(instanceName + ".encoder", params.externalEncoderChannel): null;

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
                sensors.setEncoder(encoder.getAbsoluteEncoder(), params.externalEncoderInverted);
            }
        }

        primaryMotor = createMotor(
            instanceName + ".primary", params.primaryMotorId, params.primaryMotorType,
            params.primaryMotorBrushless, params.primaryMotorAbsEnc, sensors);
        primaryMotor.setMotorInverted(params.primaryMotorInverted);

        if (params.followerMotorId != -1)
        {
            TrcMotor followerMotor = createMotor(
                instanceName + ".follower", params.followerMotorId, params.followerMotorType,
                params.followerMotorBrushless, params.followerMotorAbsEnc, null);
            followerMotor.follow(primaryMotor, params.primaryMotorInverted != params.followerMotorInverted);
        }

        primaryMotor.setPositionSensorScaleAndOffset(
            params.positionScale, params.positionOffset, params.positionZeroOffset);

        if (params.positionPresets != null)
        {
            primaryMotor.setPresets(false, params.positionPresetTolerance, params.positionPresets);
        }
    }   //FrcMotorActuator

    /**
     * This method returns the created primary motor.
     *
     * @return primary motor.
     */
    public TrcMotor getMotor()
    {
        return primaryMotor;
    }   //getMotor

    /**
     * This method creates a motor with the specified parameters and initializes it.
     *
     * @param name specifies the instance name of the motor.
     * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
     * @param motorType specifies the motor type.
     * @param brushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
     * @param absEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
     *        applicable for SparkMax).
     * @param sensors specifies external sensors, can be null if none.
     * @return created motor.
     */
    private TrcMotor createMotor(
        String name, int motorId, MotorType motorType, boolean brushless, boolean absEnc,
        TrcMotor.ExternalSensors sensors)
    {
        TrcMotor motor;

        switch (motorType)
        {
            case CanTalonFx:
                motor = new FrcCANTalonFX(name, motorId, sensors);
                break;
            
            case CanTalonSrx:
                motor = new FrcCANTalonSRX(name, motorId, sensors);
                break;

            case CanSparkMax:
                motor = new FrcCANSparkMax(name, motorId, brushless, absEnc, sensors);
                break;

            case PwmTalonFx:
                motor = new FrcPWMTalonFX(name, motorId, sensors);
                break;

            case PwmTalonSrx:
                motor = new FrcPWMTalonSRX(name, motorId, sensors);
                break;

            case PwmSparkMax:
                motor = new FrcPWMSparkMax(name, motorId, sensors);
                break;

            case PwmVictorSpx:
                motor = new FrcPWMVictorSPX(name, motorId, sensors);
                break;

            case CRServo:
                motor = new FrcCRServo(name, motorId, sensors);
                break;

            default:
                motor = null;
                break;
        }
        motor.resetFactoryDefault();
        motor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        motor.setBrakeModeEnabled(true);

        return motor;
    }   //createMotor

}   //class FrcMotorActuator
