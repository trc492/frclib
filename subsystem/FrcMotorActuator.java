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

package frclib.subsystem;

import java.util.Arrays;

import frclib.motor.FrcMotor;
import frclib.motor.FrcMotor.MotorType;
import frclib.sensor.FrcAnalogEncoder;
import frclib.sensor.FrcDigitalInput;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor or a continuous
 * rotation servo, optionally a lower limit switch, an upper limit switch and an encoder. It creates all the necessary
 * components for a PID controlled actuator which could include a software PID controller.
 */
public class FrcMotorActuator
{
    /**
     * This class contains all the parameters related to the actuator.
     */
    public static class Params
    {
        public boolean motorInverted = false;
        public TrcMotor followerMotor = null;
        public boolean followerMotorInverted = false;
        public int lowerLimitSwitchChannel = -1;
        public boolean lowerLimitSwitchInverted = false;
        public int upperLimitSwitchChannel = -1;
        public boolean upperLimitSwitchInverted = false;
        public int externalEncoderChannel = -1;
        public boolean encoderInverted = false; //???
        public boolean voltageCompensationEnabled = false;
        public double positionScale = 1.0;
        public double positionOffset = 0.0;
        public double positionZeroOffset = 0.0;
        public double[] positionPresets = null;
        public double positionPresetTolerance = 0.0;

        /**
         * This methods sets the motor direction.
         *
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setMotorInverted(boolean inverted)
        {
            motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This methods sets the follower motor if there is one and also sets its direction.
         *
         * @param hasFollowerMotor specifies true if there is a follower motor, false otherwise.
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(TrcMotor followerMotor, boolean inverted)
        {
            this.followerMotor = followerMotor;
            this.followerMotorInverted = inverted;
            return this;
        }   //setFollowerMotor

        /**
         * This method sets the lower limit switch properties.
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
         * This method sets the upper limit switch properties.
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
         * This method sets whether the actuator has an external encoder.
         *
         * @param channel specifies analog channel if there is an external encoder, -1 otherwise.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(int channel, boolean inverted)
        {
            externalEncoderChannel = channel;
            encoderInverted = inverted;
            return this;
        }   //setExternalEncoder

        /**
         * This method enables/disables voltage compensation on the actuator motor.
         *
         * @param enabled specifies true to enable voltage compensation, false to disable.
         * @return this object for chaining.
         */
        public Params setVoltageCompensationEnabled(boolean enabled)
        {
            this.voltageCompensationEnabled = enabled;
            return this;
        }   //setVoltageCompensationEnabled

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
         * This method sets an array of preset positions for the motor actuator.
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

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "motorInverted=" + motorInverted +
                   ",followerMotor=" + followerMotor +
                   ",followerInverted=" + followerMotorInverted +
                   ",lowerLimitChannel=" + lowerLimitSwitchChannel +
                   ",lowerLimitInverted=" + lowerLimitSwitchInverted +
                   ",upperLimitChannel=" + upperLimitSwitchChannel +
                   ",upperLimitInverted=" + upperLimitSwitchInverted +
                   ",encoderChannel=" + externalEncoderChannel +
                   ",encoderInverted=" + encoderInverted +
                   ",voltageCompEnabled=" + voltageCompensationEnabled +
                   ",posScale=" + positionScale +
                   ",posOffset=" + positionOffset +
                   ",posZeroOffset=" + positionZeroOffset +
                   ",posPresets=" + Arrays.toString(positionPresets);
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final TrcMotor actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motorType specifies the motor type.
     * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor or CRServo).
     * @param brushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
     * @param absEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
     *        applicable for SparkMax).
     * @param params specifies the parameters to set up the actuator.
     */
    public FrcMotorActuator(
        String instanceName, MotorType motorType, int motorId, boolean brushless, boolean absEnc, Params params)
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

        this.instanceName = instanceName;
        actuator = FrcMotor.createMotor(
            instanceName, motorId, motorType, brushless, absEnc, lowerLimitSwitch, upperLimitSwitch,
            encoder != null? encoder.getAbsoluteEncoder(): null);

        if (params.followerMotor != null)
        {
            params.followerMotor.follow(actuator, params.motorInverted != params.followerMotorInverted);
        }

        if (lowerLimitSwitch != null)
        {
            actuator.enableLowerLimitSwitch(params.lowerLimitSwitchInverted);
        }

        if (upperLimitSwitch != null)
        {
            actuator.enableUpperLimitSwitch(params.upperLimitSwitchInverted);
        }

        if (motorType == MotorType.CRServo)
        {
            // CRServo does not support native PID control, use software PID instead.
            actuator.setSoftwarePidEnabled(true);
        }
        else
        {
            actuator.setBrakeModeEnabled(true);
        }

        if (params.voltageCompensationEnabled)
        {
            actuator.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        }

        actuator.setOdometryEnabled(true, true, true);
        actuator.setMotorInverted(params.motorInverted);
        actuator.setPositionSensorScaleAndOffset(
            params.positionScale, params.positionOffset, params.positionZeroOffset);
        actuator.setPresets(false, params.positionPresetTolerance, params.positionPresets);
    }   //FrcMotorActuator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the actuator object.
     *
     * @return actuator object.
     */
    public TrcMotor getActuator()
    {
        return actuator;
    }   //getActuator

}   //class FrcMotorActuator
