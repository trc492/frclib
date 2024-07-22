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

import frclib.motor.FrcMotor;
import frclib.motor.FrcMotor.MotorType;
import frclib.sensor.FrcAnalogInput;
import frclib.sensor.FrcDigitalInput;
import trclib.dataprocessor.TrcTrigger;
import trclib.dataprocessor.TrcTriggerDigitalInput;
import trclib.dataprocessor.TrcTriggerThresholdZones;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcIntake;;

/**
 * This class implements a platform dependent Smart Intake Subsystem. An Intake consists of a DC motor or a continuous
 * rotation servo. Optionally, it may have entry and exit sensors to detect the game element entering or exiting the
 * Intake and allows callback actions such as stopping the Intake motor.
 */
public class FrcIntake
{
    public enum SensorType
    {
        DigitalSensor,
        AnalogSensor
    }   //enum SensorType

    /**
     * This class contains all the parameters related to the Intake.
     */
    public static class Params
    {
        public boolean motorInverted = false;
        public TrcMotor followerMotor = null;
        public boolean followerMotorInverted = false;
        public boolean voltageCompensationEnabled = false;
        public SensorType entrySensorType = null;
        public int entrySensorChannel = -1;
        public boolean entrySensorInverted = false;
        public double entrySensorThreshold = 0.0;
        public TrcEvent.Callback entryTriggerCallback = null;
        public SensorType exitSensorType = null;
        public int exitSensorChannel = -1;
        public boolean exitSensorInverted = false;
        public double exitSensorThreshold = 0.0;
        public TrcEvent.Callback exitTriggerCallback = null;

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
         * This method specifies the entry sensor type and parameters if there is one.
         *
         * @param sensorType specifies the sensor type, null if there is no sensor.
         * @param channel specifies the channel number the sensor is plugged into (analog or digital input).
         * @param inverted specifies true if the sensor polarity is inverted.
         * @param threshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
         *        digital.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setEntrySensor(
            SensorType sensorType, int channel, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.entrySensorType = sensorType;
            this.entrySensorChannel = channel;
            this.entrySensorInverted = inverted;
            this.entrySensorThreshold = threshold;
            this.entryTriggerCallback = triggerCallback;
            return this;
        }   //setEntrySensor

        /**
         * This method specifies the exit sensor type and parameters if there is one.
         *
         * @param sensorType specifies the sensor type, null if there is no sensor.
         * @param channel specifies the channel number the sensor is plugged into (analog or digital input).
         * @param inverted specifies true if the sensor polarity is inverted.
         * @param threshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
         *        digital.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setExitSensor(
            SensorType sensorType, int channel, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.exitSensorType = sensorType;
            this.entrySensorChannel = channel;
            this.exitSensorInverted = inverted;
            this.exitSensorThreshold = threshold;
            this.exitTriggerCallback = triggerCallback;
            return this;
        }   //setExitSensor

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
                   ",voltageCompEnabled=" + voltageCompensationEnabled +
                   ",entrySensorType=" + entrySensorType +
                   ",entrySensorChannel=" + entrySensorChannel +
                   ",entrySensorInverted=" + entrySensorInverted +
                   ",entrySensorThreshold=" + entrySensorThreshold +
                   ",entryTriggerCallback=" + (entryTriggerCallback != null) +
                   ",exitSensorType=" + exitSensorType +
                   ",exitSensorChannel=" + exitSensorChannel +
                   ",exitSensorInverted=" + exitSensorInverted +
                   ",exitSensorThreshold=" + exitSensorThreshold +
                   ",exitTriggerCallback=" + (exitTriggerCallback != null);
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final TrcIntake intake;
    private FrcDigitalInput digitalSensor;
    private FrcAnalogInput analogSensor;

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
    public FrcIntake(
        String instanceName, MotorType motorType, int motorId, boolean brushless, boolean absEnc, Params params)
    {
        this.instanceName = instanceName;

        TrcMotor intakeMotor = FrcMotor.createMotor(motorType, brushless, false, instanceName + ".motor", motorId);
        intakeMotor.resetFactoryDefault();
        intakeMotor.setMotorInverted(params.motorInverted);
        intakeMotor.setBrakeModeEnabled(true);
        if (params.followerMotor != null)
        {
            params.followerMotor.follow(intakeMotor, params.motorInverted != params.followerMotorInverted);
        }
        if (params.voltageCompensationEnabled)
        {
            intakeMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        }

        TrcIntake.Trigger entryTrigger = null, exitTrigger = null;
        if (params.entrySensorType != null)
        {
            entryTrigger = createTrigger(
                instanceName + ".entry", params.entrySensorType, params.entrySensorChannel,
                params.entrySensorInverted, params.entrySensorThreshold, params.entryTriggerCallback);
        }
 
        if (params.exitSensorType != null)
        {
            exitTrigger = createTrigger(
                instanceName + ".exit", params.exitSensorType, params.exitSensorChannel,
                params.exitSensorInverted, params.exitSensorThreshold, params.exitTriggerCallback);
        }

        intake = new TrcIntake(instanceName, intakeMotor, entryTrigger, exitTrigger);
    }   //FrcIntake

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
     * This method returns the intake object.
     *
     * @return intake object.
     */
    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    /**
     * This method creates an TrcIntake Trigger.
     *
     * @param instanceName specifies the instance name of the Intake.
     * @param sensorType specifies whether the sensor is an analog sensor or a digital sensor.
     * @param channel specifies the channel number the sensor is connected to (analog or digital input channel).
     * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
     * @param threshold specifies the analog sensor threshold value, ignored if sensor is digital.
     * @param triggerCallback specifies the trigger event callback, null if not provided.
     * @return the created TrcIntake Trigger.
     */
    private TrcIntake.Trigger createTrigger(
        String instanceName, SensorType sensorType, int channel, boolean inverted, double threshold,
        TrcEvent.Callback triggerCallback)
    {
        TrcIntake.Trigger trigger;

        if (sensorType == SensorType.DigitalSensor)
        {
            analogSensor = null;
            digitalSensor = new FrcDigitalInput(instanceName, channel);
            digitalSensor.setInverted(inverted);
            TrcTrigger digitalTrigger = new TrcTriggerDigitalInput(instanceName, digitalSensor);
            trigger = new TrcIntake.Trigger(digitalTrigger, triggerCallback, null, null);
        }
        else
        {
            digitalSensor = null;
            analogSensor = new FrcAnalogInput(instanceName, channel);
            analogSensor.setEnabled(inverted);
            TrcTrigger analogTrigger = new TrcTriggerThresholdZones(
                instanceName, this::getAnalogInput, new double[] {threshold}, false);
            trigger = new TrcIntake.Trigger(analogTrigger, triggerCallback, threshold, inverted);
        }

        return trigger;
    }   //createTrigger

    /**
     * This method returns the analog sensor value.
     *
     * @return analog sensor value.
     */
    private double getAnalogInput()
    {
        return analogSensor != null? analogSensor.getData(0).value: 0.0;
    }   //getAnalogInput

}   //class FrcIntake
