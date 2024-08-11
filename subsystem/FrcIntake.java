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

import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcSensorTrigger;
import frclib.sensor.FrcSensorTrigger.SensorType;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcIntake;

/**
 * This class implements a platform dependent Smart Intake Subsystem. An Intake consists of a DC motor or a continuous
 * rotation servo. Optionally, it may have entry and exit sensors to detect the game element entering or exiting the
 * Intake and allows callback actions such as stopping the Intake motor.
 */
public class FrcIntake
{
    /**
     * This class contains all the parameters of the Intake.
     */
    public static class Params
    {
        private FrcMotorActuator.Params motorParams = null;

        private int entrySensorChannel = -1;
        private SensorType entrySensorType = null;
        private boolean entrySensorInverted = false;
        private double entrySensorThreshold = 0.0;
        private TrcEvent.Callback entryTriggerCallback = null;

        private int exitSensorChannel = -1;
        private SensorType exitSensorType = null;
        private boolean exitSensorInverted = false;
        private double exitSensorThreshold = 0.0;
        private TrcEvent.Callback exitTriggerCallback = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "motorParams=" + motorParams +
                   "\nentrySensorChannel=" + entrySensorChannel +
                   ",entrySensorType=" + entrySensorType +
                   ",entrySensorInverted=" + entrySensorInverted +
                   ",entrySensorThreshold=" + entrySensorThreshold +
                   ",entryTriggerCallback=" + (entryTriggerCallback != null) +
                   "\nexitSensorChannel=" + exitSensorChannel +
                   ",exitSensorType=" + exitSensorType +
                   ",exitSensorInverted=" + exitSensorInverted +
                   ",exitSensorThreshold=" + exitSensorThreshold +
                   ",exitTriggerCallback=" + (exitTriggerCallback != null);
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

            this.motorParams = new FrcMotorActuator.Params().setPrimaryMotor(motorId, motorType, brushless, absEnc, inverted);
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
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            this.motorParams.setFollowerMotor(motorId, motorType, brushless, absEnc, inverted);
            return this;
        }   //setFollowerMotor

        /**
         * This method specifies the entry sensor type and parameters if there is one.
         *
         * @param channel specifies the channel number the sensor is plugged into (analog or digital input).
         * @param sensorType specifies the sensor type, null if there is no sensor.
         * @param inverted specifies true if the sensor polarity is inverted.
         * @param threshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
         *        digital.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setEntrySensor(
            int channel, SensorType sensorType, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.entrySensorChannel = channel;
            this.entrySensorType = sensorType;
            this.entrySensorInverted = inverted;
            this.entrySensorThreshold = threshold;
            this.entryTriggerCallback = triggerCallback;
            return this;
        }   //setEntrySensor

        /**
         * This method specifies the exit sensor type and parameters if there is one.
         *
         * @param channel specifies the channel number the sensor is plugged into (analog or digital input).
         * @param sensorType specifies the sensor type, null if there is no sensor.
         * @param inverted specifies true if the sensor polarity is inverted.
         * @param threshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
         *        digital.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setExitSensor(
            int channel, SensorType sensorType, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.entrySensorChannel = channel;
            this.exitSensorType = sensorType;
            this.exitSensorInverted = inverted;
            this.exitSensorThreshold = threshold;
            this.exitTriggerCallback = triggerCallback;
            return this;
        }   //setExitSensor

    }   //class Params

    private final TrcIntake intake;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FrcIntake(String instanceName, Params params)
    {
        TrcMotor primaryMotor = new FrcMotorActuator(instanceName, params.motorParams).getMotor();
        TrcIntake.TriggerParams entryTrigger = createTriggerParams(
            instanceName + ".entry", params.entrySensorChannel, params.entrySensorType, params.entrySensorInverted,
            params.entrySensorThreshold, params.entryTriggerCallback);
        TrcIntake.TriggerParams exitTrigger = createTriggerParams(
            instanceName + ".exit", params.exitSensorChannel, params.exitSensorType, params.exitSensorInverted,
            params.exitSensorThreshold, params.exitTriggerCallback);

        intake = new TrcIntake(instanceName, primaryMotor, entryTrigger, exitTrigger);
    }   //FrcIntake

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
     * This method creates the trigger parameters for the Intake sensor.
     *
     * @param instanceName specifies instance name of the trigger.
     * @param sensorChannel specifies the channel number the sensor is plugged into (analog or digital input).
     * @param sensorType specifies the sensor type.
     * @param sensorInverted specifies true if the sensor polarity is inverted.
     * @param sensorThreshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
     *        digital.
     * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
     * @return created trigger parameters, null if there is no sensor.
     */
    private TrcIntake.TriggerParams createTriggerParams(
        String instanceName, int sensorChannel, SensorType sensorType, boolean sensorInverted, double sensorThreshold,
        TrcEvent.Callback triggerCallback)
    {
        TrcIntake.TriggerParams triggerParams = null;

        if (sensorChannel != -1)
        {
            triggerParams = new TrcIntake.TriggerParams(
                new FrcSensorTrigger(
                    instanceName, sensorChannel, sensorType, sensorInverted, sensorThreshold).getTrigger(),
                triggerCallback);
        }

        return triggerParams;
    }   //createTriggerParams

}   //class FrcIntake
