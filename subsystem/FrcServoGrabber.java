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

import frclib.motor.FrcServoActuator;
import frclib.sensor.FrcAnalogInput;
import frclib.sensor.FrcDigitalInput;
import frclib.sensor.FrcSensorTrigger.SensorType;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerDigitalInput;
import trclib.sensor.TrcTriggerThresholdZones;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a platform dependent Smart Intake Subsystem. An Intake consists of a DC motor or a continuous
 * rotation servo. Optionally, it may have entry and exit sensors to detect the game element entering or exiting the
 * Intake and allows callback actions such as stopping the Intake motor.
 */
public class FrcServoGrabber
{
    /**
     * This class contains all the parameters of the Servo Grabber.
     */
    public static class Params
    {
        private FrcServoActuator.Params servoParams = null;

        private int sensorChannel = -1;
        private SensorType sensorType = null;
        private boolean sensorInverted = false;
        private double triggerThreshold = 0.0;
        private double hasObjectThreshold = 0.0;
        private TrcEvent.Callback triggerCallback = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "servoParams=" + servoParams +
                   ",sensorChannel=" + sensorChannel +
                   ",sensorType=" + sensorType +
                   ",sensorInverted=" + sensorInverted +
                   ",triggerThreshold=" + triggerThreshold +
                   ",hasObjectThreshold=" + hasObjectThreshold +
                   ",triggerCallback=" + (triggerCallback != null);
        }   //toString

        /**
         * This methods sets the parameters of the primary servo.
         *
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryServo(int channel, boolean inverted)
        {
            this.servoParams = new FrcServoActuator.Params().setPrimaryServo(channel, inverted);
            return this;
        }   //setPrimaryServo

        /**
         * This methods sets the parameter of the follower servo if there is one.
         *
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerServo(int channel, boolean inverted)
        {
            if (servoParams == null)
            {
                throw new IllegalStateException("Must set the primary servo parameters first.");
            }

            servoParams.setFollowerServo(channel, inverted);
            return this;
        }   //setFollowerServo

        /**
         * This method specifies the entry sensor type and parameters if there is one.
         *
         * @param channel specifies the channel number the sensor is plugged into (analog or digital input).
         * @param sensorType specifies the sensor type, null if there is no sensor.
         * @param inverted specifies true if the sensor polarity is inverted.
         * @param triggerThreshold specifies the trigger threshold value if it is an analog sensor, ignored if sensor
         *        is digital.
         * @param hasObjectThreshold specifies the threshold value to detect object possession if it is an analog
         *        sensor, ignored if sensor is digital.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setSensorTrigger(
            int channel, SensorType sensorType, boolean inverted, double triggerThreshold, double hasObjectThreshold,
            TrcEvent.Callback triggerCallback)
        {
            this.sensorChannel = channel;
            this.sensorType = sensorType;
            this.sensorInverted = inverted;
            this.triggerThreshold = triggerThreshold;
            this.hasObjectThreshold = hasObjectThreshold;
            this.triggerCallback = triggerCallback;
            return this;
        }   //setSensorTrigger

    }   //class Params

    private final TrcServoGrabber grabber;
    private FrcDigitalInput digitalSensor;
    private FrcAnalogInput analogSensor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param servoParams specifies the servo actuator params.
     * @param sensorTrigger specifies the sensor trigger object, can be null if none.
     * @param triggerCallback specifies the callback handler when the sensor is triggered, null if no sensorTrigger.
     */
    public FrcServoGrabber(String instanceName, Params params)
    {
        TrcServo servo = new FrcServoActuator(instanceName + ".servo", params.servoParams).getServo();
        TrcTrigger sensorTrigger = createTrigger(
            instanceName + ".trigger", params.sensorChannel, params.sensorType, params.sensorInverted,
            params.triggerThreshold);
        TrcServoGrabber.Params grabberParams = new TrcServoGrabber.Params().setServo(servo);
        if (sensorTrigger != null)
        {
            grabberParams.setSensorTrigger(
                sensorTrigger, params.sensorInverted, params.triggerThreshold, params.hasObjectThreshold,
                params.triggerCallback);
        }

        grabber = new TrcServoGrabber(instanceName, grabberParams);
    }   //FrcServoGrabber

    /**
     * This method creates a Sensor Trigger.
     *
     * @param instanceName specifies the instance name of the Trigger.
     * @param sensorChannel specifies the channel number the sensor is connected to (analog or digital input channel).
     * @param sensorType specifies whether the sensor is an analog sensor or a digital sensor.
     * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
     * @param threshold specifies the analog sensor threshold value, ignored if sensor is digital.
     * @return the created Trigger.
     */
    private TrcTrigger createTrigger(
        String instanceName, int sensorChannel, SensorType sensorType, boolean inverted, double threshold)
    {
        TrcTrigger trigger = null;

        if (sensorChannel != -1)
        {
            if (sensorType == SensorType.DigitalSensor)
            {
                analogSensor = null;
                digitalSensor = new FrcDigitalInput(instanceName, sensorChannel);
                digitalSensor.setInverted(inverted);
                trigger = new TrcTriggerDigitalInput(instanceName, digitalSensor);
            }
            else
            {
                digitalSensor = null;
                analogSensor = new FrcAnalogInput(instanceName, sensorChannel);
                analogSensor.setEnabled(inverted);
                trigger = new TrcTriggerThresholdZones(
                    instanceName, this::getAnalogInput, new double[] {threshold}, false);
            }
        }

        return trigger;
    }   //createTrigger

    /**
     * This method returns the created servo grabber.
     *
     * @return servo grabber.
     */
    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

    /**
     * This method returns the analog sensor value.
     *
     * @return analog sensor value.
     */
    private double getAnalogInput()
    {
        return analogSensor != null? analogSensor.getData(0).value: 0.0;
    }   //getAnalogInput

}   //class FrcServoGrabber
