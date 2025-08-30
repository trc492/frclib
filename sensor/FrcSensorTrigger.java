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

package frclib.sensor;

import trclib.motor.TrcMotor;
import trclib.sensor.TrcAnalogSensor;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerDigitalInput;
import trclib.sensor.TrcTriggerThresholdRange;

/**
 * This class creates an FRC platform specific Sensor Trigger with the specified parameters.
 */
public class FrcSensorTrigger
{
    private final String instanceName;
    private FrcAnalogInput analogInput = null;
    private TrcAnalogSensor analogSensor = null;
    private TrcMotor motor = null;
    private TrcTrigger trigger = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcSensorTrigger(String instanceName)
    {
        this.instanceName = instanceName;
    }   //FrcSensorTrigger

    /**
     * This method creates a digital input trigger.
     *
     * @param sensorChannel specifies the digital input channel the sensor is connected to.
     * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
     * @return this object for chaining.
     */
    public FrcSensorTrigger setDigitalInputTrigger(int sensorChannel, boolean sensorInverted)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        FrcDigitalInput digitalInput = new FrcDigitalInput(instanceName + ".digitalInput", sensorChannel);
        digitalInput.setInverted(sensorInverted);
        trigger = new TrcTriggerDigitalInput(instanceName + ".trigger", digitalInput);
        return this;
    }   //setDigitalInputTrigger

    /**
     * This method creates an analog input trigger.
     *
     * @param sensorChannel specifies the analog input channel the sensor is connected to.
     * @param lowerTriggerThreshold specifies the lower trigger threshold value.
     * @param upperTriggerThreshold specifies the upper trigger threshold value.
     * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
     *        trigger range to be triggered.
     * @return this object for chaining.
     */
    public FrcSensorTrigger setAnalogInputTrigger(
        int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold, double triggerSettlingPeriod)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        analogInput = new FrcAnalogInput(instanceName + ".analogInput", sensorChannel);
        trigger = new TrcTriggerThresholdRange(instanceName + ".trigger", this::getAnalogValue);
        ((TrcTriggerThresholdRange) trigger).setTrigger(
            lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod);
        return this;
    }   //setAnalogInputTrigger

    /**
     * This method creates an analog sensor trigger.
     *
     * @param analogSource specifies the method to call to get the analog sensor value.
     * @param lowerTriggerThreshold specifies the lower trigger threshold value.
     * @param upperTriggerThreshold specifies the upper trigger threshold value.
     * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
     *        trigger range to be triggered.
     * @return this object for chaining.
     */
    public FrcSensorTrigger setAnalogSensorTrigger(
        TrcAnalogSensor.AnalogDataSource analogSource, double lowerTriggerThreshold, double upperTriggerThreshold,
        double triggerSettlingPeriod)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        analogSensor = new TrcAnalogSensor(instanceName + ".analogSensor", analogSource);
        trigger = new TrcTriggerThresholdRange(instanceName + ".trigger", this::getAnalogValue);
        ((TrcTriggerThresholdRange) trigger).setTrigger(
            lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod);
        return this;
    }   //setAnalogSensorTrigger

    /**
     * This method creates a motor current trigger.
     *
     * @param motor specifies the motor to get the current value from.
     * @param lowerTriggerThreshold specifies the lower trigger threshold value.
     * @param upperTriggerThreshold specifies the upper trigger threshold value.
     * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
     *        trigger range to be triggered.
     * @return this object for chaining.
     */
    public FrcSensorTrigger setMotorCurrentTrigger(
        TrcMotor motor, double lowerTriggerThreshold, double upperTriggerThreshold, double triggerSettlingPeriod)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        this.motor = motor;
        trigger = new TrcTriggerThresholdRange(instanceName + ".trigger", this::getAnalogValue);
        ((TrcTriggerThresholdRange) trigger).setTrigger(
            lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod);
        return this;
    }   //setMotorCurrentTrigger

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
     * This method returns the created sensor trigger.
     *
     * @return sensor trigger.
     */
    public TrcTrigger getTrigger()
    {
        return trigger;
    }   //getTrigger

    /**
     * This method returns the analog sensor value.
     *
     * @return analog sensor value.
     */
    private double getAnalogValue()
    {
        double data = 0.0;

        if (analogInput != null)
        {
            data = analogInput.getData(0).value;
        }
        else if (analogSensor != null)
        {
            data = analogSensor.getData(0).value;
        }
        else if (motor != null)
        {
            data = motor.getCurrent();
        }

        return data;
    }   //getAnalogValue

}   //class FrcSensorTrigger
