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
import frclib.sensor.FrcSensorTrigger;
import trclib.motor.TrcServo;
import trclib.sensor.TrcAnalogSource;
import trclib.subsystem.TrcServoClaw;

/**
 * This class implements a platform dependent Servo Claw Subsystem. A Servo Claw consists of one or two servos.
 * Optionally, it may have a sensor to detect the game element entering the proximity of the claw to activate the
 * grabbing action.
 */
public class FrcServoClaw
{
    /**
     * This class contains all the parameters of the Servo Claw.
     */
    public static class Params
    {
        private FrcServoActuator.Params servoParams = null;
        private double openPos = 1.0;
        private double openTime = 0.5;
        private double closePos = 0.0;
        private double closeTime = 0.5;
        private FrcSensorTrigger sensorTrigger = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "servoParams=(" + servoParams +
                   "),openPos=" + openPos +
                   ",openTime=" + openTime +
                   ",closePos=" + closePos +
                   ",closeTime=" + closeTime +
                   ",sensorTrigger=" + sensorTrigger;
        }   //toString

        /**
         * This methods sets the parameters of the primary servo.
         *
         * @param servoName specifies the name of the servo.
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryServo(String servoName, int channel, boolean inverted)
        {
            this.servoParams = new FrcServoActuator.Params()
                .setPrimaryServo(servoName, channel, inverted);
            return this;
        }   //setPrimaryServo

        /**
         * This methods sets the parameter of the follower servo if there is one.
         *
         * @param servoName specifies the name of the servo.
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerServo(String servoName, int channel, boolean inverted)
        {
            if (servoParams == null)
            {
                throw new IllegalStateException("Must set the primary servo parameters first.");
            }

            servoParams.setFollowerServo(servoName, channel, inverted);
            return this;
        }   //setFollowerServo

        /**
         * This method sets the open/close parameters of the servo claw.
         *
         * @param openPos specifies the open position in physical unit.
         * @param openTime specifies the time in seconds required to open from fully close position.
         * @param closePos specifies the close position in physical unit.
         * @param closeTime specifies the time in seconds required to close from fully open position.
         * @return this parameter object.
         */
        public Params setOpenCloseParams(double openPos, double openTime, double closePos, double closeTime)
        {
            this.openPos = openPos;
            this.openTime = openTime;
            this.closePos = closePos;
            this.closeTime = closeTime;
            return this;
        }   //setOpenCloseParams

        /**
         * This method creates the digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the digital input channel the sensor is connected to.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setDigitalInputTrigger(String sensorName, int sensorChannel, boolean sensorInverted)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FrcSensorTrigger().setDigitalInputTrigger(sensorName, sensorChannel, sensorInverted);
            return this;
        }   //setDigitalInputTrigger

        /**
         * This method creates the analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the analog input channel the sensor is connected to.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @return this object for chaining.
         */
        public Params setAnalogInputTrigger(
            String sensorName, int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold,
            double triggerSettlingPeriod)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FrcSensorTrigger()
                .setAnalogInputTrigger(
                    sensorName, sensorChannel, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod);
            return this;
        }   //setAnalogInputTrigger

        /**
         * This method creates the analog source trigger.
         *
         * @param sourceName specifies the name of the data source.
         * @param dataSource specifies the method to call to get the data source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @return this object for chaining.
         */
        public Params setAnalogSourceTrigger(
            String sourceName, TrcAnalogSource.AnalogDataSource dataSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FrcSensorTrigger()
                .setAnalogSourceTrigger(
                    sourceName, dataSource, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod);
            return this;
        }   //setAnalogSourceTrigger

    }   //class Params

    private final TrcServoClaw claw;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the servo claw parameters.
     */
    public FrcServoClaw(String instanceName, Params params)
    {
        TrcServo servo = new FrcServoActuator(params.servoParams).getServo();
        TrcServoClaw.Params clawParams = new TrcServoClaw.Params()
            .setServo(servo)
            .setOpenCloseParams(params.openPos, params.openTime, params.closePos, params.closeTime);

        if (params.sensorTrigger != null)
        {
            clawParams.setSensorTrigger(params.sensorTrigger.getTrigger());
        }

        claw = new TrcServoClaw(instanceName, clawParams);
    }   //FrcServoClaw

    /**
     * This method returns the created servo claw.
     *
     * @return servo claw.
     */
    public TrcServoClaw getClaw()
    {
        return claw;
    }   //getClaw

}   //class FrcServoClaw
