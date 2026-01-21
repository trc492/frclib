/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.motor.FrcMotorActuator.SparkMaxMotorParams;
import frclib.sensor.FrcSensorTrigger;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcPidStorage;

/**
 * This class implements a platform dependent PID Controlled Storage Subsystem. It consists of a motor or a
 * continuous rotation servo that transports the game elements in the storage from the entrance to the exit.
 * Optionally, it may have entry and exit sensors to detect the game element entering or exiting the Storage and
 * allows callback actions such as advancing the Storage to the next position.
 */
public class FrcPidStorage
{
    /**
     * This class contains all the parameters of the Storage subsystem.
     */
    public static class Params
    {
        private FrcMotorActuator.Params motorParams = null;
        private final TrcPidStorage.StorageParams storageParams = new TrcPidStorage.StorageParams();
        private TrcPidStorage.TriggerParams entryTriggerParams = null;
        private TrcPidStorage.TriggerParams exitTriggerParams = null;

       /**
        * This method returns the string format of the Params info.
        *
        * @return string format of the params info.
        */
        @Override
        public String toString()
        {
            return "motorParams=" + motorParams +
                   ",storageParams=" + storageParams +
                   ",entryTriggerParams=" + entryTriggerParams +
                   ",exitTriggerParams=" + exitTriggerParams;
        }   //toString

        /**
         * This method sets the parameters of the primary motor.
         *
         * @param motorType specifies the motor type.
         * @param sparkMaxParams specifies extra parameters for SparkMax motor, null if motor type is not SparkMax.
         * @param motorName specifies the name of the motor.
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(
            MotorType motorType, SparkMaxMotorParams sparkMaxParams, String motorName, int motorId, String canBusName,
            boolean inverted)
        {
            if (motorId == -1)
            {
                throw new IllegalArgumentException("Must provide a valid primary motor ID.");
            }

            motorParams = new FrcMotorActuator.Params().setPrimaryMotor(
                motorName, motorType, inverted, motorId, canBusName, sparkMaxParams);
            return this;
        }   //setPrimaryMotor

        /**
         * This method sets the parameters of the follower motor.
         *
         * @param motorType specifies the motor type.
         * @param sparkMaxParams specifies extra parameters for SparkMax motor, null if motor type is not SparkMax.
         * @param motorName specifies the name of the motor.
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(
            MotorType motorType, SparkMaxMotorParams sparkMaxParams, String motorName, int motorId, String canBusName,
            boolean inverted)
        {
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            motorParams.addFollowerMotor(motorName, motorType, inverted, motorId, canBusName, sparkMaxParams);
            return this;
        }   //setFollowerMotor

        /**
         * This method sets the lower limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param channel specifies the digital input channel the limit switch is connected to.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(String name, int channel, boolean inverted)
        {
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            motorParams.setLowerLimitSwitch(name, channel, inverted);
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param channel specifies the digital input channel the limit switch is connected to.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(String name, int channel, boolean inverted)
        {
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            motorParams.setUpperLimitSwitch(name, channel, inverted);
            return this;
        }   //setUpperLimitSwitch

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
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }
 
            motorParams.setPositionScaleAndOffset(scale, offset, zeroOffset);
            return this;
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
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            motorParams.setPositionPresets(tolerance, posPresets);
            return this;
        }   //setPositionPresets

        /**
         * This method sets the distance between objects inside the storage.
         *
         * @param distance specifies the distance between objects in the storage.
         * @return this parameter object.
         */
        public Params setObjectDistance(double distance)
        {
            storageParams.setObjectDistance(distance);
            return this;
        }   //setObjectDistance

        /**
         * This method sets the storage move power.
         *
         * @param power specifies the motor power to move the object in the storage.
         * @return this parameter object.
         */
        public Params setMovePower(double power)
        {
            storageParams.setMovePower(power);
            return this;
        }   //setMovePower

        /**
         * This method sets the maximum number of objects the storage can hold.
         *
         * @param capacity specifies maximum number of objects the storage can hold.
         * @return this parameter object.
         */
        public Params setMaxCapacity(int capacity)
        {
            storageParams.setMaxCapacity(capacity);
            return this;
        }   //setMaxCapacity

        /**
         * This method creates the entry digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setEntryDigitalInputTrigger(
            String sensorName, int sensorChannel, boolean sensorInverted, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (entryTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            entryTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setDigitalInputTrigger(sensorName, sensorChannel, sensorInverted).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setEntryDigitalInputTrigger

        /**
         * This method creates the entry digital source trigger.
         *
         * @param sourceName specifies the name of the digital source.
         * @param digitalSource specifies the method to call to get the digital state value.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setEntryDigitalSourceTrigger(
            String sourceName, BooleanSupplier digitalSource, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (entryTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            entryTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setEntryDigitalSourceTrigger

        /**
         * This method creates the entry analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setEntryAnalogInputTrigger(
            String sensorName, int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold,
            double triggerSettlingPeriod, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
            Object triggerCallbackContext)
        {
            if (entryTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            entryTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setAnalogInputTrigger(
                    sensorName, sensorChannel, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod)
                    .getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setEntryAnalogInputTrigger

        /**
         * This method creates the entry analog source trigger.
         *
         * @param sourceName specifies the name of the analog source.
         * @param analogSource specifies the method to call to get the analog source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setEntryAnalogSourceTrigger(
            String sourceName, DoubleSupplier analogSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (entryTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            entryTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setAnalogSourceTrigger(
                    sourceName, analogSource, lowerTriggerThreshold, upperTriggerThreshold,
                    triggerSettlingPeriod).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
       }   //setEntryAnalogSourceTrigger

        /**
         * This method creates the exit digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setExitDigitalInputTrigger(
            String sensorName, int sensorChannel, boolean sensorInverted, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (exitTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            exitTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setDigitalInputTrigger(sensorName, sensorChannel, sensorInverted).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setExitDigitalInputTrigger

        /**
         * This method creates the exit digital source trigger.
         *
         * @param sourceName specifies the name of the digital source.
         * @param digitalSource specifies the method to call to get the digital state value.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setExitDigitalSourceTrigger(
            String sourceName, BooleanSupplier digitalSource, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (exitTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            exitTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setExitDigitalSourceTrigger

        /**
         * This method creates the exit analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setExitAnalogInputTrigger(
            String sensorName, int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold,
            double triggerSettlingPeriod, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
            Object triggerCallbackContext)
        {
            if (exitTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            exitTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setAnalogInputTrigger(
                    sensorName, sensorChannel, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod)
                    .getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setExitAnalogInputTrigger

        /**
         * This method creates the exit analog source trigger.
         *
         * @param sourceName specifies the name of the analog source.
         * @param analogSource specifies the method to call to get the analog source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setExitAnalogSourceTrigger(
            String sourceName, DoubleSupplier analogSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod, boolean advanceOnTrigger,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (exitTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            exitTriggerParams = new TrcPidStorage.TriggerParams(
                new FrcSensorTrigger().setAnalogSourceTrigger(
                    sourceName, analogSource, lowerTriggerThreshold, upperTriggerThreshold,
                    triggerSettlingPeriod).getTrigger(),
                advanceOnTrigger, triggerCallback, triggerCallbackContext);
            return this;
        }   //setExitAnalogSourceTrigger

    }   //class Params

    private final TrcPidStorage pidStorage;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FrcPidStorage(String instanceName, Params params)
    {
        TrcMotor motor = new FrcMotorActuator(params.motorParams).getMotor();
        motor.setBrakeModeEnabled(true);
        motor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        pidStorage = new TrcPidStorage(
            instanceName,
            motor,
            params.storageParams,
            params.entryTriggerParams,
            params.exitTriggerParams);
    }   //FrcRollerIntake

    /**
     * This method returns the PID Storage object.
     *
     * @return pidStorage object.
     */
    public TrcPidStorage getPidStorage()
    {
        return pidStorage;
    }   //getPidStorage

}   //class FrcPidStorage
