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
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcRollerIntake.TriggerAction;

/**
 * This class implements a platform dependent Roller Intake Subsystem. A Roller Intake consists of a DC motor or a
 * continuous rotation servo. Optionally, it may have front and back sensors to detect the game element entering
 * or exiting the Intake and allows callback actions such as stopping the Intake motor.
 */
public class FrcRollerIntake
{
    /**
     * This class contains all the parameters of the Roller Intake.
     */
    public static class Params
    {
        private FrcMotorActuator.Params motorParams = null;
        private final TrcRollerIntake.IntakeParams  intakeParams = new TrcRollerIntake.IntakeParams();
        private TrcRollerIntake.TriggerParams frontTriggerParams = null;
        private TrcRollerIntake.TriggerParams backTriggerParams = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "motorParams=" + motorParams +
                   ",intakeParams=" + intakeParams +
                   ",frontTriggerParams=" + frontTriggerParams +
                   ",backTriggerParams=" + backTriggerParams;
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

            this.motorParams = new FrcMotorActuator.Params().setPrimaryMotor(
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

            this.motorParams.addFollowerMotor(motorName, motorType, inverted, motorId, canBusName, sparkMaxParams);
            return this;
        }   //setFollowerMotor

        /**
         * This method sets various power levels of the Roller Intake.
         *
         * @param intakePower specifies the intake power.
         * @param ejectPower specifies the eject power.
         * @param retainPower specifies the retain power.
         * @return this parameter object.
         */
        public Params setPowerLevels(double intakePower, double ejectPower, double retainPower)
        {
            intakeParams.setPowerLevels(intakePower, ejectPower, retainPower);
            return this;
        }   //setPowerLevels

        /**
         * This method sets various power levels of the Roller Intake.
         *
         * @param intakeFinishDelay specifies the intake finish delay in seconds.
         * @param ejectFinishDelay specifies the eject finish delay in seconds.
         * @return this parameter object.
         */
        public Params setFinishDelays(double intakeFinishDelay, double ejectFinishDelay)
        {
            intakeParams.setFinishDelays(intakeFinishDelay, ejectFinishDelay);
            return this;
        }   //setFinishDelays

        /**
         * This method creates the front digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setFrontDigitalInputTrigger(
            String sensorName, int sensorChannel, boolean sensorInverted, TriggerAction triggerAction,
            TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (frontTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            frontTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setDigitalInputTrigger(sensorName, sensorChannel, sensorInverted).getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setFrontDigitalInputTrigger

        /**
         * This method creates the front digital source trigger.
         *
         * @param sourceName specifies the name of the digital source.
         * @param digitalSource specifies the method to call to get the digital state value.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setFrontDigitalSourceTrigger(
            String sourceName, BooleanSupplier digitalSource, TriggerAction triggerAction, TriggerMode triggerMode,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (frontTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            frontTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), triggerAction,
                triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setFrontDigitalSourceTrigger

        /**
         * This method creates the front analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setFrontAnalogInputTrigger(
            String sensorName, int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold,
            double triggerSettlingPeriod, TriggerAction triggerAction, TriggerMode triggerMode,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (frontTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            frontTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setAnalogInputTrigger(
                    sensorName, sensorChannel, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod)
                    .getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setFrontAnalogInputTrigger

        /**
         * This method creates the front analog source trigger.
         *
         * @param sourceName specifies the name of the analog source.
         * @param analogSource specifies the method to call to get the analog source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setFrontAnalogSourceTrigger(
            String sourceName, DoubleSupplier analogSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod, TriggerAction triggerAction,
            TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (frontTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            frontTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setAnalogSourceTrigger(
                    sourceName, analogSource, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod)
                    .getTrigger(),
                    triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setFrontAnalogSourceTrigger

        /**
         * This method creates the front motor current trigger.
         *
         * @param motor specifies the intake motor to get current value from.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setFrontMotorCurrentTrigger(
            TrcMotor motor, double lowerTriggerThreshold, double upperTriggerThreshold, double triggerSettlingPeriod,
            TriggerAction triggerAction, TriggerMode triggerMode, TrcEvent.Callback triggerCallback,
            Object triggerCallbackContext)
        {
            if (frontTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            frontTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setMotorCurrentTrigger(
                    motor, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod).getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setFrontMotorCurrentTrigger

        /**
         * This method creates the back digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setBackDigitalInputTrigger(
            String sensorName, int sensorChannel, boolean sensorInverted, TriggerAction triggerAction,
            TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (backTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            backTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setDigitalInputTrigger(sensorName, sensorChannel, sensorInverted).getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setBackDigitalInputTrigger

        /**
         * This method creates the back digital source trigger.
         *
         * @param sourceName specifies the name of the digital source.
         * @param digitalSource specifies the method to call to get the digital state value.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setBackDigitalSourceTrigger(
            String sourceName, BooleanSupplier digitalSource, TriggerAction triggerAction, TriggerMode triggerMode,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (backTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            backTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), triggerAction,
                triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setBackDigitalSourceTrigger

        /**
         * This method creates the back analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorChannel specifies the channel the sensor is connected to.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setBackAnalogInputTrigger(
            String sensorName, int sensorChannel, double lowerTriggerThreshold, double upperTriggerThreshold,
            double triggerSettlingPeriod, TriggerAction triggerAction, TriggerMode triggerMode,
            TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (backTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            backTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setAnalogInputTrigger(
                    sensorName, sensorChannel, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod)
                    .getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setBackAnalogInputTrigger

        /**
         * This method creates the back analog source trigger.
         *
         * @param sourceName specifies the name of the analog source.
         * @param analogSource specifies the method to call to get the analog source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setBackAnalogSourceTrigger(
            String sourceName, DoubleSupplier analogSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod, TriggerAction triggerAction,
            TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
        {
            if (backTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            backTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setAnalogSourceTrigger(
                    sourceName, analogSource, lowerTriggerThreshold, upperTriggerThreshold,
                    triggerSettlingPeriod).getTrigger(), triggerAction, triggerMode, triggerCallback,
                    triggerCallbackContext);
            return this;
        }   //setBackAnalogSourceTrigger

        /**
         * This method creates the back motor current trigger.
         *
         * @param motor specifies the intake motor to get current value from.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @param triggerAction specifies the action when the trigger occurs.
         * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
         * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
         * @param triggerCallbackContext specifies the callback context object.
         * @return this object for chaining.
         */
        public Params setBackMotorCurrentTrigger(
            TrcMotor motor, double lowerTriggerThreshold, double upperTriggerThreshold, double triggerSettlingPeriod,
            TriggerAction triggerAction, TriggerMode triggerMode, TrcEvent.Callback triggerCallback,
            Object triggerCallbackContext)
        {
            if (backTriggerParams != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            backTriggerParams = new TrcRollerIntake.TriggerParams(
                new FrcSensorTrigger().setMotorCurrentTrigger(
                    motor, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod).getTrigger(),
                triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
            return this;
        }   //setBackMotorCurrentTrigger

   }   //class Params

    private final TrcRollerIntake intake;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FrcRollerIntake(String instanceName, Params params)
    {
        intake = new TrcRollerIntake(
            instanceName,
            new FrcMotorActuator(params.motorParams).getMotor(),
            params.intakeParams,
            params.frontTriggerParams,
            params.backTriggerParams);
    }   //FrcRollerIntake

    /**
     * This method returns the intake object.
     *
     * @return intake object.
     */
    public TrcRollerIntake getIntake()
    {
        return intake;
    }   //getIntake

}   //class FrcRollerIntake
