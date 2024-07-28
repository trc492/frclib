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

import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcDigitalInput;
import trclib.sensor.TrcEncoder;

public class FrcMotor
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
     * This method create an array of motors and configure them (can be drive motor or steer motor for Swerve Drive).
     *
     * @param name specifies the instance name of the motor.
     * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
     * @param motorType specifies the motor type.
     * @param brushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
     * @param absEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
     *        applicable for SparkMax).
     * @param lowerLimitSwitch specifies an external lower limit switch overriding the motor controller one, can be
     *        null if there is none.
     * @param upperLimitSwitch specifies an external upper limit switch overriding the motor controller one, can be
     *        null if there is none.
     * @param encoder specifies an external encoder overriding the motor controller one, can be null if there is none.
     * @return created motor.
     */
    public static TrcMotor createMotor(
        String name, int motorId, MotorType motorType, boolean brushless, boolean absEnc,
        TrcDigitalInput lowerLimitSw, TrcDigitalInput upperLimitSw, TrcEncoder encoder)
    {
        TrcMotor motor;

        switch (motorType)
        {
            case CanTalonFx:
                motor = new FrcCANTalonFX(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;
            
            case CanTalonSrx:
                motor = new FrcCANTalonSRX(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;

            case CanSparkMax:
                motor = new FrcCANSparkMax(name, motorId, brushless, absEnc, lowerLimitSw, upperLimitSw, encoder);
                break;

            case PwmTalonFx:
                motor = new FrcPWMTalonFX(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;

            case PwmTalonSrx:
                motor = new FrcPWMTalonSRX(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;

            case PwmSparkMax:
                motor = new FrcPWMSparkMax(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;

            case PwmVictorSpx:
                motor = new FrcPWMVictorSPX(name, motorId, lowerLimitSw, upperLimitSw, encoder);
                break;

            case CRServo:
                motor = new FrcCRServo(name, motorId, lowerLimitSw, upperLimitSw, encoder);
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

    /**
     * This method create an array of motors and configure them (can be drive motor or steer motor for Swerve Drive).
     *
     * @param name specifies the instance name of the motor.
     * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
     * @param motorType specifies the motor type.
     * @param brushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
     * @param absEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
     *        applicable for SparkMax).
     * @return created motor.
     */
    public static TrcMotor createMotor(
        String name, int motorId, MotorType motorType, boolean brushless, boolean absEnc)
    {
        return createMotor(name, motorId, motorType, brushless, absEnc, null, null, null);
    }   //createMotor

}   //class FrcMotor
