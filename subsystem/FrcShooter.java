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
import trclib.motor.TrcMotor;
import trclib.subsystem.TrcShooter;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor or a continuous
 * rotation servo, optionally a lower limit switch, an upper limit switch and an encoder. It creates all the necessary
 * components for a PID controlled actuator which could include a software PID controller.
 */
public class FrcShooter
{
    /**
     * This class contains all the parameters related to the shooter.
     */
    public static class Params
    {
        private int shooterMotor1Id = -1;
        private MotorType shooterMotor1Type = null;
        private boolean shooterMotor1Brushless = false;
        private boolean shooterMotor1AbsEnc = false;
        private boolean shooterMotor1Inverted = false;

        private int shooterMotor2Id = -1;
        private MotorType shooterMotor2Type = null;
        private boolean shooterMotor2Brushless = false;
        private boolean shooterMotor2AbsEnc = false;
        private boolean shooterMotor2Inverted = false;
        private boolean shooterMotor2IsFollower = false;

        private int tiltMotorId = -1;
        private MotorType tiltMotorType = null;
        private boolean tiltMotorBrushless = false;
        private boolean tiltMotorAbsEnc = false;
        private FrcMotorActuator.Params tiltMotorParams = null;
        private TrcShooter.PanTiltParams tiltParams = null;
        
        private int panMotorId = -1;
        private MotorType panMotorType = null;
        private boolean panMotorBrushless = false;
        private boolean panMotorAbsEnc = false;
        private FrcMotorActuator.Params panMotorParams = null;
        private TrcShooter.PanTiltParams panParams = null;

        /**
         * This method sets the parameters of the shooter motor 1.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor1(
            int motorId, MotorType motorType, boolean motorBrushless, boolean motorAbsEnc, boolean motorInverted)
        {
            this.shooterMotor1Id = motorId;
            this.shooterMotor1Type = motorType;
            this.shooterMotor1Brushless = motorBrushless;
            this.shooterMotor1AbsEnc = motorAbsEnc;
            this.shooterMotor1Inverted = motorInverted;
            return this;
        }   //setShooterMotor1

        /**
         * This method sets the parameters of the shooter motor 2.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param isFollower specifies true if motor2 is a follower of motor1, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor2(
            int motorId, MotorType motorType, boolean motorBrushless, boolean motorAbsEnc, boolean motorInverted,
            boolean isFollower)
        {
            this.shooterMotor2Id = motorId;
            this.shooterMotor2Type = motorType;
            this.shooterMotor2Brushless = motorBrushless;
            this.shooterMotor2AbsEnc = motorAbsEnc;
            this.shooterMotor2Inverted = motorInverted;
            this.shooterMotor2IsFollower = isFollower;
            return this;
        }   //setShooterMotor2

        /**
         * This method sets the parameters of tilt if there is one.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorParams specifies motor parameters.
         * @param tiltParams specifies tilt parameters.
         * @return this object for chaining.
         */
        public Params setTiltMotor(
            int motorId, MotorType motorType, boolean motorBrushless, boolean motorAbsEnc,
            FrcMotorActuator.Params motorParams, TrcShooter.PanTiltParams tiltParams)
        {
            this.tiltMotorId = motorId;
            this.tiltMotorType = motorType;
            this.tiltMotorBrushless = motorBrushless;
            this.tiltMotorAbsEnc = motorAbsEnc;
            this.tiltMotorParams = motorParams;
            this.tiltParams = tiltParams;
            return this;
        }   //setTiltMotor

        /**
         * This method sets the parameters of pan if there is one.
         *
         * @param motorId specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
         * @param motorType specifies the motor type.
         * @param motorBrushless specifies true if motor is brushless, false if brushed (only applicable for SparkMax).
         * @param motorAbsEnc specifies true if uses DutyCycle absolute encoder, false to use relative encoder (only
         *        applicable for SparkMax).
         * @param motorParams specifies motor parameters.
         * @param panParams specifies pan parameters.
         * @return this object for chaining.
         */
        public Params setPanMotor(
            int motorId, MotorType motorType, boolean motorBrushless, boolean motorAbsEnc,
            FrcMotorActuator.Params motorParams, TrcShooter.PanTiltParams panParams)
        {
            this.panMotorId = motorId;
            this.panMotorType = motorType;
            this.panMotorBrushless = motorBrushless;
            this.panMotorAbsEnc = motorAbsEnc;
            this.panMotorParams = motorParams;
            this.panParams = panParams;
            return this;
        }   //setPanMotor

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return "shooterMotor1Id=" + shooterMotor1Id +
                   ",shooterMotor1Type=" + shooterMotor1Type +
                   ",shooterMotor1Brushless=" + shooterMotor1Brushless +
                   ",shooterMotor1AbsEnc=" + shooterMotor1AbsEnc +
                   ",shooterMotor1Inverted=" + shooterMotor1Inverted +
                   "\nshooterMotor2Id=" + shooterMotor2Id +
                   ",shooterMotor2Type=" + shooterMotor2Type +
                   ",shooterMotor2Brushless=" + shooterMotor2Brushless +
                   ",shooterMotor2AbsEnc=" + shooterMotor2AbsEnc +
                   ",shooterMotor2Inverted=" + shooterMotor2Inverted +
                   ",shooterMotorIsFollower=" + shooterMotor2IsFollower +
                   "\ntiltMotorId=" + tiltMotorId +
                   ",tiltMotorType=" + tiltMotorType +
                   ",tiltMotorBrushless=" + tiltMotorBrushless +
                   ",tiltMotorAbsEnc=" + tiltMotorAbsEnc +
                   ",tiltMotorParams=" + tiltMotorParams +
                   ",tiltParams=" + tiltParams +
                   "\npanMotorId=" + panMotorId +
                   ",panMotorType=" + panMotorType +
                   ",panMotorBrushless=" + panMotorBrushless +
                   ",panMotorAbsEnc=" + panMotorAbsEnc +
                   ",panMotorParams=" + panMotorParams +
                   ",panParams=" + panParams;
        }   //toString

    }   //class Params

    private final TrcShooter shooter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param Params params specifies the shooter parameters.
     */
    public FrcShooter(String instanceName, Params params)
    {
        if (params.shooterMotor1Id == -1)
        {
            throw new IllegalArgumentException("Shooter must have motor 1.");
        }

        TrcMotor shooterMotor1 = FrcMotor.createMotor(
            instanceName + "shootMotor1", params.shooterMotor1Id, params.shooterMotor1Type,
            params.shooterMotor1Brushless, params.shooterMotor1AbsEnc);
        // Use Coast Mode for shooter motor.
        shooterMotor1.setBrakeModeEnabled(false);
        shooterMotor1.setMotorInverted(params.shooterMotor1Inverted);

        TrcMotor shooterMotor2 = null;
        if (params.shooterMotor2Id != -1)
        {
            shooterMotor2 = FrcMotor.createMotor(
                instanceName + "shootMotor2", params.shooterMotor2Id, params.shooterMotor2Type,
                params.shooterMotor2Brushless, params.shooterMotor2AbsEnc);
            // Use Coast Mode for shooter motor.
            shooterMotor2.setBrakeModeEnabled(false);
            if (params.shooterMotor2IsFollower)
            {
                // Motor 2 is a follower, don't need to include it when creating shooter.
                shooterMotor2.follow(shooterMotor1, params.shooterMotor2Inverted);
                shooterMotor2 = null;
            }
            else
            {
                shooterMotor2.setMotorInverted(params.shooterMotor2Inverted);
            }
        }

        TrcMotor tiltMotor = null;
        if (params.tiltMotorId != -1)
        {
            tiltMotor = new FrcMotorActuator(
                instanceName + ".tilt", params.tiltMotorId, params.tiltMotorType, params.tiltMotorBrushless,
                params.tiltMotorAbsEnc, params.tiltMotorParams).getActuator();
        }

        TrcMotor panMotor = null;
        if (params.panMotorId != -1)
        {
            panMotor = new FrcMotorActuator(
                instanceName + ".pan", params.panMotorId, params.panMotorType, params.panMotorBrushless,
                params.panMotorAbsEnc, params.panMotorParams).getActuator();
        }

        shooter = new TrcShooter(
            instanceName, shooterMotor1, shooterMotor2, tiltMotor, params.tiltParams, panMotor, params.panParams);
    }   //FrcShooter

    /**
     * This method returns the shooter object.
     *
     * @return shooter object.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

}   //class FrcShooter
