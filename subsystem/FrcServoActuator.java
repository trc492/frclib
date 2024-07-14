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

import frclib.motor.FrcServo;

public class FrcServoActuator
{
    /**
     * This class contains all the parameters related to the actuator servo.
     */
    public static class Params
    {
        public boolean servoInverted = false;
        public int followerServoChannel = -1;
        public boolean followerServoInverted = false;
        public double logicalPosMin = 0.0;
        public double logicalPosMax = 1.0;
        public double physicalPosMin = 0.0;
        public double physicalPosMax = 1.0;
        public Double maxStepRate = null;
        public double presetTolerance = 0.0;
        public double[] positionPresets = null;

        /**
         * This methods sets the servo direction.
         *
         * @param inverted specifies true to invert servo direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setServoInverted(boolean inverted)
        {
            servoInverted = inverted;
            return this;
        }   //setServoInverted

        /**
         * This methods sets if the actuator has a follower servo and if the follower servo is inverted.
         *
         * @param followerServoChannel specifies the PWM channel for the follower servo, -1 if no follower.
         * @param followerServoInverted specifies true if the follower servo is inverted, false otherwise. Only
         *        applicable if there is a follower.
         * @return this object for chaining.
         */
        public Params setFollowerServo(int followerServoChannel, boolean followerServoInverted)
        {
            this.followerServoChannel = followerServoChannel;
            this.followerServoInverted = followerServoInverted;
            return this;
        }   //setFollowerServo

        /**
         * This method sets the logical position range of the servo in the range of 0.0 to 1.0.
         *
         * @param minPos specifies the min logical position.
         * @param maxPos specifies the max logical position.
         * @return this object for chaining.
         */
        public Params setLogicalPosRange(double minPos, double maxPos)
        {
            logicalPosMin = minPos;
            logicalPosMax = maxPos;
            return this;
        }   //setLogicalPosRange

        /**
         * This method sets the physical position range of the servo in real world physical unit.
         *
         * @param minPos specifies the min physical position.
         * @param maxPos specifies the max physical position.
         * @return this object for chaining.
         */
        public Params setPhysicalPosRange(double minPos, double maxPos)
        {
            physicalPosMin = minPos;
            physicalPosMax = maxPos;
            return this;
        }   //setPhysicalPosRange

        /**
         * This method sets the maximum stepping rate of the servo. This enables setPower to speed control the servo.
         *
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this parameter object.
         */
        public Params setMaxStepRate(double maxStepRate)
        {
            this.maxStepRate = maxStepRate;
            return this;
        }   //setMaxStepRate

        /**
         * This method sets an array of preset positions for the servo actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public Params setPositionPresets(double tolerance, double... posPresets)
        {
            presetTolerance = tolerance;
            positionPresets = posPresets;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the servoParams info.
         *
         * @return string format of the servo param info.
         */
        @Override
        public String toString()
        {
            return "servoInverted=" + servoInverted +
                   ",followerServo=" + followerServoChannel +
                   ",followerInverted=" + followerServoInverted +
                   ",logicalMin=" + logicalPosMin +
                   ",logicalMax=" + logicalPosMax +
                   ",physicalMin=" + physicalPosMin +
                   ",physicalMax=" + physicalPosMax +
                   ",maxStepRate=" + maxStepRate +
                   ",posPresets=" + Arrays.toString(positionPresets);
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final FrcServo actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmChannel specifies the PWM channel the servo is connected to.
     * @param params specifies the parameters to set up the actuator servo.
     */
    public FrcServoActuator(String instanceName, int pwmChannel, Params params)
    {
        this.instanceName = instanceName;
        actuator = new FrcServo(instanceName + ".servo", pwmChannel);
        actuator.setInverted(params.servoInverted);
        actuator.setLogicalPosRange(params.logicalPosMin, params.logicalPosMax);
        actuator.setPhysicalPosRange(params.physicalPosMin, params.physicalPosMax);
        actuator.setPosPresets(params.presetTolerance, params.positionPresets);

        if (params.maxStepRate != null)
        {
            actuator.setMaxStepRate(params.maxStepRate);
        }

        if (params.followerServoChannel != -1)
        {
            FrcServo follower = new FrcServo(instanceName + ".followerServo", params.followerServoChannel);
            follower.setInverted(params.followerServoInverted);
            follower.follow(actuator);
        }
    }   //FrcServoActuator

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
    public FrcServo getActuator()
    {
        return actuator;
    }   //getActuator

}   //class FrcServoActuator
