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

import trclib.motor.TrcServo;

/**
 * This class creates an FRC platform specific servo with the specified parameters.
 */
public class FrcServoActuator
{
    /**
     * This class contains all the parameters for creating the servo.
     */
    public static class Params
    {
        private String primaryServoName = null;
        private int primaryServoChannel = -1;
        private boolean primaryServoInverted = false;

        private String followerServoName = null;
        private int followerServoChannel = -1;
        private boolean followerServoInverted = false;

        private final TrcServo.Params servoParams = new TrcServo.Params();

        /**
         * This method returns the string format of the servoParams info.
         *
         * @return string format of the servo param info.
         */
        @Override
        public String toString()
        {
            return "primaryServoName=" + primaryServoName +
                   ",primaryServoChannel=" + primaryServoChannel +
                   ",primaryServoInverted=" + primaryServoInverted +
                   "\nfollowerServoName=" + followerServoName +
                   ",followerServoChannel=" + followerServoChannel +
                   ",followerServoInverted=" + followerServoInverted +
                   "\nservoParams=(" + servoParams + ")";
        }   //toString

        /**
         * This methods sets the parameters of the primary servo.
         *
         * @param name specifies the name of the servo.
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryServo(String name, int channel, boolean inverted)
        {
            if (channel == -1)
            {
                throw new IllegalArgumentException("Must provide a valid primary servo PWM channel.");
            }

            this.primaryServoName = name;
            this.primaryServoChannel = channel;
            this.primaryServoInverted = inverted;
            return this;
        }   //setPrimaryServo

        /**
         * This methods sets the parameter of the follower servo.
         *
         * @param name specifies the name of the servo.
         * @param channel specifies the PWM channel for the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerServo(String name, int channel, boolean inverted)
        {
            this.followerServoName = name;
            this.followerServoChannel = channel;
            this.followerServoInverted = inverted;
            return this;
        }   //setFollowerServo

        /**
         * This method sets the physical position range of the servo in real world physical unit.
         *
         * @param minPos specifies the min physical position.
         * @param maxPos specifies the max physical position.
         * @return this object for chaining.
         */
        public Params setPhysicalPosRange(double minPos, double maxPos)
        {
            this.servoParams.setPhysicalPosRange(minPos, maxPos);
            return this;
        }   //setPhysicalPosRange

        /**
         * This method sets the logical position range of the servo in the range of 0.0 to 1.0.
         *
         * @param minPos specifies the min logical position.
         * @param maxPos specifies the max logical position.
         * @return this object for chaining.
         */
        public Params setLogicalPosRange(double minPos, double maxPos)
        {
            this.servoParams.setLogicalPosRange(minPos,maxPos);
            return this;
        }   //setLogicalPosRange

        /**
         * This method sets the maximum stepping rate of the servo. This enables setPower to speed control the servo.
         *
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this parameter object.
         */
        public Params setMaxStepRate(double maxStepRate)
        {
            this.servoParams.setMaxStepRate(maxStepRate);
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
            this.servoParams.setPosPresets(tolerance, posPresets);
            return this;
        }   //setPositionPresets

    }   //class Params

    private final TrcServo primaryServo;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the parameters to set up the actuator servo.
     */
    public FrcServoActuator(Params params)
    {
        primaryServo = new FrcServo(params.primaryServoName, params.primaryServoChannel, params.servoParams);
        primaryServo.setInverted(params.primaryServoInverted);

        if (params.followerServoChannel != -1)
        {
            FrcServo followerServo = new FrcServo(params.followerServoName, params.followerServoChannel);
            followerServo.setInverted(params.followerServoInverted);
            followerServo.follow(primaryServo);
        }
    }   //FrcServoActuator

    /**
     * This method returns the created primary servo.
     *
     * @return primary servo.
     */
    public TrcServo getServo()
    {
        return primaryServo;
    }   //getServo

}   //class FrcServoActuator
