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

package frclib.drivebase;

import trclib.drivebase.TrcMecanumDriveBase;

/**
 * This class creates the FrcMecanum drive base subsystem that consists of wheel motors and related objects for
 * driving a mecanum robot.
 */
public class FrcMecanumDrive extends FrcRobotDrive
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotInfo specifies the Mecanum Robot Info.
     * @param useExternalOdometry specifies true to use Odometry wheels, false to use drive motor odometry.
     */
    public FrcMecanumDrive(RobotInfo robotInfo, boolean useExternalOdometry)
    {
        super(robotInfo);
        TrcMecanumDriveBase driveBase = new TrcMecanumDriveBase(
            driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
            driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK], gyro);
        configDriveBase(driveBase, useExternalOdometry);
    }   //FrcMecanumDrive

}   //class FrcMecanumDrive
