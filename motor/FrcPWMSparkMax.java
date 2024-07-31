/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import trclib.motor.TrcMotor;

public class FrcPWMSparkMax extends FrcPWMMotorController<PWMSparkMax>
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmChannel specifies the PWM channel number of the motor.
     * @param sensors specifies external sensors, can be null if none.
     */
    public FrcPWMSparkMax(String instanceName, int pwmChannel, TrcMotor.ExternalSensors sensors)
    {
        super(instanceName, new PWMSparkMax(pwmChannel), sensors);
    }   //FrcPWMSparkMax

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmChannel specifies the PWM channel number of the motor.
     */
    public FrcPWMSparkMax(
        String instanceName, int pwmChannel)
    {
        this(instanceName, pwmChannel, null);
    }   //FrcPWMSparkMax

}   //class FrcPWMSparkMax
