/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib.driverio;

import edu.wpi.first.wpilibj.PWM;
import trclib.output.TrcRevBlinkin;

/**
 * This class implements a platform dependent REV Blinkin device. It provides a platform dependent method that
 * sets the color pattern value to the device.
 */
public class FrcRevBlinkin extends TrcRevBlinkin
{
    private static final Pattern offPattern = new Pattern("Off", RevLedPattern.SolidBlack);

    private PWM device;
    private Pattern currPattern = offPattern;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel      specifies the PWM channel the device is on.
     */
    public FrcRevBlinkin(String instanceName, int channel)
    {
        super(instanceName);
        device = new PWM(channel);
        device.setBoundsMicroseconds(2003, 1550, 1500, 1460, 999);
        device.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        device.setSpeed(0.0);
        device.setZeroLatch();
        setPattern(currPattern);
    }   //FrcRevBlinkin

    //
    // Implements TrcRevBlinkin abstract methods.
    //

    /**
     * This method gets the current set LED pattern.
     *
     * @return currently set LED pattern.
     */
    @Override
    public Pattern getPattern()
    {
        return currPattern;
    }   //getPattern

    /**
     * This method sets the LED pattern to the physical REV Blinkin device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        pattern = pattern == null ? offPattern : pattern;

        currPattern = pattern;
        device.setPosition(pattern.ledPattern.value);
    }   //setPattern

}   //class FrcRevBlinkin
