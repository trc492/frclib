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

package frclib.archive;

import edu.wpi.first.wpilibj.DigitalInput;
import trclib.archive.TrcDigitalInput;

/**
 * This class implements a platform dependent digital input sensor extending TrcDigitalInput. It provides
 * implementation of the abstract methods in TrcDigitalInput.
 */
public class FrcDigitalInput extends TrcDigitalInput
{
    private DigitalInput digitalInput;
    private boolean state = false;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the digital I/O channel.
     */
    public FrcDigitalInput(String instanceName, int channel)
    {
        super(instanceName);
        digitalInput = new DigitalInput(channel);
    }   //FrcDigitalInput

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the digital input sensor.
     *
     * @return true if the digital input sensor is active, false otherwise.
     */
    @Override
    public boolean getInputState()
    {
        if (getInputElapsedTimer != null) getInputElapsedTimer.recordStartTime();
        state = digitalInput.get();
        if (getInputElapsedTimer != null) getInputElapsedTimer.recordEndTime();

        return state;
    }   //getInputState

}   //class FrcDigitalInput
