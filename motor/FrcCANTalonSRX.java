/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import trclib.motor.TrcMotor;

/**
 * This class implements a CanTalonSrx motor controller. It extends the TrcMotor class and
 * implements the abstract methods required by TrcMotor to be compatible with the TRC library.
 */
public class FrcCANTalonSRX extends FrcCANPhoenix5Controller<TalonSRX>
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param motorParams specifies the motor params, can be null if not provided.
     */
    public FrcCANTalonSRX(String instanceName, int canId, TrcMotor.Params motorParams)
    {
        super(instanceName, new TalonSRX(canId), motorParams);
    }   //FrcCANTalonSRX

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     */
    public FrcCANTalonSRX(String instanceName, int canId)
    {
        this(instanceName, canId, null);
    }   //FrcCANTalonSRX

    /**
     * This method synchronizes the internal encoder to the absolute zero position.
     *
     * @param rangeLow specifies the low range of the absolute encoder reading.
     * @param rangeHigh specifies the high range of the absolute encoder reading.
     * @param crossZeroOnInterval specifies true if the absoulte encoder will cross the zero point.
     * @param zeroOffset specifies the encoder offset of the absolute zero position.
     */
    public void setAbsoluteZeroOffset(int rangeLow, int rangeHigh, boolean crossZeroOnInterval, int zeroOffset)
    {
        ErrorCode error;
        SensorCollection sensorCollection = motor.getSensorCollection();

        error = sensorCollection.syncQuadratureWithPulseWidth(
            rangeLow, rangeHigh, crossZeroOnInterval, -zeroOffset, 10);
        if (error != ErrorCode.OK)
        {
            tracer.traceErr(instanceName, "syncQuadratureWithPulseWidth failed (error=" + error.name() + ").");
        }

        error = motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        if (error != ErrorCode.OK)
        {
            tracer.traceErr(instanceName, "configSelectedFeedbackSensor failed (error=" + error.name() + ").");
        }

        tracer.traceDebug(
            instanceName,
            "zeroOffset=" + zeroOffset +
            ", pwmPos=" + sensorCollection.getPulseWidthPosition() +
            ", quadPos=" + sensorCollection.getQuadraturePosition() +
            ", selectedPos=" + motor.getSelectedSensorPosition());
    }   //setAbsoluteZeroOffset

}   //class FrcCANTalonSRX
