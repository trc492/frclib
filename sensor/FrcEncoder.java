/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib.sensor;

import edu.wpi.first.wpilibj.Encoder;
import trclib.sensor.TrcEncoder;

/**
 * This interface specifies a common implementation of a generic encoder with which makes different tpes of encoders
 * compatible with each other.
 */
public class FrcEncoder extends Encoder implements TrcEncoder
{
    public enum EncoderType
    {
        CANCoder,
        Canandmag,
        AnalogEncoder
    }   //enum EncoderType

    private boolean inverted = false;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;

    public FrcEncoder(int channelA, int channelB, EncodingType encodingType)
    {
        super(channelA, channelB, false, encodingType);
    }   //FrcEncoder

    /**
     * This method creates an encoder with the specified parameters and initializes it.
     *
     * @param encoderName specifies the instance name of the encoder.
     * @param encoderType specifies the encoder type.
     * @param inverted specifies true to invert the direction of the encoder, false otherwise.
     * @param encoderId specifies the ID for the encoder (CAN ID for CAN encoder, analog channel for analog encoder).
     * @param canBusName specifies the can bus name.
     * @return created encoder.
     */
    public static TrcEncoder createEncoder(
        String encoderName, EncoderType encoderType, boolean inverted, int encoderId, String canBusName)
    {
        TrcEncoder encoder = null;

        switch (encoderType)
        {
            case CANCoder:
                FrcCANCoder canCoder = new FrcCANCoder(encoderName, encoderId, canBusName);
                canCoder.resetFactoryDefault();
                canCoder.setInverted(inverted);
                canCoder.setAbsoluteRange(true);
                // CANCoder is already normalized to the range of 0 to 1.0 for a revolution
                // (revolution per count).
                canCoder.setScaleAndOffset(1.0, 0.0, 0.0);
                encoder = canCoder;
                break;

            case Canandmag:
                FrcCanandmag canandmag = new FrcCanandmag(encoderName, encoderId, canBusName);
                canandmag.resetFactoryDefaults(false);
                canandmag.setInverted(inverted);
                // Canandmag is already normalized to the range of 0 to 1.0 for a revolution (revolution per count).
                canandmag.setScaleAndOffset(1.0, 0.0, 0.0);
                encoder = canandmag;
                break;

            case AnalogEncoder:
                encoder = new FrcAnalogEncoder(encoderName, encoderId).getAbsoluteEncoder();
                encoder.setInverted(inverted);
                // Analog Encoder is already normalized to the range of 0 to 1.0 for a revolution
                // (revolution per count).
                encoder.setScaleAndOffset(1.0, 0.0, 0.0);
                break;

            default:
                throw new UnsupportedOperationException("Encoder type " + encoderType + " is not supported.");
        }

        return encoder;
    }   //createEncoder

    //
    // Implements TrcEncoder interface.
    //

    /**
     * This method reads the absolute position of the encoder.
     *
     * @return absolute position of the encoder.
     */
    @Override
    public double getRawPosition()
    {
        return super.get();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        return (super.get() - zeroOffset) * scale + offset;
    }   //getScaledPosition

    /**
     * This method reads the raw encoder velocity in encoder units per second.
     *
     * @return raw encoder velocity in encoder units per second.
     */
    @Override
    public double getRawVelocity()
    {
        return super.getRate();
    }   //getRawVelocity

    /**
     * This method returns the encoder velocity adjusted by scale.
     *
     * @return encoder velocity adjusted by scale.
     */
    @Override
    public double getScaledVelocity()
    {
        return super.getRate() * scale;
    }   //getScaledVelocity

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        this.inverted = inverted;
        super.setReverseDirection(inverted);
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return inverted;
    }   //isInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset, double zeroOffset)
    {
        this.scale = scale;
        this.offset = offset;
        this.zeroOffset = zeroOffset;
    }   //setScaleAndOffset

}   //interface FrcEncoder
