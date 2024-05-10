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

import java.util.Arrays;

import edu.wpi.first.wpilibj.SPI;
import trclib.archive.TrcSerialBusDevice;

/**
 * This class implements a platform dependent SPI device. It extends TrcSerialBusDevice to provide asynchronous
 * request queue support and creates the SPI device using WPILib.
 */
public class FrcSpiDevice extends TrcSerialBusDevice
{
    private SPI spi;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param spi specifies the SPI port the device is connected to.
     * @param useRequestQueue specifies true to use a request queue, false otherwise.
     */
    public FrcSpiDevice(String instanceName, SPI spi, boolean useRequestQueue)
    {
        super(instanceName, useRequestQueue);
        this.spi = spi;
    }   //FrcSpiDevice

    //
    // Implements TrcSerialBusDevice abstract methods.
    //

    /**
     * This method is called to read data from the device with the specified length.
     *
     * @param address specifies the data address if any (not applicable for SPI).
     * @param length specifies the number of bytes to read.
     * @return a byte array containing the data read.
     */
    @Override
    public byte[] readData(int address, int length)
    {
        byte[] buffer = new byte[length];

        spi.read(true, buffer, length);
        tracer.traceDebug(instanceName, "addr=" + address + ",len=" + length + ",data=" + Arrays.toString(buffer));

        return buffer;
    }   //readData

    /**
     * This method is called to write data to the device with the specified data buffer and length.
     *
     * @param address specifies the data address if any (not applicable for SPI).
     * @param buffer specifies the buffer containing the data to be written to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    @Override
    public int writeData(int address, byte[] buffer, int length)
    {
        tracer.traceDebug(instanceName, "addr=" + address + ",data=" + Arrays.toString(buffer) + ",len=" + length);
        spi.write(buffer, length);

        return length;
    }   //writeData

}   //class FrcSpiDevice
