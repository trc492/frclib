/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import trclib.archive.TrcPixyCam1;
import trclib.archive.TrcSerialBusDevice;

/**
 * This class implements a platform dependent pixy camera 1 that is either connected to an I2C bus, SPI or a
 * Serial Port. It provides access to the last detected objects reported by the pixy camera asynchronously.
 */
public class FrcPixyCam1 extends TrcPixyCam1
{
    public static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;
    public static final int DEF_I2C_ADDRESS = 0x54;

    public static final SerialPort.Port DEF_SERIAL_PORT = SerialPort.Port.kOnboard;
    public static final int DEF_BAUD_RATE = 19200;
    public static final int DEF_DATA_BITS = 8;
    public static final SerialPort.Parity DEF_PARITY = SerialPort.Parity.kNone;
    public static final SerialPort.StopBits DEF_STOP_BITS = SerialPort.StopBits.kOne;

    private final TrcSerialBusDevice pixyCam;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the SPI port on the RoboRIO.
     */
    public FrcPixyCam1(String instanceName, SPI.Port port)
    {
        super(instanceName, true);
        SPI spi = new SPI(port);
        spi.setChipSelectActiveLow();

        pixyCam = new FrcSpiDevice(instanceName, spi, true);
    }   //FrcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcPixyCam1(String instanceName, I2C.Port port, int devAddress)
    {
        super(instanceName, false);
        pixyCam = new FrcI2cDevice(instanceName, port, devAddress, true);
    }   //FrcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     */
    public FrcPixyCam1(String instanceName, I2C.Port port)
    {
        this(instanceName, port, DEF_I2C_ADDRESS);
    }   //FrcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     */
    public FrcPixyCam1(
        String instanceName, SerialPort.Port port, int baudRate, int dataBits, SerialPort.Parity parity,
        SerialPort.StopBits stopBits)
    {
        super(instanceName, false);
        pixyCam = new FrcSerialPortDevice(instanceName, port, baudRate, dataBits, parity, stopBits, true);
    }   //FrcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     */
    public FrcPixyCam1(String instanceName, SerialPort.Port port, int baudRate)
    {
        this(instanceName, port, baudRate, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     */
    public FrcPixyCam1(String instanceName, SerialPort.Port port)
    {
        this(instanceName, port, DEF_BAUD_RATE, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcPixyCam1

    /**
     * This method checks if the pixy camera is enabled.
     *
     * @return true if pixy camera is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        boolean enabled = pixyCam.isEnabled();

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the pixy camera.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        pixyCam.setEnabled(enabled);
        if (enabled)
        {
            start();
        }
        else
        {
            end();
        }
    }   //setEnabled

    //
    // Implements TrcPixyCam1 abstract methods.
    //

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     */
    @Override
    public void asyncReadData(RequestId requestId, int length)
    {
        pixyCam.asyncRead(requestId, -1, length, false, this::requestHandler, null);
    }   //asyncReadData

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     * @param data specifies the data buffer.
     */
    @Override
    public void asyncWriteData(RequestId requestId, byte[] data)
    {
        pixyCam.asyncWrite(requestId, data, data.length, null);
    }   //asyncWriteData

}   //class FrcPixyCam1
