/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import trclib.sensor.TrcGyro;
import trclib.timer.TrcTimer;

public class FrcPigeon2 extends TrcGyro
{
    private class GyroInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            builder.setSmartDashboardType("Pigeon2");
            builder.addDoubleProperty("Value", () -> getZHeading().value, null);
        }   //initSendable

    }   //class GyroInfo

    public Pigeon2 pigeon;
    private double xSign = 1.0;
    private double ySign = 1.0;
    private double zSign = 1.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the Pigeon2.
     * @param canBus specifies the CAN Bus the Pigeon2 is connected to.
     */
    public FrcPigeon2(String instanceName, int canId, CANBus canBus)
    {
        super(instanceName, 3, GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS, null);
        this.pigeon = new Pigeon2(canId, canBus);
    }   //FrcPigeon2

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the Pigeon2.
     * @param canBusName specifies the CAN Bus name the Pigeon2 is connected to.
     */
    public FrcPigeon2(String instanceName, int canId, String canBusName)
    {
        this(instanceName, canId, new CANBus(canBusName));
    }   //FrcPigeon2

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the Pigeon2.
     */
    public FrcPigeon2(String instanceName, int canId)
    {
        this(instanceName, canId, new CANBus("rio"));
    }   //FrcPigeon2

    /**
     * This method creates a GyroInfo object and returns it.
     *
     * @return created GyroInfo object.
     */
    public Sendable getGyroSendable()
    {
        GyroInfo gyroInfo = new GyroInfo();
        SendableRegistry.setName(gyroInfo, toString());
        return gyroInfo;
    }   //getGyroSendable

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    public SensorData<Double> getRawXData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = pigeon.getAngularVelocityYWorld().getValueAsDouble();
        }
        else if (dataType == DataType.HEADING)
        {
            value = pigeon.getPitch().getValueAsDouble();
        }
        SensorData<Double> data = new SensorData<>(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    public SensorData<Double> getRawYData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            // Pigeon's coordinate system is NWU, so its X axis is our Y axis.
            value = pigeon.getAngularVelocityXWorld().getValueAsDouble();
        }
        else if (dataType == DataType.HEADING)
        {
            value = pigeon.getRoll().getValueAsDouble();
        }
        SensorData<Double> data = new SensorData<>(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    public SensorData<Double> getRawZData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        }
        else if (dataType == DataType.HEADING)
        {
            value = -pigeon.getYaw().getValueAsDouble();
        }
        SensorData<Double> data = new SensorData<>(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawZData

    /**
     * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        xSign = inverted? -1.0: 1.0;
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        ySign = inverted? -1.0: 1.0;
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        zSign = inverted? -1.0: 1.0;
    }   //setZInverted

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData<Double> getXRotationRate()
    {
        return new SensorData<>(
            TrcTimer.getCurrentTime(), xSign*pigeon.getAngularVelocityYDevice().getValueAsDouble());
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData<Double> getYRotationRate()
    {
        return new SensorData<>(
            TrcTimer.getCurrentTime(), ySign*pigeon.getAngularVelocityXDevice().getValueAsDouble());
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData<Double> getZRotationRate()
    {
        return new SensorData<>(
            TrcTimer.getCurrentTime(), zSign*pigeon.getAngularVelocityZDevice().getValueAsDouble());
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData<Double> getXHeading()
    {
        return new SensorData<>(TrcTimer.getCurrentTime(), xSign*pigeon.getPitch().getValueAsDouble());
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData<Double> getYHeading()
    {
        return new SensorData<>(TrcTimer.getCurrentTime(), ySign*pigeon.getRoll().getValueAsDouble());
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData<Double> getZHeading()
    {
        return new SensorData<>(TrcTimer.getCurrentTime(), zSign*-pigeon.getYaw().getValueAsDouble());
    }   //getZHeading

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis integrator.");
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis integrator.");
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        pigeon.reset();;
    }   //resetZIntegrator

}   //class FrcPigeon2
