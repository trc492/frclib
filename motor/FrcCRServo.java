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

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;

/**
 * This class implements an FRC Continuous Rotation Servo extending TrcMotor. It provides implementation of the
 * TrcMotorController interface in TrcMotor. It supports limit switches and an external position sensor (e.g. encoder).
 * When this class is constructed with limit switches, all motor movements will respect them and will not move the
 * motor into the direction where the limit switch is activated.
 */
public class FrcCRServo extends TrcMotor
{
    private final Servo servo;
    private double sign = 1.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name of the motor.
     * @param pwmChannel specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
     * @param sensors specifies external sensors, can be null if none.
     */
    public FrcCRServo(String instanceName, int pwmChannel, TrcMotor.ExternalSensors sensors)
    {
        super(instanceName, sensors);
        servo = new Servo(pwmChannel);
    }   //FrcCRServo

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name of the motor.
     * @param pwmChannel specifies the ID for the motor (CAN ID for CAN motor, PWM channel for PWM motor).
     */
    public FrcCRServo(String instanceName, int pwmChannel)
    {
        this(instanceName, pwmChannel, null);
    }   //FrcCRServo

    //
    // Implements TrcMotorController interface.
    //

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        throw new UnsupportedOperationException("CRServo does not support resetFactoryDefault.");
    }   //resetFactoryDefault

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return RobotController.getBatteryVoltage();
    }   //getBusVoltage

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent specifies threshold current in amperes to be exceeded before limiting occurs.
     *        If this value is less than currentLimit, then currentLimit is used as the threshold.
     * @param triggerThresholdTime specifies how long current must exceed threshold (seconds) before limiting occurs.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        throw new UnsupportedOperationException("CRServo does not support setCurrentLimit.");
    }   //setCurrentLimit

    /**
     * This method sets the stator current limit of the motor.
     *
     * @param currentLimit specifies the stator current limit in amperes.
     */
    @Override
    public void setStatorCurrentLimit(double currentLimit)
    {
        throw new UnsupportedOperationException("CRServo does not support setStatorCurrentLimit.");
    }   //setStatorCurrentLimit

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        throw new UnsupportedOperationException("CRServo does not support setCloseLoopRampRate.");
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        throw new UnsupportedOperationException("CRServo does not support setOpenLoopRampRate.");
    }   //setOpenLoopRampRate

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        throw new UnsupportedOperationException("CRServo does not support setBrakeModeEnabled.");
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //disableMotorRevLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //isMotorFwdLimitSwitchEnabled

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorRevLimitSwitchInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //setMotorRevLimitSwitchInverted

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorFwdLimitSwitchInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //isMotorRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        throw new UnsupportedOperationException("CRServo does not support limit switches.");
    }   //isMotorFwdLimitSwitchActive

    /**
     * This method sets the soft position limit for the reverse direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorRevSoftPositionLimit(Double limit)
    {
        throw new UnsupportedOperationException("CRServo does not support soft limits.");
    }   //setMotorRevSoftPositionLimit

    /**
     * This method sets the soft position limit for the forward direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorFwdSoftPositionLimit(Double limit)
    {
        throw new UnsupportedOperationException("CRServo does not support soft limits.");
    }   //setMotorFwdSoftPositionLimit

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setMotorPositionSensorInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("CRServo does not support position sensor.");
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        throw new UnsupportedOperationException("CRServo does not support position sensor.");
    }   //isMotorPositionSensorInverted

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        throw new UnsupportedOperationException("CRServo does not support position sensor.");
    }   //resetMotorPosition

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        sign = inverted? -1.0: 1.0;
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return sign == -1.0;
    }   //isMotorInverted

    /**
     * This method sets the percentage motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        servo.setSpeed(sign * power);
    }   //setMotorPower

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    @Override
    public double getMotorPower()
    {
        return servo.getSpeed() * sign;
    }   //getMotorPower

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     * @param acceleration specifies the max motor acceleration rotations per second square, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts.
     */
    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorVelocity.");
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in sensor units per second.
     */
    @Override
    public double getMotorVelocity()
    {
        throw new UnsupportedOperationException("CRServo does not support getMotorVelocity.");
    }   //getMotorVelocity

    /**
     * This method commands the motor to go to the given position using close loop control and optionally limits the
     * power of the motor movement.
     *
     * @param position specifies the position in rotations.
     * @param powerLimit specifies the maximum power output limits, can be null if not provided. If not provided, the
     *        previous set limit is applied.
     * @param velocity specifies the max motor velocity rotations per second, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts.
     */
    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorPosition.");
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in raw sensor units.
     */
    @Override
    public double getMotorPosition()
    {
        throw new UnsupportedOperationException("CRServo does not support position sensor.");
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorCurrent.");
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        throw new UnsupportedOperationException("CRServo does not support getMotorCurrent.");
    }   //getMotorCurrent

    /**
     * This method sets the PID coefficients of the motor controller's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorVelocityPidCoefficients.");
    }   //setMotorVelocityPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorVelocityPidCoefficients()
    {
        throw new UnsupportedOperationException("CRServo does not support getMotorVelocityPidCoefficients.");
    }   //getMotorVelocityPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorPositionPidCoefficients.");
    }   //setMotorPositionPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorPositionPidCoefficients()
    {
        throw new UnsupportedOperationException("CRServo does not support getMotorPositionPidCoefficients.");
    }   //getMotorPositionPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("CRServo does not support setMotorCurretPidCoefficients.");
    }   //setMotorCurrentPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients()
    {
        throw new UnsupportedOperationException("CRServo does not support getMotorCurretPidCoefficients.");
    }   //geteMotorCurrentPidCoefficients

    //
    // The following methods override the software simulation in TrcMotor providing direct support in hardware.
    //

}   //class FrcCRServo
