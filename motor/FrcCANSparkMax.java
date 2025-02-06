/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcAbsoluteEncoder;

/**
 * This class implements a SparkMAX motor controller by REV robototics. It extends the TrcMotor class and
 * implements the abstract methods required by TrcMotor to be compatible with the TRC library.
 * Reference manual and API doc of the motor controller can be found here:
 * http://www.revrobotics.com/sparkmax-users-manual/?mc_cid=a60a44dc08&mc_eid=1935741b98#section-2-3
 * https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
 * https://docs.revrobotics.com/revlib/24-to-25
 */
public class FrcCANSparkMax extends TrcMotor
{
    private static final ClosedLoopSlot PIDSLOT_POSITION = ClosedLoopSlot.kSlot0;
    private static final ClosedLoopSlot PIDSLOT_VELOCITY = ClosedLoopSlot.kSlot1;
    private static final ClosedLoopSlot PIDSLOT_CURRENT = ClosedLoopSlot.kSlot2;

    public final SparkMax motor;
    private final SparkClosedLoopController pidCtrl;
    private final RelativeEncoder relativeEncoder;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final TrcAbsoluteEncoder absEncoderConverter;
    public SparkMaxConfig config;
    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private REVLibError lastError = null;
    private double zeroOffset = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param brushless specifies true if the motor is brushless, false otherwise.
     * @param absEncoder specifies true if uses DutyCycle absolute encoder, false to use relative encoder.
     * @param sensors specifies external sensors, can be null if none.
     */
    public FrcCANSparkMax(
        String instanceName, int canId, boolean brushless, boolean absEncoder, TrcMotor.ExternalSensors sensors)
    {
        super(instanceName, sensors);
        motor = new SparkMax(canId, brushless? MotorType.kBrushless: MotorType.kBrushed);
        pidCtrl = motor.getClosedLoopController();
        if (absEncoder)
        {
            relativeEncoder = null;
            absoluteEncoder = motor.getAbsoluteEncoder();    //getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            absEncoderConverter = new TrcAbsoluteEncoder(instanceName, absoluteEncoder::getPosition, 0.0, 1.0);
            absEncoderConverter.setTaskEnabled(true);
        }
        else
        {
            relativeEncoder = motor.getEncoder();
            absoluteEncoder = null;
            absEncoderConverter = null;
        }
        config = new SparkMaxConfig();
    }   //FrcCANSparkMax

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param brushless specifies true if the motor is brushless, false otherwise.
     * @param absEncoder specifies true if uses DutyCycle absolute encoder, false to use relative encoder.
     */
    public FrcCANSparkMax(String instanceName, int canId, boolean brushless, boolean absEncoder)
    {
        this(instanceName, canId, brushless, absEncoder, null);
    }   //FrcCANSparkMax

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param brushless specifies true if the motor is brushless, false otherwise.
     */
    public FrcCANSparkMax(String instanceName, int canId, boolean brushless)
    {
        this(instanceName, canId, brushless, false, null);
    }   //FrcCANSparkMax

    /**
     * This method returns the number of error responses seen from the motor after sending a command.
     *
     * @return The number of non-OK error code responses seen from the motor
     * after sending a command.
     */
    public int getErrorCount()
    {
        return errorCount;
    } //getErrorCount

    /**
     * The method returns the last error code. If there is none, null is returned.
     *
     * @return last error code.
     */
    public REVLibError getLastError()
    {
        return lastError;
    }   //getLastError

    /**
     * This method checks for error code returned by the motor controller executing the last command. If there was
     * an error, the error count is incremented.
     *
     * @param operation specifies the operation that failed.
     * @param errorCode specifies the error code returned by the motor controller.
     */
    private REVLibError recordResponseCode(String operation, REVLibError errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(REVLibError.kOk))
        {
            errorCount++;
            tracer.traceErr(instanceName, operation + " (ErrCode=" + errorCode + ")");
        }
        return errorCode;
    }   //recordResponseCode

    /**
     * This method returns the motor type.
     *
     * @return true if the motor is brushless, false otherwise.
     */
    public boolean isBrushless()
    {
        return motor.getMotorType() == MotorType.kBrushless;
    }   //isBrushless

    //
    // Implements TrcMotorController interface.
    //

    // /**
    //  * This method is used to check if the motor controller supports close loop control natively.
    //  *
    //  * @return true if motor controller supports close loop control, false otherwise.
    //  */
    // public boolean supportCloseLoopControl()
    // {
    //     return true;
    // }   // supportCloseLoopControl

    // /**
    //  * This method checks if the device is connected to the robot.
    //  *
    //  * @return true if the device is connected, false otherwise.
    //  */
    // @Override
    // public boolean isConnected()
    // {
    //     // Hacky, but should work
    //     return motor.getFirmwareString() != null;
    // }   //isConnected

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        config = new SparkMaxConfig();
        recordResponseCode(
            "resetFactoryDefault",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //resetFactoryDefault

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return motor.getBusVoltage();
    }   //getBusVoltage

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent not used. SparkMax does not support this.
     * @param triggerThresholdTime not used. SparkMax does not support this.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        config.smartCurrentLimit((int) currentLimit);
        recordResponseCode(
            "setCurrentLimit",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setCurrentLimit

    /**
     * This method sets the stator current limit of the motor.
     *
     * @param currentLimit specifies the stator current limit in amperes.
     */
    @Override
    public void setStatorCurrentLimit(double currentLimit)
    {
        throw new UnsupportedOperationException("Motor controller does not support setStatorCurrentLimit.");
    }   //setStatorCurrentLimit

    // /**
    //  * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
    //  * By setting a non-default limits, it effectively limits the output power of the close loop control.
    //  *
    //  * @param revLimit specifies the percentage output limit of the reverse direction.
    //  * @param fwdLimit specifies the percentage output limit of the forward direction.
    //  */
    // @Override
    // public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
    // {
    //     recordResponseCode("setOutputRange", pidCtrl.setOutputRange(revLimit, fwdLimit));
    // }   //setCloseLoopOutputLimits

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        config.closedLoopRampRate(rampTime);
        recordResponseCode(
            "setClosedLoopRampRate",
             motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        config.openLoopRampRate(rampTime);
        recordResponseCode(
            "setOpenLoopRampRate",
             motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
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
        config.idleMode(enabled? IdleMode.kBrake: IdleMode.kCoast);
        recordResponseCode(
            "setIdleMode",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        // sparkMaxRevLimitSwitch = motor.getReverseLimitSwitch();
        config.limitSwitch.reverseLimitSwitchType(normalClose? Type.kNormallyClosed: Type.kNormallyOpen);
        config.limitSwitch.reverseLimitSwitchEnabled(true);
        recordResponseCode(
            "enableRevLimitSwitch",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        // sparkMaxFwdLimitSwitch = motor.getForwardLimitSwitch();
        config.limitSwitch.forwardLimitSwitchType(normalClose? Type.kNormallyClosed: Type.kNormallyOpen);
        config.limitSwitch.forwardLimitSwitchEnabled(true);
        recordResponseCode(
            "enableFwdLimitSwitch",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        config.limitSwitch.reverseLimitSwitchEnabled(false);
        recordResponseCode(
            "disableRevLimitSwitch",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        config.limitSwitch.forwardLimitSwitchEnabled(false);
        recordResponseCode(
            "disableFwdLimitSwitch",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //disableMotorFwdLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        return motor.configAccessor.limitSwitch.getReverseLimitSwitchEnabled();
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        return motor.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
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
        throw new UnsupportedOperationException(
            "SparkMax does not support inverting limit switch. Use enableMotorRevLimitSwitch to configure its type.");
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
        throw new UnsupportedOperationException(
            "SparkMax does not support inverting limit switch. Use enableMotorFwdLimitSwitch to configure its type.");
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        return motor.getReverseLimitSwitch().isPressed();
    }   //isMotorRevLimitSwitchClosed

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        return motor.getForwardLimitSwitch().isPressed();
    }   //isMotorFwdLimitSwitchActive

    /**
     * This method sets the soft position limit for the reverse direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorRevSoftPositionLimit(Double limit)
    {
        if (limit != null)
        {
            config.softLimit.reverseSoftLimit(limit);
            config.softLimit.reverseSoftLimitEnabled(true);
        }
        else
        {
            config.softLimit.reverseSoftLimitEnabled(false);
        }
        recordResponseCode(
            "setReverseSoftLimit",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setMotorRevSoftPositionLimit

    /**
     * This method sets the soft position limit for the forward direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorFwdSoftPositionLimit(Double limit)
    {
        if (limit != null)
        {
            config.softLimit.forwardSoftLimit(limit);
            config.softLimit.forwardSoftLimitEnabled(true);
        }
        else
        {
            config.softLimit.forwardSoftLimitEnabled(false);
        }
        recordResponseCode(
            "setForwardSoftLimit",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

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
        if (absoluteEncoder != null)
        {
            config.absoluteEncoder.inverted(inverted);
        }
        else
        {
            config.encoder.inverted(inverted);
        }
        recordResponseCode(
            "setEncoderInverted",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        if (absoluteEncoder != null)
        {
            return motor.configAccessor.absoluteEncoder.getInverted();
        }
        else
        {
            return motor.configAccessor.encoder.getInverted();
        }
    }   //isMotorPositionSensorInverted

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    public void resetMotorPosition(boolean hardware)
    {
        if (hardware)
        {
            // Only relative encoder allows reset its position.
            if (absoluteEncoder != null)
            {
                config.absoluteEncoder.zeroOffset(absoluteEncoder.getPosition());
                recordResponseCode(
                    "absEncoderReset", motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
            }
            else if (relativeEncoder != null)
            {
                recordResponseCode("relEncoderReset", relativeEncoder.setPosition(0.0));
            }
        }
        else
        {
            zeroOffset = getRawMotorPosition();
        }
    }   //resetMotorPosition

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        resetMotorPosition(true);
    }   //resetMotorPosition

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        config.inverted(inverted);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.configAccessor.getInverted();
    }   //isMotorInverted

    /**
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        motor.set(power);
    }   //setMotorPower

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    @Override
    public double getMotorPower()
    {
        // Motor power might have changed by closed loop controls, so get it directly from the motor.
        return motor.getAppliedOutput();
    }   //getMotorPower

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     * @param acceleration specifies the max motor acceleration rotations per second square (not supported).
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1 (not supported).
     */
    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward)
    {
        // setVelocity takes a velocity value in RPM.
        recordResponseCode(
            "setVelocity", pidCtrl.setReference(velocity*60.0, ControlType.kVelocity, PIDSLOT_VELOCITY));
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in rotations per second.
     */
    @Override
    public double getMotorVelocity()
    {
        return (relativeEncoder != null? relativeEncoder.getVelocity(): absoluteEncoder.getVelocity()) / 60.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to go to the given position using close loop control and optionally limits the
     * power of the motor movement.
     *
     * @param position specifies the position in rotations.
     * @param powerLimit specifies the maximum power output limits, can be null if not provided. If not provided, the
     *        previous set limit is applied.
     * @param velocity specifies the max motor veloicty rotations per second (not supportec).
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1 (not supported).
     */
    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward)
    {
        if (powerLimit != null)
        {
            config.closedLoop.outputRange(-powerLimit, powerLimit, PIDSLOT_POSITION);
            recordResponseCode(
                "setOutputRange",
                motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        }
        recordResponseCode("setPosition", pidCtrl.setReference(position, ControlType.kPosition, PIDSLOT_POSITION));
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in number of rotations.
     */
    private double getRawMotorPosition()
    {
        return relativeEncoder != null? relativeEncoder.getPosition(): absEncoderConverter.getContinuousValue();
    }   //getRawMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in number of rotations.
     */
    @Override
    public double getMotorPosition()
    {
        return getRawMotorPosition() - zeroOffset;
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        recordResponseCode("setCurrent", pidCtrl.setReference(current, ControlType.kCurrent, PIDSLOT_CURRENT));
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        return motor.getOutputCurrent();
    }   //getMotorCurrent

    /**
     * This method sets the PID coefficients of the specified slot.
     *
     * @param pidSlot specifies the PID slot.
     * @param pidCoeff specifies the PID coefficients to set.
     */
    private void setPidCoefficients(ClosedLoopSlot pidSlot, TrcPidController.PidCoefficients pidCoeff)
    {
        config.closedLoop.pidf(pidCoeff.kP, pidCoeff.kI, pidCoeff.kD, pidCoeff.kF, pidSlot);
        config.closedLoop.iZone(pidCoeff.iZone, pidSlot);
        recordResponseCode(
            "setPIDF",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }   //setPidCoefficients

    /**
     * This method returns the PID coefficients of the specified slot.
     *
     * @param pidSlot specifies the PID slot.
     * @return PID coefficients of the motor's PID controller.
     */
    private TrcPidController.PidCoefficients getPidCoefficients(ClosedLoopSlot pidSlot)
    {
        return new TrcPidController.PidCoefficients(
            motor.configAccessor.closedLoop.getP(pidSlot),
            motor.configAccessor.closedLoop.getI(pidSlot),
            motor.configAccessor.closedLoop.getD(pidSlot),
            motor.configAccessor.closedLoop.getFF(pidSlot),
            motor.configAccessor.closedLoop.getIZone(pidSlot));
    }   //getPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_VELOCITY, pidCoeff);
    }   //setMotorVelocityPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorVelocityPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_VELOCITY);
    }   //getMotorVelocityPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_POSITION, pidCoeff);
    }   //setMotorPositionPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorPositionPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_POSITION);
    }   //getMotorPositionPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_CURRENT, pidCoeff);
    }   //setMotorCurrentPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_CURRENT);
    }   //geteMotorCurrentPidCoefficients

    //
    // The following methods override the software simulation in TrcMotor providing direct support in hardware.
    //

    /**
     * This method enables/disables voltage compensation so that it will maintain the motor output regardless of
     * battery voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery to enable, null to disable.
     */
    @Override
    public void setVoltageCompensationEnabled(Double batteryNominalVoltage)
    {
        if (batteryNominalVoltage != null)
        {
            config.voltageCompensation(batteryNominalVoltage);
        }
        else
        {
            config.disableVoltageCompensation();
        }
        recordResponseCode(
            "setVoltageCompensation",
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
}   //setVoltageCompensationEnabled

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    @Override
    public boolean isVoltageCompensationEnabled()
    {
        return motor.configAccessor.getVoltageCompensationEnabled();
    }   //isVoltageCompensationEnabled

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     * @param inverted specifies true if this motor is inverted from the motor it is following, false otherwise.
     * @param scale specifies the value scale for the follower motor, 1.0 by default.
     */
    @Override
    public void follow(TrcMotor otherMotor, boolean inverted, double scale)
    {
        if (scale == 1.0 && otherMotor instanceof FrcCANSparkMax)
        {
            // Can only follow the same type of motor natively and scale must be 1.0.
            ((FrcCANSparkMax) otherMotor).addFollower(this, scale, true);
            config.follow(((FrcCANSparkMax) otherMotor).motor, inverted);
            recordResponseCode(
                "follow", motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        }
        else
        {
            super.follow(otherMotor, inverted, scale);
        }
    }   //follow

}   //class FrcCANSparkMax
