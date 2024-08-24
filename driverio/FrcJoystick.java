/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import trclib.dataprocessor.TrcUtil;
import trclib.driverio.TrcGameController;

/**
 * This class implements the platform dependent joystick. It provides monitoring of the joystick buttons. If the
 * caller of this class provides a button notification handler, it will call it when there are button events.
 */
public class FrcJoystick extends TrcGameController
{
    private static final int TRIGGER                    = (1);
    private static final int BUTTON_2                   = (1 << 1);
    private static final int BUTTON_3                   = (1 << 2);
    private static final int BUTTON_4                   = (1 << 3);
    private static final int BUTTON_5                   = (1 << 4);
    private static final int BUTTON_6                   = (1 << 5);
    private static final int BUTTON_7                   = (1 << 6);
    private static final int BUTTON_8                   = (1 << 7);
    private static final int BUTTON_9                   = (1 << 8);
    private static final int BUTTON_10                  = (1 << 9);
    private static final int BUTTON_11                  = (1 << 10);
    private static final int BUTTON_12                  = (1 << 11);

    public enum ButtonType
    {
        Trigger,
        Button2,
        Button3,
        Button4,
        Button5,
        Button6,
        Button7,
        Button8,
        Button9,
        Button10,
        Button11,
        Button12
    }   //enum ButtonType

    /**
     * This interface, if provided, will allow this class to do a notification callback when there are button
     * activities.
     */
    public interface ButtonEventHandler
    {
        /**
         * This method is called when button event is detected.
         *
         * @param buttonType specifies the button type that generates the event.
         * @param pressed specifies true if the button is pressed, false otherwise.
         */
        void buttonEvent(ButtonType buttonType, boolean pressed);

    }   //interface ButtonEventHandler

    private static final double DEF_DEADBAND_THRESHOLD = 0.15;
    private static HashMap<Integer, ButtonType> buttonTypeMap = null;
    private final int port;
    public final Joystick joystick;
    private ButtonEventHandler buttonEventHandler = null;
    private int xSign = 1, ySign = 1, zSign = 1, twistSign = 1;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the USB port the joystick is plugged into.
     * @param deadbandThreshold specifies the deadband of the analog joystick.
     */
    public FrcJoystick(String instanceName, int port, double deadbandThreshold)
    {
        super(instanceName, deadbandThreshold);
        this.port = port;
        this.joystick = new Joystick(port);
        if (buttonTypeMap == null)
        {
            buttonTypeMap = new HashMap<>();
            buttonTypeMap.put(TRIGGER, ButtonType.Trigger);
            buttonTypeMap.put(BUTTON_2, ButtonType.Button2);
            buttonTypeMap.put(BUTTON_3, ButtonType.Button3);
            buttonTypeMap.put(BUTTON_4, ButtonType.Button4);
            buttonTypeMap.put(BUTTON_5, ButtonType.Button5);
            buttonTypeMap.put(BUTTON_6, ButtonType.Button6);
            buttonTypeMap.put(BUTTON_7, ButtonType.Button7);
            buttonTypeMap.put(BUTTON_8, ButtonType.Button8);
            buttonTypeMap.put(BUTTON_9, ButtonType.Button9);
            buttonTypeMap.put(BUTTON_10, ButtonType.Button10);
            buttonTypeMap.put(BUTTON_11, ButtonType.Button11);
            buttonTypeMap.put(BUTTON_12, ButtonType.Button12);
            super.init();
        }
    }   //FrcJoystick

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the USB port the joystick is plugged into.
     */
    public FrcJoystick(String instanceName, int port)
    {
        this(instanceName, port, DEF_DEADBAND_THRESHOLD);
    }   //FrcJoystick

    /**
     * This method sets the button event handler.
     *
     * @param buttonEventHandler specifies button event notification handler, null to disable event notification.
     */
    public void setButtonEventHandler(ButtonEventHandler buttonEventHandler)
    {
        this.buttonEventHandler = buttonEventHandler;
        setButtonEventEnabled(buttonEventHandler != null);
    }   //setButtonEventHandler

    /**
     * This method inverts the analog axes.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     * @param zInverted specifies true if inverting the z-axis, false otherwise.
     * @param twistInverted specifies true if inverting the twist-axis, false otherwise.
     */
    public void setInverted(boolean xInverted, boolean yInverted, boolean zInverted, boolean twistInverted)
    {
        xSign = xInverted? -1: 1;
        ySign = yInverted? -1: 1;
        zSign = zInverted? -1: 1;
        twistSign = twistInverted? -1: 1;
    }   //setInverted

    /**
     * This method inverts the analog axes.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setInverted(boolean xInverted, boolean yInverted)
    {
        xSign = xInverted? -1: 1;
        ySign = yInverted? -1: 1;
    }   //setInverted

    /**
     * This method returns the x-axis value of the stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the stick.
     */
    public double getX(double cubicCoefficient)
    {
        return xSign * adjustAnalogControl(joystick.getX(), cubicCoefficient);
    }   //getX

    /**
     * This method returns the x-axis value of the stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return x-axis value of the stick.
     */
    public double getX(boolean doExp)
    {
        return xSign * adjustAnalogControl(joystick.getX(), doExp);
    }   //getX

    /**
     * This method returns the x-axis value of the stick.
     *
     * @return x-axis value of the stick.
     */
    public double getX()
    {
        return getX(false);
    }   //getX

    /**
     * This method returns the y-axis value of the stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return y-axis value of the stick.
     */
    public double getY(double cubicCoefficient)
    {
        return ySign * adjustAnalogControl(joystick.getY(), cubicCoefficient);
    }   //getY

    /**
     * This method returns the y-axis value of the stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return y-axis value of the stick.
     */
    public double getY(boolean doExp)
    {
        return ySign * adjustAnalogControl(joystick.getY(), doExp);
    }   //getY

    /**
     * This method returns the y-axis value of the stick.
     *
     * @return y-axis value of the stick.
     */
    public double getY()
    {
        return getY(false);
    }   //getY

    /**
     * This method returns the z-axis value of the stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return z-axis value of the stick.
     */
    public double getZ(double cubicCoefficient)
    {
        return zSign * adjustAnalogControl(joystick.getZ(), cubicCoefficient);
    }   //getZ

    /**
     * This method returns the z-axis value of the stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return z-axis value of the stick.
     */
    public double getZ(boolean doExp)
    {
        return zSign * adjustAnalogControl(joystick.getZ(), doExp);
    }   //getZ

    /**
     * This method returns the z-axis value of the stick.
     *
     * @return z-axis value of the stick.
     */
    public double getZ()
    {
        return getZ(false);
    }   //getZ

    /**
     * This method returns the twist-axis value of the stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return twist-axis value of the stick.
     */
    public double getTwist(double cubicCoefficient)
    {
        return twistSign * adjustAnalogControl(joystick.getTwist(), cubicCoefficient);
    }   //getTwist

    /**
     * This method returns the twist-axis value of the stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return twist-axis value of the stick.
     */
    public double getTwist(boolean doExp)
    {
        return twistSign * adjustAnalogControl(joystick.getTwist(), doExp);
    }   //getTwist

    /**
     * This method returns the twist-axis value of the stick.
     *
     * @return twist-axis value of the stick.
     */
    public double getTwist()
    {
        return getTwist(false);
    }   //getTwist

    /**
     * This method returns the throttle value of the stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return throttle value of the stick.
     */
    public double getThrottle(double cubicCoefficient)
    {
        return adjustAnalogControl(joystick.getThrottle(), cubicCoefficient);
    }   //getThrottle

    /**
     * This method returns the throttle value of the stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return throttle value of the stick.
     */
    public double getThrottle(boolean doExp)
    {
        return adjustAnalogControl(joystick.getThrottle(), doExp);
    }   //getThrottle

    /**
     * This method returns the throttle value of the stick.
     *
     * @return throttle value of the stick.
     */
    public double getThrottle()
    {
        return getThrottle(false);
    }   //getThrottle

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @param drivePowerScale specifies the scaling factor for drive power.
     * @param turnPowerScale specifies the scaling factor for turn power.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(
        DriveMode driveMode, boolean doExp, double drivePowerScale, double turnPowerScale)
    {
        double x = 0.0, y = 0.0, rot = 0.0;

        switch (driveMode)
        {
            case ArcadeMode:
                x = getX(doExp);
                y = getY(doExp);
                rot = getZ(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case HolonomicMode:
            case TankMode:
                throw new UnsupportedOperationException("Single joystick only supports Arcade Drive Mode.");
        }

        double mag = TrcUtil.magnitude(x, y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
        }
        x *= drivePowerScale;
        y *= drivePowerScale;
        rot *= turnPowerScale;

        return new double[] { x, y, rot };
    }   //getDriveInput

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(DriveMode driveMode, boolean doExp)
    {
        return getDriveInputs(driveMode, doExp, 1.0, 1.0);
    }   //getDriveInputs

    //
    // Implements TrcGameController abstract methods.
    //

    /**
     * This method returns the button states in an integer by combining all the button states.
     *
     * @return button states.
     */
    @Override
    public int getButtons()
    {
        int buttons = DriverStation.getStickButtons(port);
        tracer.traceDebug(instanceName, "buttons=0x" + Integer.toHexString(buttons));
        return buttons;
    }   //getButtons

    /**
     * This method is called when a button event is detected. It translates the button value into button type and
     * calls the button event handler.
     *
     * @param buttonValue specifies the button value that generated the event.
     * @param pressed specifies true if the button is pressed, false if it is released.
     */
    @Override
    protected void notifyButtonEvent(int buttonValue, boolean pressed)
    {
        if (buttonEventHandler != null)
        {
            ButtonType buttonType = buttonTypeMap.get(buttonValue);
            buttonEventHandler.buttonEvent(buttonType, pressed);
        }
    }   //notifyButtonEvent

}   //class FrcJoystick
