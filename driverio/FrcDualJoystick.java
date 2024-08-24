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
public class FrcDualJoystick extends TrcGameController
{
    private static final int LEFT_TRIGGER               = (1);
    private static final int LEFT_BUTTON_2              = (1 << 1);
    private static final int LEFT_BUTTON_3              = (1 << 2);
    private static final int LEFT_BUTTON_4              = (1 << 3);
    private static final int LEFT_BUTTON_5              = (1 << 4);
    private static final int LEFT_BUTTON_6              = (1 << 5);
    private static final int LEFT_BUTTON_7              = (1 << 6);
    private static final int LEFT_BUTTON_8              = (1 << 7);
    private static final int LEFT_BUTTON_9              = (1 << 8);
    private static final int LEFT_BUTTON_10             = (1 << 9);
    private static final int LEFT_BUTTON_11             = (1 << 10);
    private static final int LEFT_BUTTON_12             = (1 << 11);
    private static final int RIGHT_TRIGGER              = (1 << 12);
    private static final int RIGHT_BUTTON_2             = (1 << 13);
    private static final int RIGHT_BUTTON_3             = (1 << 14);
    private static final int RIGHT_BUTTON_4             = (1 << 15);
    private static final int RIGHT_BUTTON_5             = (1 << 16);
    private static final int RIGHT_BUTTON_6             = (1 << 17);
    private static final int RIGHT_BUTTON_7             = (1 << 18);
    private static final int RIGHT_BUTTON_8             = (1 << 19);
    private static final int RIGHT_BUTTON_9             = (1 << 20);
    private static final int RIGHT_BUTTON_10            = (1 << 21);
    private static final int RIGHT_BUTTON_11            = (1 << 22);
    private static final int RIGHT_BUTTON_12            = (1 << 23);

    public enum ButtonType
    {
        LeftTrigger,
        LeftButton2,
        LeftButton3,
        LeftButton4,
        LeftButton5,
        LeftButton6,
        LeftButton7,
        LeftButton8,
        LeftButton9,
        LeftButton10,
        LeftButton11,
        LeftButton12,
        RightTrigger,
        RightButton2,
        RightButton3,
        RightButton4,
        RightButton5,
        RightButton6,
        RightButton7,
        RightButton8,
        RightButton9,
        RightButton10,
        RightButton11,
        RightButton12
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
    private final int leftPort, rightPort;
    public final Joystick leftJoystick, rightJoystick;
    private ButtonEventHandler buttonEventHandler = null;
    private int leftXSign = 1, leftYSign = 1, leftZSign = 1, leftTwistSign = 1;
    private int rightXSign = 1, rightYSign = 1, rightZSign = 1, rightTwistSign = 1;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param leftPort specifies the USB port for the left joystick.
     * @param rightPort specifies the USB port for the left joystick.
     * @param deadbandThreshold specifies the deadband of the analog joysticks.
     */
    public FrcDualJoystick(String instanceName, int leftPort, int rightPort, double deadbandThreshold)
    {
        super(instanceName, deadbandThreshold);
        this.leftPort = leftPort;
        this.rightPort = rightPort;
        this.leftJoystick = new Joystick(leftPort);
        this.rightJoystick = new Joystick(rightPort);
        if (buttonTypeMap == null)
        {
            buttonTypeMap = new HashMap<>();
            buttonTypeMap.put(LEFT_TRIGGER, ButtonType.LeftTrigger);
            buttonTypeMap.put(LEFT_BUTTON_2, ButtonType.LeftButton2);
            buttonTypeMap.put(LEFT_BUTTON_3, ButtonType.LeftButton3);
            buttonTypeMap.put(LEFT_BUTTON_4, ButtonType.LeftButton4);
            buttonTypeMap.put(LEFT_BUTTON_5, ButtonType.LeftButton5);
            buttonTypeMap.put(LEFT_BUTTON_6, ButtonType.LeftButton6);
            buttonTypeMap.put(LEFT_BUTTON_7, ButtonType.LeftButton7);
            buttonTypeMap.put(LEFT_BUTTON_8, ButtonType.LeftButton8);
            buttonTypeMap.put(LEFT_BUTTON_9, ButtonType.LeftButton9);
            buttonTypeMap.put(LEFT_BUTTON_10, ButtonType.LeftButton10);
            buttonTypeMap.put(LEFT_BUTTON_11, ButtonType.LeftButton11);
            buttonTypeMap.put(LEFT_BUTTON_12, ButtonType.LeftButton12);
            buttonTypeMap.put(RIGHT_TRIGGER, ButtonType.RightTrigger);
            buttonTypeMap.put(RIGHT_BUTTON_2, ButtonType.RightButton2);
            buttonTypeMap.put(RIGHT_BUTTON_3, ButtonType.RightButton3);
            buttonTypeMap.put(RIGHT_BUTTON_4, ButtonType.RightButton4);
            buttonTypeMap.put(RIGHT_BUTTON_5, ButtonType.RightButton5);
            buttonTypeMap.put(RIGHT_BUTTON_6, ButtonType.RightButton6);
            buttonTypeMap.put(RIGHT_BUTTON_7, ButtonType.RightButton7);
            buttonTypeMap.put(RIGHT_BUTTON_8, ButtonType.RightButton8);
            buttonTypeMap.put(RIGHT_BUTTON_9, ButtonType.RightButton9);
            buttonTypeMap.put(RIGHT_BUTTON_10, ButtonType.RightButton10);
            buttonTypeMap.put(RIGHT_BUTTON_11, ButtonType.RightButton11);
            buttonTypeMap.put(RIGHT_BUTTON_12, ButtonType.RightButton12);
            super.init();
        }
    }   //FrcDualJoystick

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param leftPort specifies the USB port for the left joystick.
     * @param rightPort specifies the USB port for the left joystick.
     */
    public FrcDualJoystick(String instanceName, int leftPort, int rightPort)
    {
        this(instanceName, leftPort, rightPort, DEF_DEADBAND_THRESHOLD);
    }   //FrcDualJoystick

    /**
     * This method sets the button event handler for the joystick buttons.
     *
     * @param buttonEventHandler specifies button event notification handler, null to disable event notification.
     */
    public void setButtonEventHandler(ButtonEventHandler buttonEventHandler)
    {
        this.buttonEventHandler = buttonEventHandler;
        setButtonEventEnabled(buttonEventHandler != null);
    }   //setButtonEventHandler

    /**
     * This method inverts the analog axes of the left joystick.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     * @param zInverted specifies true if inverting the z-axis, false otherwise.
     * @param twistInverted specifies true if inverting the twist-axis, false otherwise.
     */
    public void setLeftStickInverted(boolean xInverted, boolean yInverted, boolean zInverted, boolean twistInverted)
    {
        leftXSign = xInverted? -1: 1;
        leftYSign = yInverted? -1: 1;
        leftZSign = zInverted? -1: 1;
        leftTwistSign = twistInverted? -1: 1;
    }   //setLeftStickInverted

    /**
     * This method inverts the analog axes of the left joystick.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setLeftStickInverted(boolean xInverted, boolean yInverted)
    {
        leftXSign = xInverted? -1: 1;
        leftYSign = yInverted? -1: 1;
    }   //setLeftStickInverted

    /**
     * This method inverts the analog axes of the right joystick.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     * @param zInverted specifies true if inverting the z-axis, false otherwise.
     * @param twistInverted specifies true if inverting the twist-axis, false otherwise.
     */
    public void setRightStickInverted(boolean xInverted, boolean yInverted, boolean zInverted, boolean twistInverted)
    {
        rightXSign = xInverted? -1: 1;
        rightYSign = yInverted? -1: 1;
        rightZSign = zInverted? -1: 1;
        rightTwistSign = twistInverted? -1: 1;
    }   //setRightStickInverted

    /**
     * This method inverts the analog axes of the right joystick.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setRightStickInverted(boolean xInverted, boolean yInverted)
    {
        rightXSign = xInverted? -1: 1;
        rightYSign = yInverted? -1: 1;
    }   //setRightStickInverted

    /**
     * This method returns the x-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(double cubicCoefficient)
    {
        return leftXSign * adjustAnalogControl(leftJoystick.getX(), cubicCoefficient);
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(boolean doExp)
    {
        return leftXSign * adjustAnalogControl(leftJoystick.getX(), doExp);
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX()
    {
        return getLeftStickX(false);
    }   //getLeftStickX

    /**
     * This method returns the y-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(double cubicCoefficient)
    {
        return leftYSign * adjustAnalogControl(leftJoystick.getY(), cubicCoefficient);
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(boolean doExp)
    {
        return leftYSign * adjustAnalogControl(leftJoystick.getY(), doExp);
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY()
    {
        return getLeftStickY(false);
    }   //getLeftStickY

    /**
     * This method returns the z-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return z-axis value of the left stick.
     */
    public double getLeftStickZ(double cubicCoefficient)
    {
        return leftZSign * adjustAnalogControl(leftJoystick.getZ(), cubicCoefficient);
    }   //getLeftStickZ

    /**
     * This method returns the z-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return z-axis value of the left stick.
     */
    public double getLeftStickZ(boolean doExp)
    {
        return leftZSign * adjustAnalogControl(leftJoystick.getZ(), doExp);
    }   //getLeftStickZ

    /**
     * This method returns the z-axis value of the left stick.
     *
     * @return z-axis value of the left stick.
     */
    public double getLeftStickZ()
    {
        return getLeftStickZ(false);
    }   //getLeftStickZ

    /**
     * This method returns the twist-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return twist-axis value of the left stick.
     */
    public double getLeftStickTwist(double cubicCoefficient)
    {
        return leftTwistSign * adjustAnalogControl(leftJoystick.getTwist(), cubicCoefficient);
    }   //getLeftStickTwist

    /**
     * This method returns the twist-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return twist-axis value of the left stick.
     */
    public double getLeftStickTwist(boolean doExp)
    {
        return leftTwistSign * adjustAnalogControl(leftJoystick.getTwist(), doExp);
    }   //getLeftStickTwist

    /**
     * This method returns the twist-axis value of the left stick.
     *
     * @return twist-axis value of the left stick.
     */
    public double getLeftStickTwist()
    {
        return getLeftStickTwist(false);
    }   //getLeftStickTwist

    /**
     * This method returns the x-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(double cubicCoefficient)
    {
        return rightXSign * adjustAnalogControl(rightJoystick.getX(), cubicCoefficient);
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(boolean doExp)
    {
        return rightXSign * adjustAnalogControl(rightJoystick.getX(), doExp);
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX()
    {
        return getRightStickX(false);
    }   //getRightStickX

    /**
     * This method returns the y-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(double cubicCoefficient)
    {
        return rightYSign * adjustAnalogControl(rightJoystick.getY(), cubicCoefficient);
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(boolean doExp)
    {
        return rightYSign * adjustAnalogControl(rightJoystick.getY(), doExp);
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY()
    {
        return getRightStickY(false);
    }   //getRightStickY

    /**
     * This method returns the z-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return z-axis value of the right stick.
     */
    public double getRightStickZ(double cubicCoefficient)
    {
        return rightZSign * adjustAnalogControl(rightJoystick.getZ(), cubicCoefficient);
    }   //getRightStickZ

    /**
     * This method returns the z-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return z-axis value of the right stick.
     */
    public double getRightStickZ(boolean doExp)
    {
        return rightZSign * adjustAnalogControl(rightJoystick.getZ(), doExp);
    }   //getRightStickZ

    /**
     * This method returns the z-axis value of the right stick.
     *
     * @return z-axis value of the right stick.
     */
    public double getRightStickZ()
    {
        return getRightStickZ(false);
    }   //getRightStickZ

    /**
     * This method returns the twist-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return twist-axis value of the right stick.
     */
    public double getRightStickTwist(double cubicCoefficient)
    {
        return rightTwistSign * adjustAnalogControl(rightJoystick.getTwist(), cubicCoefficient);
    }   //getRightStickTwist

    /**
     * This method returns the twist-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return twist-axis value of the right stick.
     */
    public double getRightStickTwist(boolean doExp)
    {
        return rightTwistSign * adjustAnalogControl(rightJoystick.getTwist(), doExp);
    }   //getRightStickTwist

    /**
     * This method returns the twist-axis value of the right stick.
     *
     * @return twist-axis value of the right stick.
     */
    public double getRightStickTwist()
    {
        return getRightStickTwist(false);
    }   //getRightStickTwist

    /**
     * This method returns the throttle value of the left joystick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return throttle value of the left joystick.
     */
    public double getLeftThrottle(double cubicCoefficient)
    {
        return adjustAnalogControl(leftJoystick.getThrottle(), cubicCoefficient);
    }   //getLeftThrottle

    /**
     * This method returns the throttle value of the left joystick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return throttle value of the left joystick.
     */
    public double getLeftThrottle(boolean doExp)
    {
        return adjustAnalogControl(leftJoystick.getThrottle(), doExp);
    }   //getLeftThrottle

    /**
     * This method returns the throttle value of the left joystick.
     *
     * @return throttle value of the left joystick.
     */
    public double getLeftThrottle()
    {
        return getLeftThrottle(false);
    }   //getLeftThrottle

    /**
     * This method returns the throttle value of the right joystick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return throttle value of the right joystick.
     */
    public double getRightThrottle(double cubicCoefficient)
    {
        return adjustAnalogControl(rightJoystick.getThrottle(), cubicCoefficient);
    }   //getRightThrottle

    /**
     * This method returns the throttle value of the right joystick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return throttle value of the right joystick.
     */
    public double getRightThrottle(boolean doExp)
    {
        return adjustAnalogControl(rightJoystick.getThrottle(), doExp);
    }   //getRightThrottle

    /**
     * This method returns the throttle value of the right joystick.
     *
     * @return throttle value of the right joystick.
     */
    public double getRightThrottle()
    {
        return getRightThrottle(false);
    }   //getRightThrottle

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
            case HolonomicMode:
                x = getRightStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getRightStickZ(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case ArcadeMode:
                x = getLeftStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getRightStickX(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case TankMode:
                double leftPower = getLeftStickY(doExp);
                double rightPower = getRightStickY(doExp);
                x = 0.0;
                y = (leftPower + rightPower)/2.0;
                rot = (leftPower - rightPower)/2.0;
                tracer.traceDebug(instanceName, driveMode + ":left=" + leftPower + ",right=" + rightPower);
                break;
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
        int buttons = DriverStation.getStickButtons(leftPort) | (DriverStation.getStickButtons(rightPort) << 12);
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

}   //class FrcDualJoystick
