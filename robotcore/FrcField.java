/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib.robotcore;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import trclib.pathdrive.TrcPose2D;

public class FrcField
{
    private static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final double fieldLength = Units.metersToInches(aprilTagFieldLayout.getFieldLength());
    private static final double fieldWidth = Units.metersToInches(aprilTagFieldLayout.getFieldWidth());

    /**
     * This method returns the Field Pose3d of the specified AprilTag ID.
     *
     * @param aprilTagId specifies the AprilTag ID to get its field pose.
     * @return field Pose3d of the specified AprilTag.
     */
    public static Pose3d getAprilTagFieldPose3d(int aprilTagId)
    {
        Optional<Pose3d> aprilTagPoseOptional = aprilTagFieldLayout.getTagPose(aprilTagId);
        return aprilTagPoseOptional.isPresent()? aprilTagPoseOptional.get(): null;
    }   //getAprilTagFieldPose3d

    /**
     * This method returns the Field TrcPose2D pose of the specified AprilTag ID.
     *
     * @param aprilTagId specifies the AprilTag ID to get its field pose.
     * @return field pose of the specified AprilTag.
     */
    public static TrcPose2D getAprilTagFieldPose(int aprilTagId)
    {
        TrcPose2D aprilTagFieldPose = null;
        Pose3d aprilTagPose3d = getAprilTagFieldPose3d(aprilTagId);

        if (aprilTagPose3d != null)
        {
            Translation2d translation2d = aprilTagPose3d.getTranslation().toTranslation2d();
            Rotation2d rotation2d = aprilTagPose3d.getRotation().toRotation2d();
            aprilTagFieldPose = new TrcPose2D(
                -Units.metersToInches(translation2d.getY()),
                Units.metersToInches(translation2d.getX()),
                -rotation2d.getDegrees());
        }

        return aprilTagFieldPose;
    }   //getAprilTagFieldPose

    /**
     * This method returns the field length in inches.
     *
     * @return field length.
     */
    public static double getFieldLength()
    {
        return fieldLength;
    }   //getFieldLength

    /**
     * This method returns the field width in inches.
     *
     * @return field width.
     */
    public static double getFieldWidth()
    {
        return fieldWidth;
    }   //getFieldWidth

}   //class FrcField
