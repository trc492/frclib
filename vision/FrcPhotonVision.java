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

package frclib.vision;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionPerformanceMetrics;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements vision detection using PhotonLib extending PhotonCamera.
 */
public abstract class FrcPhotonVision extends PhotonCamera
{
    // private static final String moduleName = FrcPhotonVision.class.getSimpleName();
    // private static final TrcDbgTrace staticTracer = new TrcDbgTrace();

    /**
     * This method is provided by the subclass to provide the target offset from ground so that vision can
     * accurately calculate the target position from the camera.
     *
     * @param target specifies the photon detected target.
     * @return target ground offset in inches.
     */
    public abstract double getTargetGroundOffset(PhotonTrackedTarget target);

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final double timestamp;
        public final PhotonTrackedTarget target;
        public final Rect rect;
        public final double area;
        public final TrcPose2D targetPose;
        public final TrcPose2D robotPose;
        public final Point[] corners = new Point[4];
        public final double pixelWidth, pixelHeight, rotatedRectAngle;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param timestamp specifies the time stamp of the frame it was taken.
         * @param target specifies the photon detected target.
         * @param robotToCamera specifies the Transform3d of the camera position on the robot.
         * @param robotPose specifies the estimated robot pose.
         */
        public DetectedObject(
            double timestamp, PhotonTrackedTarget target, Transform3d robotToCamera, TrcPose2D robotPose)
        {
            this.timestamp = timestamp;
            this.target = target;
            this.rect = getObjectRect();
            this.area = target.getArea();
            this.targetPose = getTargetPose(target, target.getBestCameraToTarget(), robotToCamera);
            this.robotPose = robotPose;
            for (int i = 0; i < corners.length; i++)
            {
                corners[i] = new Point(target.minAreaRectCorners.get(i).x, target.minAreaRectCorners.get(i).y);
            }
            double side1 = TrcUtil.magnitude(corners[1].x - corners[0].x, corners[1].y - corners[0].y);
            double side2 = TrcUtil.magnitude(corners[2].x - corners[1].x, corners[2].y - corners[1].y);
            if (side2 > side1)
            {
                pixelWidth = side1;
                pixelHeight = side2;
                rotatedRectAngle = Math.toDegrees(Math.atan((corners[1].y - corners[0].y) / (corners[1].x - corners[0].x)));
            }
            else
            {
                pixelWidth = side2;
                pixelHeight = side1;
                rotatedRectAngle = Math.toDegrees(Math.atan((corners[2].y - corners[1].y) / (corners[2].x - corners[1].x)));
            }
        }   //DetectedObject

        /**
         * This method adds a transform to the detected target and returns the result 2D pose projected on the ground.
         *
         * @param target specifies the photon detected target object.
         * @param robotToCam specifies the Transform3d of the camera position on the robot.
         * @param transform specifies the transform to be added to the detected target.
         * @return 2D pose of the new target projected on the ground.
         */
        public TrcPose2D addTransformToTarget(
            PhotonTrackedTarget target, Transform3d robotToCam, Transform3d transform)
        {
            return getTargetPose(target, target.getBestCameraToTarget().plus(transform), robotToCam);
        }   //addTransformToTarget

        /**
         * This method calculates the rectangle of the detected AprilTag.
         *
         * @param corners specifies the corners of the MinAreaRect.
         * @return AprilTag rectangle.
         */
        public static Rect getDetectedRect(Point[] corners)
        {
            Rect rect = null;

            if (corners != null &&
                corners[0] != null && corners[1] != null && corners[2] != null && corners[3] != null)
            {
                double xMin = Math.min(corners[0].x, corners[3].x);
                double xMax = Math.max(corners[1].x, corners[2].x);
                double yMin = Math.min(corners[2].y, corners[3].y);
                double yMax = Math.max(corners[0].y, corners[1].y);
                rect = new Rect((int)xMin, (int)yMin, (int)(xMax - xMin), (int)(yMax - yMin));
            }

            return rect;
        }   //getDetectedRect

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            // Calculate rect from AprilTag detection corner points.
            return getDetectedRect(corners);
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            return area;
        }   //getObjectArea

        // /**
        //  * This method returns the rect of the detected object.
        //  *
        //  * @param target specifies the detected target.
        //  * @return rect of the detected target.
        //  */
        // private Rect getRect(PhotonTrackedTarget target)
        // {
        //     Rect rect = null;
        //     List<TargetCorner> corners = target.getDetectedCorners();
        //     TargetCorner lowerLeftCorner = null;
        //     TargetCorner lowerRightCorner = null;
        //     TargetCorner upperLeftCorner = null;
        //     TargetCorner upperRightCorner = null;

        //     if (corners != null && corners.size() >= 4)
        //     {
        //         lowerLeftCorner = corners.get(0);
        //         lowerRightCorner = corners.get(1);
        //         upperRightCorner = corners.get(2);
        //         upperLeftCorner = corners.get(3);
        //     }
        //     else if ((corners = target.getMinAreaRectCorners()) != null && corners.size() >= 4)
        //     {
        //         upperLeftCorner = corners.get(0);
        //         upperRightCorner = corners.get(1);
        //         lowerRightCorner = corners.get(2);
        //         lowerLeftCorner = corners.get(3);
        //     }

        //     if (upperLeftCorner != null)
        //     {
        //         double width =
        //             ((upperRightCorner.x - upperLeftCorner.x) + (lowerRightCorner.x - lowerLeftCorner.x))/2.0;
        //         double height =
        //             ((lowerLeftCorner.y - upperLeftCorner.y) + (lowerRightCorner.y - upperRightCorner.y))/2.0;
        //         rect = new Rect((int)upperLeftCorner.x, (int)upperLeftCorner.y, (int)width, (int)height);
        //         staticTracer.traceDebug(
        //             moduleName + ".Id" + target.getFiducialId(),
        //             " UpperLeft: x=" + upperLeftCorner.x + ", y=" + upperLeftCorner.y +
        //             "\nUpperRight: x=" + upperRightCorner.x + ", y=" + upperRightCorner.y +
        //             "\n LowerLeft: x=" +  lowerLeftCorner.x + ", y=" + lowerLeftCorner.y +
        //             "\nLowerRight: x=" +  lowerRightCorner.x + ", y=" + lowerRightCorner.y);
        //     }

        //     return rect;
        // }   //getRect

        /**
         * This method returns the object's pixel width.
         *
         * @return object pixel width, null if not supported.
         */
        @Override
        public Double getPixelWidth()
        {
            return pixelWidth;
        }   //getPixelWidth

        /**
         * This method returns the object's pixel height.
         *
         * @return object pixel height, null if not supported.
         */
        @Override
        public Double getPixelHeight()
        {
            return pixelHeight;
        }   //getPixelHeight

        /**
         * This method returns the object's rotated rectangle angle.
         *
         * @return rotated rectangle angle.
         */
        @Override
        public Double getRotatedRectAngle()
        {
            return rotatedRectAngle;
        }   //getRotatedRectAngle

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose2D getObjectPose()
        {
            return targetPose.clone();
        }   //getObjectPose

        /**
         * This method returns the objects real world width.
         *
         * @return object real world width, null if not supported.
         */
        @Override
        public Double getObjectWidth()
        {
            return null;
        }   //getObjectWidth

        /**
         * This method returns the objects real world depth.
         *
         * @return object real world depth, null if not supported.
         */
        @Override
        public Double getObjectDepth()
        {
            return null;
        }   //getObjectDepth

        /**
         * This method returns the rotated rect vertices of the detected object.
         *
         * @return rotated rect vertices.
         */
        @Override
        public Point[] getRotatedRectVertices()
        {
            return corners;
        }   //getRotatedRectVertices

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "{pose=" + targetPose +
                   ",robotPose=" + robotPose +
                   ",target=" + target + "}";
        }   //toString

    }   //class DetectedObject

    private static AprilTagFieldLayout fieldLayout = null;
    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    private final Transform3d robotToCamera;
    private TrcVisionPerformanceMetrics performanceMetrics = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCamera specifies the Transform3d of the camera position on the robot.
     * @param fieldLayoutType specifies the field layout.
     */
    public FrcPhotonVision(String cameraName, Transform3d robotToCamera, AprilTagFields fieldLayoutType)
    {
        super(cameraName);
        this.tracer = new TrcDbgTrace();
        this.instanceName = cameraName;
        this.robotToCamera = robotToCamera;
        if (fieldLayout == null)
        {
            fieldLayout = AprilTagFieldLayout.loadField(fieldLayoutType);
        }
    }   //FrcPhotonVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCamera specifies the Transform3d of the camera position on the robot.
     */
    public FrcPhotonVision(String cameraName, Transform3d robotToCamera)
    {
        this(cameraName, robotToCamera, AprilTagFields.kDefaultField);
    }   //FrcPhotonVision

    /**
     * This method returns the photon camera name.
     *
     * @return photon camera name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables performance metrics.
     *
     * @param enabled specifies true to enable performance metrics, false to disable.
     */
    public void setPerformanceMetricsEnabled(boolean enabled)
    {
        if (performanceMetrics == null && enabled)
        {
            performanceMetrics = new TrcVisionPerformanceMetrics(instanceName);
        }
        else if (performanceMetrics != null && !enabled)
        {
            performanceMetrics = null;
        }
    }   //setPerformanceMetricsEnabled

    /**
     * This method prints the performance metrics to the trace log.
     */
    public void printPerformanceMetrics()
    {
        if (performanceMetrics != null)
        {
            performanceMetrics.printMetrics(tracer);
        }
    }   //printPerformanceMetrics

    /**
     * This method projects a given tanslation and rotation transform a TrcPose2D on the ground.
     *
     * @param translation specifies the translation in 3D.
     * @param rotation specifies the rotation in 3D.
     * @return projected pose on the ground.
     */
    public static TrcPose2D projectTo2d(Translation3d translation, Rotation3d rotation)
    {
        Translation2d translation2d = translation.toTranslation2d();
        Rotation2d rotation2d = rotation.toRotation2d();
        return new TrcPose2D(
            -Units.metersToInches(translation2d.getY()),
            Units.metersToInches(translation2d.getX()),
            -rotation2d.getDegrees());
    }   //projectTo2d

    /**
     * This method projects a Pose3d to a TrcPose2D on the ground.
     *
     * @param pose specifies the pose in 3D.
     * @return projected pose on the ground.
     */
    public static TrcPose2D projectPose3dTo2d(Pose3d pose3d)
    {
        return projectTo2d(pose3d.getTranslation(), pose3d.getRotation());
    }   //projectPose3dTo2d

    /**
     * This method returns the 3D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @param fieldLayoutType specifies the season field type, null if use default field.
     * @return 3D location of the AprilTag.
     */
    public static Pose3d getAprilTagFieldPose3d(int aprilTagId, AprilTagFields fieldLayoutType)
    {
        if (fieldLayout == null)
        {
            fieldLayout = AprilTagFieldLayout.loadField(
                fieldLayoutType != null? fieldLayoutType: AprilTagFields.kDefaultField);
        }

        Optional<Pose3d> aprilTagPoseOptional = fieldLayout.getTagPose(aprilTagId);
        return aprilTagPoseOptional.isPresent()? aprilTagPoseOptional.get(): null;
    }   //getAprilTagFieldPose3d

    /**
     * This method returns the 2D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @param fieldLayoutType specifies the season field type, null if use default field.
     * @return 2D location of the AprilTag.
     */
    public static TrcPose2D getAprilTagFieldPose(int aprilTagId, AprilTagFields fieldLayoutType)
    {
        return projectPose3dTo2d(getAprilTagFieldPose3d(aprilTagId, fieldLayoutType));
    }   //getAprilTagFieldPose

    /**
     * This method returns the 2D field location of the AprilTag with its given ID.
     *
     * @param aprilTagId sepcifies the AprilTag ID to retrieve its field location.
     * @return 2D location of the AprilTag.
     */
    public static TrcPose2D getAprilTagFieldPose(int aprilTagId)
    {
        TrcPose2D aprilTagFieldPose = projectPose3dTo2d(getAprilTagFieldPose3d(aprilTagId, null));
        aprilTagFieldPose.angle += 180.0;
        aprilTagFieldPose.angle %= 360.0;
        return aprilTagFieldPose;
    }   //getAprilTagFieldPose

    /**
     * This method calculates the target pose of the detected object. If PhotonVision 3D model is enabled
     * (transform3d.translation3d is not zero), it will use the 3D info to calculate the detected object pose
     * projected on the ground. Otherwise, it will use the 2D model (yaw and pitch angles).
     *
     * @param target specifies the Photon detected target.
     * @param camToTarget specifies the Transform3d of the target position from the camera.
     * @param robotToCam specifies the Transform3d of the camera position on the robot.
     * @return target pose from the camera.
     */
    private TrcPose2D getTargetPose(PhotonTrackedTarget target, Transform3d camToTarget, Transform3d robotToCam)
    {
        TrcPose2D targetPose = null;

        if (camToTarget.getX() != 0.0 || camToTarget.getY() != 0.0 || camToTarget.getZ() != 0.0)
        {
            // Transform3d robotToTarget = robotToCam.plus(camToTarget);
            // targetPose = projectTo2d(robotToTarget.getTranslation(), robotToTarget.getRotation());
            // Use PhotonVision 3D model.
            Transform3d translatedCamTransform = new Transform3d(
                new Translation3d(0, 0, 0), robotToCam.getRotation());
            Transform3d projectedCamToTarget = translatedCamTransform.plus(camToTarget);
            Translation2d camToTargetTranslation = projectedCamToTarget.getTranslation().toTranslation2d();
            // Rotation2d camToTargetRotation = projectedCamToTarget.getRotation().toRotation2d();
            var tagXAxisBlock = projectedCamToTarget.getRotation().toMatrix().transpose().block(3, 1, 0, 0);
            Vector3D tagXAxis = new Vector3D(tagXAxisBlock.get(0, 0), tagXAxisBlock.get(1, 0), 0);
            tagXAxis = tagXAxis.normalize().negate();
            // tracer.traceInfo(instanceName, tagXAxis.toString());
            Vector3D robotForward = new Vector3D(1, 0, 0);
            double angle = Math.atan2(
                Vector3D.crossProduct(tagXAxis, robotForward).getNorm(),
                Vector3D.dotProduct(tagXAxis, robotForward));
            angle *= Math.signum(tagXAxis.dotProduct(new Vector3D(0, 1, 0)));
            double deltaX = Units.metersToInches(-camToTargetTranslation.getY());
            double deltaY = Units.metersToInches(camToTargetTranslation.getX());
            // double deltaAngle = Math.toDegrees(Math.atan(deltaX / deltaY));
            targetPose = new TrcPose2D(deltaX, deltaY, Units.radiansToDegrees(angle));
        }
        else
        {
            // Use PhotonVision 2D model.
            double camPitchRadians = -robotToCam.getRotation().getY();
            double targetPitchRadians = Units.degreesToRadians(target.getPitch());
            double targetYawDegrees = target.getYaw();
            double targetYawRadians = Units.degreesToRadians(targetYawDegrees);
            double targetDistanceInches =
                (getTargetGroundOffset(target) - Units.metersToInches(robotToCam.getZ())) /
                Math.tan(camPitchRadians + targetPitchRadians);
            targetPose = new TrcPose2D(
                targetDistanceInches * Math.sin(targetYawRadians),
                targetDistanceInches * Math.cos(targetYawRadians),
                targetYawDegrees);
        }

        return targetPose;
    }   //getTargetPose

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    public DetectedObject[] getDetectedObjects()
    {
        DetectedObject[] detectedObjs = null;
        double startTime = TrcTimer.getCurrentTime();
        List<PhotonPipelineResult> results = getAllUnreadResults();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (!results.isEmpty())
        {
            PhotonPipelineResult result = results.get(results.size() - 1);

            if (result.hasTargets())
            {
                List<PhotonTrackedTarget> targets = result.getTargets();
                double timestamp = result.getTimestampSeconds();

                detectedObjs = new DetectedObject[targets.size()];
                for (int i = 0; i < targets.size(); i++)
                {
                    PhotonTrackedTarget target = targets.get(i);
                    detectedObjs[i] = new DetectedObject(
                        timestamp, target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
                    tracer.traceDebug(instanceName, "[" + i + "] DetectedObj=" + detectedObjs[i]);
                }
            }
        }

        return detectedObjs;
    }   //getDetectedObjects

    /**
     * This method returns the best detected object.
     *
     * @return best detected object.
     */
    public DetectedObject getBestDetectedObject()
    {
        DetectedObject bestDetectedObj = null;
        double startTime = TrcTimer.getCurrentTime();
        List<PhotonPipelineResult> results = getAllUnreadResults();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (!results.isEmpty())
        {
            PhotonPipelineResult result = results.get(results.size() - 1);

            if (result.hasTargets())
            {
                PhotonTrackedTarget target = result.getBestTarget();
                bestDetectedObj = new DetectedObject(
                    result.getTimestampSeconds(), target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
                tracer.traceDebug(instanceName, "DetectedObj=" + bestDetectedObj);
            }
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method returns the detected AprilTag object.
     *
     * @param aprilTagIds specifies the set of AprilTag IDs to look for, null if looking for any AprilTag.
     * @return detected AprilTag object.
     */
    public DetectedObject getDetectedAprilTag(int... aprilTagIds)
    {
        DetectedObject detectedAprilTag = null;
        double startTime = TrcTimer.getCurrentTime();
        List<PhotonPipelineResult> results = getAllUnreadResults();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (!results.isEmpty())
        {
            for (int i = results.size() - 1; detectedAprilTag == null && i >= 0; i--)
            {
                PhotonPipelineResult result = results.get(i);
                if (result.hasTargets())
                {
                    List<PhotonTrackedTarget> targets = result.getTargets();
                    double timestamp = result.getTimestampSeconds();

                    tracer.traceDebug(
                        instanceName, "[%d]: timestamp=%.6f, numTargets=%d", i, timestamp, targets.size());
                    for (PhotonTrackedTarget target: targets)
                    {
                        // Return the detected AprilTag with matching ID or the first one if no ID is provided.
                        if (aprilTagIds == null || matchAprilTagId(target.getFiducialId(), aprilTagIds) != -1)
                        {
                            detectedAprilTag = new DetectedObject(
                                timestamp, target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
                            tracer.traceDebug(instanceName, "DetectedAprilTag=" + detectedAprilTag);
                            break;
                        }
                    }
                }
            }
        }

        return detectedAprilTag;
    }   //getDetectedAprilTag

    /**
     * This method finds a matching AprilTag ID in the specified array and returns the found index.
     *
     * @param id specifies the AprilTag ID to be matched.
     * @param aprilTagIds specifies the AprilTag ID array to find the given ID.
     * @return index in the array that matched the ID, -1 if not found.
     */
    private int matchAprilTagId(int id, int[] aprilTagIds)
    {
        int matchedIndex = -1;

        for (int i = 0; i < aprilTagIds.length; i++)
        {
            if (id == aprilTagIds[i])
            {
                matchedIndex = i;
                break;
            }
        }

        return matchedIndex;
    }   //matchAprilTagId

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @param result specifies the latest pipeline result.
     * @param robotToCamera specifies the Transform3d position of the camera from the robot center.
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getRobotEstimatedPose(PhotonPipelineResult result, Transform3d robotToCamera)
    {
        TrcPose2D robotPose = null;
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        if (bestTarget != null)
        {
            Optional<Pose3d> aprilTagPoseOptional = fieldLayout.getTagPose(bestTarget.fiducialId);
            Pose3d aprilTagPose3d = aprilTagPoseOptional.isPresent()? aprilTagPoseOptional.get(): null;

            if (aprilTagPose3d != null)
            {
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestTarget.getBestCameraToTarget(), aprilTagPose3d, robotToCamera.inverse());
                Transform2d fieldToRobot2d = new Transform2d(
                    robotPose3d.getTranslation().toTranslation2d(), robotPose3d.getRotation().toRotation2d());
                robotPose = new TrcPose2D(
                    Units.metersToInches(-fieldToRobot2d.getY()),
                    Units.metersToInches(fieldToRobot2d.getX()),
                    -fieldToRobot2d.getRotation().getDegrees());
                tracer.traceDebug(instanceName, "EstimatedRobotPose[%d]=%s", bestTarget.fiducialId, robotPose);
            }
            else
            {
                tracer.traceDebug(
                    instanceName, "EstimatedRobotPose: failed to get AprilTagPose[%d]", bestTarget.fiducialId);
            }
        }

        return robotPose;
    }   //getRobotEstimatedPose

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @param robotToCamera specifies the Transform3d position of the camera from the robot center.
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getRobotEstimatedPose(Transform3d robotToCamera)
    {
        TrcPose2D estimatedPose = null;
        double startTime = TrcTimer.getCurrentTime();
        List<PhotonPipelineResult> results = getAllUnreadResults();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (!results.isEmpty())
        {
            PhotonPipelineResult result = results.get(results.size() - 1);
            estimatedPose = getRobotEstimatedPose(result, robotToCamera);
        }

        return estimatedPose;
    }   //getRobotEstimatedPose

    /**
     * This method calculates the robot's field position by subtracting the AprilTag's field position by the AprilTag
     * position from the camera and the camera position on the robot.
     *
     * @param aprilTagFieldPose specifies the AprilTag's field position.
     * @param aprilTagTargetPose specifies the AprilTag's position from the camera.
     * @param robotToCamera specifies the camera's position on the robot.
     * @return robot's field position.
     */
    public TrcPose2D getRobotPoseFromAprilTagFieldPose(
        Pose3d aprilTagFieldPose3d, Transform3d cameraToTarget, Transform3d robotToCamera)
    {
        Pose2d robotPose2d =
            aprilTagFieldPose3d.transformBy(cameraToTarget.inverse()).transformBy(robotToCamera.inverse()).toPose2d();

        return new TrcPose2D(
            Units.metersToInches(-robotPose2d.getY()), Units.metersToInches(robotPose2d.getX()),
            -robotPose2d.getRotation().getDegrees());
    }   //getRobotPoseFromAprilTagFieldPose

    /**
     * This method calculates the target pose with an offset from the given AprilTag pose.
     *
     * @param aprilTagFieldPose3d specifies the AprilTag 3D field pose.
     * @param xOffset specifies the x-offset from AprilTag in inches.
     * @param yOffset specifies the y-offset from AprilTag in inches.
     * @param angleOffset specifies the angle offset in degrees.
     * @return calculated target pose.
     */
    public TrcPose2D getTargetPoseOffsetFromAprilTag(
        Pose3d aprilTagFieldPose3d, double xOffset, double yOffset, double angleOffset)
    {
        // TODO: Need to be debugged.
        Transform2d offset = new Transform2d(
            new Translation2d(Units.inchesToMeters(yOffset), Units.inchesToMeters(-xOffset)),
            new Rotation2d(Units.degreesToRadians(-angleOffset)));
        Pose2d targetPose2d = aprilTagFieldPose3d.toPose2d().transformBy(offset);
        return new TrcPose2D(
            Units.metersToInches(-targetPose2d.getY()), Units.metersToInches(targetPose2d.getX()),
            180.0 - targetPose2d.getRotation().getDegrees());
    }   //getTargetPoseOffsetFromAprilTag

}   //class FrcPhotonVision
