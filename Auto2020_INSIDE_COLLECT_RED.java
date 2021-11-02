package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.accelConstraint;


@Autonomous(name= "Red Inside")
public class Auto2020_INSIDE_COLLECT_RED extends LinearOpMode {
    //camera variables
    OpenCvInternalCamera2 phoneCam;  // Create phone webcam object
    static RotatedRect Ring;
    static RotatedRect rotatedRectFitToContour;
    int ringState = 3;

    //shooting variables
    double shootAngleOffset = 22;
    double shootAngleOffsetRadians = Math.toRadians(shootAngleOffset);

    //wobble variables
    int wobblePickUp = 800;
    int wobbleRaise = 50;

    TrajectoryVelocityConstraint slowMode = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(3),
                new MecanumVelocityConstraint(7, TRACK_WIDTH)
        ));





    @Override
    public void runOpMode() {
        final Hardware2021 robot = new Hardware2021();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-64.1732, -24.0, 0.0));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new RingDetectionPipeline());

        phoneCam.openCameraDevice();
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        robot.init(hardwareMap);
        robot.pusher.setPosition(0.08);
        robot.wobbleServo.setPosition(0.65);
        robot.wobbleMotor.setTargetPositionTolerance(10);
        robot.wobbleMotor.setTargetPosition(0);
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.wobbleMotor.setPower(0.5);
        robot.shooter.setVelocityPIDFCoefficients(150,0,0.3,14);
        robot.shooter2.setVelocityPIDFCoefficients(150,0,0.3,14);

        telemetry.setMsTransmissionInterval(15);

        telemetry.addData("Say", "Hello Driver");

        drive.update();

        //universal first shot

        Trajectory initialShot = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-50.0,-24,Math.atan2((-36 + 24), (74 + 50.0))+shootAngleOffsetRadians+Math.toRadians(1)))
                .build();

        //4 ring trajectories
        Trajectory collect3 = drive.trajectoryBuilder(initialShot.end())
                .splineToSplineHeading(new Pose2d(-31, -36, 0.0), 0.0)
                .forward(10, slowMode, accelConstraint)
                .build();

        Trajectory collect3_2 = drive.trajectoryBuilder(collect3.end())
                .splineToSplineHeading(new Pose2d(-29, -36, Math.atan2(0, 74+24)+shootAngleOffsetRadians+Math.toRadians(2)), Math.toRadians(180.0))
                .build();

        Trajectory collectLastAndShoot = drive.trajectoryBuilder(collect3_2.end())
                .splineToSplineHeading(new Pose2d(-6.0,-39.0, Math.atan2((0),(74+6))+shootAngleOffsetRadians+Math.toRadians(3)), 0.0)
                .build();

        Trajectory dumpWobble4 = drive.trajectoryBuilder(collectLastAndShoot.end())
                .splineToSplineHeading(new Pose2d(54.0,-48.0, Math.toRadians(-45.0)), Math.toRadians(-25.0))
                .addSpatialMarker(new Vector2d(36.0,-24.0), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.wobbleMotor.setTargetPosition(wobblePickUp);
                    }
                })
                .build();

        Trajectory park4 = drive.trajectoryBuilder(dumpWobble4.end())
                .addDisplacementMarker(5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.wobbleMotor.setTargetPosition(wobbleRaise);
                    }
                })
                .splineToConstantHeading(new Vector2d(12.0,-12.0), 0)
                .build();


        //1 ring trajectories
        Trajectory collect1AndShoot = drive.trajectoryBuilder(initialShot.end())
                .splineToConstantHeading(new Vector2d(-34.0, -36.0), Math.toRadians(0.0))
                .splineToSplineHeading(new Pose2d(-24.0,-36.0, 0), 0)
                .splineToSplineHeading(new Pose2d(-6.0,-36.0, Math.atan2((0), (74+6))+shootAngleOffsetRadians+Math.toRadians(3)), 0.0)
                .build();

        Trajectory dumpWobble1 = drive.trajectoryBuilder(collect1AndShoot.end())
                .splineToSplineHeading(new Pose2d(20.0, -30.0, 0.0), 0.0)
                .addSpatialMarker(new Vector2d(6.0,-36.0), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.wobbleMotor.setTargetPosition(wobblePickUp);
                    }
                })
                .build();

        Trajectory park1 = drive.trajectoryBuilder(dumpWobble1.end(), true)
                .addDisplacementMarker(5, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.wobbleMotor.setTargetPosition(wobbleRaise);
                    }
                })
                .back(5)
                .splineToConstantHeading(new Vector2d(12.0,-12.0), Math.toRadians(180))
                .build();


        //0 ring trajectories

        Trajectory dumpWobble0 = drive.trajectoryBuilder(initialShot.end())
                .splineToSplineHeading(new Pose2d(20.0,-44.0, Math.toRadians(-95.0)), Math.toRadians(0.0))
                .addSpatialMarker(new Vector2d(0.0,-50.0), new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        robot.wobbleMotor.setTargetPosition(wobblePickUp);
                    }
                })
                .build();

        Trajectory park0 = drive.trajectoryBuilder(dumpWobble0.end())
                .splineToLinearHeading(new Pose2d(12.0, -12.0, 0.0), 0.0)
                .build();

        Ring = null;

        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        robot.shooter.setVelocity(1620);
        robot.shooter2.setVelocity(1620);
        sleep(200);
        if(Ring != null) {
            if (Ring.center.x < 85) {
                Ring = null;
            }
        }

        if (Ring == null) {
            ringState = 0;
        }
        else {
            if(Ring.size.height < 25 || Ring.size.width < 25)
            {
                ringState = 1;
            }
            else {
                ringState = 4;
            }
        }
        if(Ring != null) {
            telemetry.addData("Ring X", Ring.center.x);
            telemetry.addData("Ring Y", Ring.center.y);
            telemetry.addData("Ring height", Ring.size.height);
            telemetry.addData("Ring Width", Ring.size.width);

        }
        telemetry.addData("case", ringState);
        telemetry.update();

        switch(ringState)
        {
            case 0:
                //sleep(8000);
                drive.followTrajectory(initialShot);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1620 - 21) && robot.shooter.getVelocity()>(1620 + 21))
                {
                    robot.shooter.setVelocity(1620);
                    robot.shooter2.setVelocity(1620);
                }
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
                drive.followTrajectory(dumpWobble0);
                while(robot.wobbleMotor.isBusy() && opModeIsActive())
                {
                }
                robot.wobbleServo.setPosition(1);
                sleep(200);
                robot.wobbleMotor.setTargetPosition(wobbleRaise);
                drive.followTrajectory(park0);
                robot.wobbleServo.setPosition(0.6);
                poseStorage.currentPose = drive.getPoseEstimate();
                while(robot.wobbleMotor.isBusy() && opModeIsActive())
                {
                }
                break;
            case 1:
                robot.shooter.setVelocity(1620);
                robot.shooter2.setVelocity(1620);
                drive.followTrajectory(initialShot);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1620 - 21) && robot.shooter.getVelocity()>(1620 + 21))
                {
                    robot.shooter.setVelocity(1620);
                    robot.shooter2.setVelocity(1620);
                }
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.shooter.setVelocity(1565);
                robot.shooter2.setVelocity(1565);
                robot.intake.setPower(1);
                drive.followTrajectory(collect1AndShoot);
                sleep(350);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1565 - 21) && robot.shooter.getVelocity()>(1565 + 21))
                {
                    robot.shooter.setVelocity(1565);
                    robot.shooter2.setVelocity(1565);
                }
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
                drive.followTrajectory(dumpWobble1);
                while(robot.wobbleMotor.isBusy())
                {
                }
                robot.wobbleServo.setPosition(1);
                drive.followTrajectory(park1);
                robot.wobbleServo.setPosition(0.6);
                poseStorage.currentPose = drive.getPoseEstimate();
                robot.intake.setPower(0);
                while(robot.wobbleMotor.isBusy() && opModeIsActive())
                {
                }
                break;
            case 3:
                telemetry.addData("Say", "I shouldn't be here");
                telemetry.update();
                break;
            case 4:
                drive.followTrajectory(initialShot);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1620 - 21) && robot.shooter.getVelocity()>(1620 + 21))
                {
                    robot.shooter.setVelocity(1620);
                    robot.shooter2.setVelocity(1620);
                }
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.intake.setPower(1);
                robot.shooter.setVelocity(1535);
                robot.shooter2.setVelocity(1535);
                drive.followTrajectory(collect3);
                drive.followTrajectory(collect3_2);
                robot.intake.setPower(1);
                sleep(700);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1535 - 21) && robot.shooter.getVelocity()>(1535 + 21))
                {
                    robot.shooter.setVelocity(1535);
                    robot.shooter2.setVelocity(1535);
                }
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.shooter.setVelocity(1565);
                robot.shooter2.setVelocity(1565);
                drive.followTrajectory(collectLastAndShoot);
                while(opModeIsActive() && robot.shooter.getVelocity()<(1565 - 21) && robot.shooter.getVelocity()>(1565 + 21))
                {
                    robot.shooter.setVelocity(1565);
                    robot.shooter2.setVelocity(1565);
                }
                sleep(700);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                sleep(150);
                robot.pusher.setPosition(0.164);
                sleep(150);
                robot.pusher.setPosition(0.08);
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
                drive.followTrajectory(dumpWobble4);
                while(robot.wobbleMotor.isBusy() && opModeIsActive())
                {
                }
                robot.wobbleServo.setPosition(1);
                drive.followTrajectory(park4);
                robot.wobbleServo.setPosition(0.6);
                poseStorage.currentPose = drive.getPoseEstimate();
                robot.intake.setPower(0);
                while(robot.wobbleMotor.isBusy() && opModeIsActive())
                {
                }
                break;
        }

    }

    static class RingDetectionPipeline extends OpenCvPipeline
    {
        //Image Buffers
        Mat hueMat = new Mat();
        Mat thresholdMat2 = new Mat();
        Mat finalThreshold = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        //Threshold values

        //Noise reduction elements
        Mat dilateElement  = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(2,2));

        //Colors
        static final Scalar RING_MIN = new Scalar(0, 150, 70);
        static final Scalar RING_MAX = new Scalar(25, 255, 255);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;

        enum Stage
        {
            FINAL,
            Cb,
            MASK,
            MASK_NR,
            CONTOURS;
        }

        Stage[] stages = Stage.values();

        //keep track of viewport stage
        int stageNum = 0;

        @Override
        public void onViewportTapped()
        {
            //change viewport stage
            int nextStageNum = stageNum + 1;
            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            switch (stages[stageNum])
            {
                case Cb:
                {
                    return hueMat;
                }

                case FINAL:
                {
                    return input;
                }

                case MASK:
                {
                    return finalThreshold;
                }

                case MASK_NR:
                {
                    return morphedThreshold;
                }

                case CONTOURS:
                {
                    return contoursOnPlainImageMat;
                }
            }
            return input;
        }
        ArrayList<MatOfPoint> findContours(Mat input)
        {
            //Contour storage
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            //Gamma adjust image, convert image to HSV, and Threshold Image, then extract H channel
            Mat lookUpTable = new Mat(1, 256, CvType.CV_8U);
            byte[] lookUpTableData = new byte[(int) (lookUpTable.total()*lookUpTable.channels())];
            for (int i = 0; i < lookUpTable.cols(); i++) {
                lookUpTableData[i] = saturate(Math.pow(i / 255.0, 0.8) * 255.0);
            }
            lookUpTable.put(0, 0, lookUpTableData);
            Core.LUT(input, lookUpTable, input);
            Imgproc.cvtColor(input, hueMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hueMat, RING_MIN, RING_MAX, thresholdMat2);
            Core.extractChannel(thresholdMat2, finalThreshold, 0);
            //Reduce noise
            morphMask(finalThreshold, morphedThreshold);

            //Find contours
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            //draw contours on a separate image for staging purposes
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);


            return contoursList;
        }
        private byte saturate(double val)
        {
            int iVal = (int) Math.round(val);
            iVal = iVal > 255 ? 255 : (iVal < 0 ? 0 : iVal);
            return (byte) iVal;
        }

        void morphMask(Mat input, Mat output)
        {
            //noise reduction through erosion and dilation
            Imgproc.dilate(input, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input)
        {
            //convert contour to different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            //do a rect fit to the contour
            rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            if (rotatedRectFitToContour.size.area() >= 200 && rotatedRectFitToContour.center.x > 80 && rotatedRectFitToContour.center.x < 150 && rotatedRectFitToContour.center.y < 130) {
                drawRotatedRect(rotatedRectFitToContour, input);
                Ring = rotatedRectFitToContour;
            }
        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn)
        {
            //draws the rotated rect

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; i++)
            {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }

    static double threshold(double value, double max, double min) {
        if (value > max) {
            value = max;
        } else if (value < min) {
            value = min;
        } else {
            value = value;
        }
        return value;
    }
}
