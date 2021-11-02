package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;


 @TeleOp
public class Teleop_BlueSide extends LinearOpMode {
     //webcam variables
     OpenCvCamera webcam;
     static RotatedRect rotatedRectFitToContour;
     static RotatedRect Target;
     static Mat hierarchy = new Mat();
     static ArrayList<MatOfPoint> filteredContours = new ArrayList<>();
     static List<MatOfPoint> contoursList = new ArrayList<>();
     static ArrayList<RotatedRect> targetList = new ArrayList<RotatedRect>();
     static ArrayList<RotatedRect> targetList2 = new ArrayList<RotatedRect>();
     static ArrayList<RotatedRect> targetList3 = new ArrayList<RotatedRect>();
     static double minDistance;
     static double targetDistance;

     //drivetrain variables
     double rotateP = 0.0211111111;
     double rotateOut;
     double rotateMinSpeed = 0.15;
     double robotAngle;
     double rotateError;
     double slowMult = 0.4;

     double p = 150;
     double i = 0;
     double d = 0;
     double f = 14;

     Vector2d finalDrivePower = new Vector2d(0, 0);

     //autoaim variables
     ElapsedTime timer = new ElapsedTime();
     double aimHeading;
     double aimHeadingIncrement = 5;
     double shootAngleOffset = 17.5;

     //Back Right 16.5631336
     //Left Side 11.6898036
     //Old Working 19.17
     //Center 13.6644878
     //Average 15.2718563

     //Shooting Position 19.3831486

     double targetSpeed;
     double shooterSpeed;
     double manualSpeed = 1620;
     double range = 0.7;
     double pixelsToDegrees;
     double pixelConversionA = 0.0000144;
     double pixelConversionB = -0.00269;
     double pixelConversionC = 0.203;
     double camRange = 0.7;
     double updatedHeading;
     Pose2d poseEstimate;

     double camCenterM = -0.49233;
     double camCenterB = 544.013;
     double newCamCenter;
     double odomCamCenter;

     //distance calculation variables
     double distanceA = 0.002689;
     double distanceB = -2.161;
     double distanceC = 509.75;
     double camDistance;
     double camMaxHeight;
     double camMinHeight;
     double camMaxWidth;
     double camMinWidth;

     //power calculation variables
     double distance;
     double a = 0.05733;
     double b = -11.6302;
     double c = 2115.37;
     double powerMax = 1750;

     //powershot variables
     boolean isAligned = false;
     double powerShotPower = 1475;
     ElapsedTime alignmentTimer = new ElapsedTime();

     //wobble variables
     boolean firstTimeWobble = true;
     int wobblePickUp = -60;
     int wobbleDump = -395;
     int wobbleRaise = -790;
     double servoOpen = 1;
     double servoClose = 0.65;

     public enum LEDState {
         START,
         RING,
         POWERSHOTS,
         AIMING,
         SHOOTING,
         MAX_POWER
     }

     public enum HeadingState {
         DRIVER_CONTROL,
         AUTO_AIM
     }

     public enum DriveState {
         SLOW_MODE,
         FAST_MODE
     }

     public enum ShootState {
         START,
         SHOOT_ODOMETRY,
         SHOOT_CAMERA,
         POWERSHOTS,
         SERVO,
         SPINUP
     }

     public enum PowershotState {
         GOTO,
         ALIGN1,
         ALIGN2,
         ALIGN3
     }

     public enum WobbleState {
         WOBBLE_UP,
         WOBBLE_DOWN,
         PICKUP,
         RAISE,
         DUMP,
         CLOSE_SERVO,
         RESET
     }

     public enum ManualState {
         FULL,
         ODOMETRY,
         MANUAL
     }

     @Override
     public void runOpMode() {
         //create hardware
         final Hardware2021 robot = new Hardware2021();

         //access camera and create camera object using opencv
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "shooterCam"), cameraMonitorViewId);

         //set targeting camera pipeline to backboard detection
         webcam.setPipeline(new TargetDetectionPipeline());

         //start camera stream
         webcam.openCameraDevice();
         webcam.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);


         //set framerate
         telemetry.setMsTransmissionInterval(15);

         //init hardware and set servo pos
         robot.init(hardwareMap);
         robot.pusher.setPosition(0.08);
         robot.wobbleServo.setPosition(servoClose);
         robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);


         //init odometry
         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
         drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         //obtain pose estimate from auto
         drive.setPoseEstimate(poseStorage.currentPose);


         //set finite state machine enums to initial values
         HeadingState headingState = HeadingState.DRIVER_CONTROL;
         DriveState driveState = DriveState.SLOW_MODE;
         ShootState shootState = ShootState.START;
         WobbleState wobbleState = WobbleState.WOBBLE_UP;
         LEDState ledState = LEDState.START;
         ManualState manualState = ManualState.FULL;

         //set launcher PID
         robot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
         robot.shooter2.setVelocityPIDFCoefficients(p, i, d, f);

         //set wobble motor tolerance
         robot.wobbleMotor.setTargetPositionTolerance(10);

         //alert drivers that program is ready
         telemetry.addData("Say", "Hello Driver");    //
         telemetry.update();

         waitForStart();

         if (isStopRequested()) return;

         while (!isStopRequested() && opModeIsActive()) {
             telemetry.addData("Manual State:", manualState);

             //manual power tuning
/*
            if(gamepad1.dpad_up) {
                targetSpeed += 10;
                sleep(200);
            }
            if(gamepad1.dpad_down) {
                targetSpeed -= 10;
                sleep(200);
            }
            if(gamepad1.dpad_right) {
                robot.shooter.setVelocity(targetSpeed);
                robot.shooter2.setVelocity(targetSpeed);
            }
            if(gamepad1.dpad_left) {
                robot.shooter.setVelocity(0);
                robot.shooter2.setVelocity(0);
            }
*/
             if(Target != null) {
                 camDistance = (distanceA * (Target.center.x * Target.center.x)) + distanceB * Target.center.x + distanceC;
                 pixelsToDegrees = pixelConversionA*(camDistance*camDistance) + pixelConversionB*camDistance + pixelConversionC;
             }


             //distance equation tuning
             if (gamepad1.dpad_up) {
                 camCenterM += 0.01;
                 sleep(100);
             }

             if (gamepad1.dpad_down) {
                 camCenterM -= 0.01;
                 sleep(100);
             }

             if (gamepad1.dpad_right) {
                 camCenterB -= 3;
                 sleep(100);
             }

             if (gamepad1.dpad_left) {
                 camCenterB += 3;
                 sleep(100);
             }

             if (gamepad1.a) {
                 c -= 5;
                 sleep(100);
             }

             if (gamepad1.y) {
                 c += 5;
                 sleep(100);
             }

             if (gamepad2.a) {
                 robot.pusher.setPosition(0.164);
                 sleep(115);
                 robot.pusher.setPosition(0.08);
                 sleep(115);
             }

             //create pose estimate based on localization
             poseEstimate = drive.getPoseEstimate();
             //get heading value and convert to 0-180 scale
             robotAngle = Math.toDegrees(poseEstimate.getHeading());
             if (robotAngle > 180) {
                 robotAngle -= 360;
             }

             //intake control
             if (gamepad2.right_bumper) {
                 robot.intake.setPower(1);
             } else if (gamepad2.left_bumper) {
                 robot.intake.setPower(-1);
             } else {
                 robot.intake.setPower(0);
             }

             switch(ledState) {
                 case START:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                     telemetry.addData("LED Color", RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                     if(robot.ringSensor.getDistance(DistanceUnit.MM) <= 70 && Math.abs(robot.intake.getPower()) > 0)
                     {
                         ledState = LEDState.RING;
                     }

                     if(shootState == ShootState.SHOOT_ODOMETRY || shootState == shootState.SHOOT_CAMERA)
                     {
                         ledState = LEDState.AIMING;
                     }

                     if(shootState == ShootState.POWERSHOTS)
                     {
                         ledState = LEDState.POWERSHOTS;
                     }

                     if(targetSpeed >= powerMax && headingState != HeadingState.DRIVER_CONTROL)
                     {
                         ledState = LEDState.MAX_POWER;
                     }
                     break;
                 case RING:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                     telemetry.addData("LED Color", RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                     if(robot.ringSensor.getDistance(DistanceUnit.MM) >= 70 || robot.intake.getPower() == 0)
                     {
                         ledState = LEDState.START;
                     }
                     break;
                 case AIMING:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                     telemetry.addData("LED Color", RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                     if(shootState == ShootState.SERVO)
                     {
                         ledState = LEDState.SHOOTING;
                     }
                     else if(shootState != ShootState.SHOOT_CAMERA && shootState != ShootState.SHOOT_ODOMETRY)
                     {
                         ledState = LEDState.START;
                     }
                     break;
                 case SHOOTING:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                     telemetry.addData("LED Color", RevBlinkinLedDriver.BlinkinPattern.GREEN);
                     if(shootState != ShootState.SERVO)
                     {
                         ledState = LEDState.START;
                     }
                     break;
                 case POWERSHOTS:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                     telemetry.addData("LED Color", RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                     if(shootState != ShootState.POWERSHOTS)
                     {
                         ledState = LEDState.START;
                     }
                     break;
                 case MAX_POWER:
                     robot.ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                     if(targetSpeed < powerMax || headingState == HeadingState.DRIVER_CONTROL)
                     {
                         ledState = LEDState.START;
                     }
                     break;
             }

             switch (manualState) {
                 case FULL:
                     if (gamepad2.dpad_right) {
                         manualState = ManualState.ODOMETRY;
                         sleep(200);
                     }
                     break;
                 case ODOMETRY:
                     if (gamepad2.dpad_right) {
                         manualState = ManualState.MANUAL;
                         sleep(200);
                     }
                     break;
                 case MANUAL:
                     if (gamepad2.dpad_right) {
                         manualState = ManualState.FULL;
                         sleep(200);
                     }
                     break;
             }

             distance = Math.sqrt(((74 - (poseEstimate.getX() - 2.75)) * (74 - (poseEstimate.getX() - 2.75))) + ((-36 - (poseEstimate.getY() - 5.5)) * (-36 - (poseEstimate.getY() - 5.5))));
             odomCamCenter = camCenterM * (distance) + camCenterB;

             //finite state machine to swap heading control of robot from driver to auto aim at target
             switch (headingState) {
                 case DRIVER_CONTROL:
                     if (shootState == ShootState.START) {
                         rotateOut = -gamepad1.right_stick_x;
                         robot.shooter.setVelocity(0);
                         robot.shooter2.setVelocity(0);
                         if (gamepad1.right_bumper) {
                             headingState = HeadingState.AUTO_AIM;
                         }
                     }
                     else if (shootState == ShootState.POWERSHOTS)
                     {
                         rotateOut = -gamepad1.right_stick_x;
                     }

                     break;
                 case AUTO_AIM:
                     switch (manualState) {
                         case FULL:
                         case ODOMETRY:
                             rotateOut = threshold(rotateOut, 1, -1);
                             if (shootState != ShootState.SHOOT_CAMERA && shootState != ShootState.SERVO && shootState != ShootState.SHOOT_ODOMETRY) {
                                 aimHeading = Math.toDegrees(Math.atan2((36 - (poseEstimate.getY() - 5.5)), (74 - poseEstimate.getX() - 2.75))) + shootAngleOffset;
                                 distance = Math.sqrt(((74 - (poseEstimate.getX() - 2.75)) * (74 - (poseEstimate.getX() - 2.75))) + ((36 - (poseEstimate.getY() - 5.5)) * (36 - (poseEstimate.getY() - 5.5))));
                                 robot.shooter.setVelocity(targetSpeed);
                                 robot.shooter2.setVelocity(targetSpeed);
                                 targetSpeed = a * (distance * distance) + b * (distance) + c;
                             }
                             rotateError = ((((aimHeading - robotAngle + 180) % 360) + 360) % 360) - 180;
                             if (Math.abs(rotateError) > range) {
                                 rotateOut = rotateError * rotateP;
                                 if (Math.abs(rotateOut) <= rotateMinSpeed) {
                                     rotateOut = (rotateMinSpeed * rotateOut) / Math.abs(rotateOut);
                                 }
                             } else {
                                 rotateOut = 0;
                             }
                             break;
                         case MANUAL:
                             rotateOut = -gamepad1.right_stick_x;
                             robot.shooter.setVelocity(1650);
                             robot.shooter2.setVelocity(1650);
                             break;
                     }
                     if (gamepad1.left_bumper) {
                         headingState = HeadingState.DRIVER_CONTROL;
                         robot.shooter.setVelocity(0);
                         robot.shooter2.setVelocity(0);
                     }
                     break;
                 default:
                     headingState = HeadingState.DRIVER_CONTROL;
             }


             //finite state machine to control drive train
             switch (driveState) {
                 case SLOW_MODE:
                     if (shootState == ShootState.START || shootState == ShootState.POWERSHOTS) {
                         //get directional input of left stick and rotate by robot heading
                         finalDrivePower = new Vector2d(
                                 -gamepad1.left_stick_x * slowMult,
                                 gamepad1.left_stick_y * slowMult
                         ).rotated(-poseEstimate.getHeading());
                         if (headingState == HeadingState.DRIVER_CONTROL) {
                             rotateOut *= 0.5;
                         }
                         if (gamepad1.right_trigger > 0.5) {
                             driveState = DriveState.FAST_MODE;
                         }
                     }
                     break;
                 case FAST_MODE:
                     //get directional input of left stick and rotate by robot heading
                     if (shootState == ShootState.START || shootState == ShootState.POWERSHOTS) {
                         finalDrivePower = new Vector2d(
                                 -gamepad1.left_stick_x,
                                 gamepad1.left_stick_y
                         ).rotated(-poseEstimate.getHeading());
                         if (gamepad1.right_trigger < 0.5) {
                             driveState = DriveState.SLOW_MODE;
                         }
                     }
                     break;
             }


             //finite state machine that controls aiming & shooting
             switch (shootState) {
                 case START:
                     if (gamepad2.right_trigger > 0.5 && headingState == HeadingState.AUTO_AIM) {
                         switch (manualState) {
                             case FULL:
                             case ODOMETRY:
                                 shootState = ShootState.SHOOT_ODOMETRY;
                                 timer.reset();
                                 break;
                             case MANUAL:
                                 shootState = ShootState.SERVO;
                                 break;
                         }
                     } else if (gamepad2.right_trigger > 0.5) {
                         shootState = ShootState.SPINUP;
                     }


                     if (gamepad2.y) {
                         shootState = ShootState.POWERSHOTS;
                     }

                     if (robot.pusher.getPosition() == 0.164) {
                         robot.pusher.setPosition(0.08);
                     }
                     break;
                 case SHOOT_ODOMETRY:
                     if (gamepad2.right_trigger < 0.5) {
                         shootState = ShootState.START;
                     }
                     finalDrivePower = new Vector2d(0, 0);
                     shooterSpeed = robot.shooter.getVelocity();
                     distance = Math.sqrt(((74 - (poseEstimate.getX() - 2.75)) * (74 - (poseEstimate.getX() - 2.75))) + ((36 - (poseEstimate.getY() - 5.5)) * (36 - (poseEstimate.getY() - 5.5))));
                     targetSpeed = a * (distance * distance) + b * (distance) + c;
                     robot.shooter.setVelocity(targetSpeed);
                     robot.shooter2.setVelocity(targetSpeed);
                     aimHeading = Math.toDegrees(Math.atan2((36 - (poseEstimate.getY() - 5.5)), (74 - poseEstimate.getX() - 2.75))) + shootAngleOffset;
                     if (Target != null) {
                         camDistance = (distanceA * (Target.center.x * Target.center.x)) + distanceB * Target.center.x + distanceC;
                         newCamCenter = camCenterM * (camDistance) + camCenterB;
                     }
                     if (Math.abs(rotateError) < range || timer.milliseconds() > 1000) {
                         switch (manualState) {
                             case FULL:
                                 if (Math.abs(newCamCenter - Target.center.y) > 9) {
                                     shootState = ShootState.SHOOT_CAMERA;
                                 } else {
                                     camDistance = (distanceA * (Target.center.x * Target.center.x)) + distanceB * Target.center.x + distanceC;
                                     targetSpeed = a * (camDistance * camDistance) + b * (camDistance) + c;
                                     robot.shooter.setVelocity(targetSpeed);
                                     robot.shooter2.setVelocity(targetSpeed);
                                     shooterSpeed = robot.shooter.getVelocity();
                                     if (shooterSpeed > (targetSpeed - 21) && shooterSpeed < (targetSpeed + 21)) {
                                         camMaxHeight = 0;
                                         camMaxWidth = 0;
                                         camMinHeight = 0;
                                         camMinWidth = 0;
                                         shootState = ShootState.SERVO;
                                     }
                                 }
                                 break;
                             case ODOMETRY:
                             case MANUAL:
                                 shootState = ShootState.SERVO;
                                 break;
                         }
                     }
                     break;
                 case SHOOT_CAMERA:
                     if (gamepad2.right_trigger < 0.5) {
                         shootState = ShootState.START;
                     }
                     finalDrivePower = new Vector2d(0, 0);
                     camDistance = (distanceA * (Target.center.x * Target.center.x)) + distanceB * Target.center.x + distanceC;
                     newCamCenter = camCenterM * (camDistance) + camCenterB;
                     aimHeadingIncrement = (newCamCenter - Target.center.y) * pixelsToDegrees;
                     aimHeading = robotAngle + aimHeadingIncrement;
                     targetSpeed = a * (camDistance * camDistance) + b * (camDistance) + c;
                     robot.shooter.setVelocity(targetSpeed);
                     robot.shooter2.setVelocity(targetSpeed);
                     shooterSpeed = robot.shooter.getVelocity();
                     if (Math.abs(aimHeadingIncrement) < camRange) {
                         updatedHeading = Math.atan2((36 - (poseEstimate.getY() - 5.5)), (74 - poseEstimate.getX() - 2.75)) + Math.toRadians(shootAngleOffset) + Math.toRadians(aimHeadingIncrement);
                         rotateOut = 0;
                         drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), updatedHeading));
                         aimHeading = Math.toDegrees(poseEstimate.getHeading());
                         shooterSpeed = robot.shooter.getVelocity();
                         if (shooterSpeed > (targetSpeed - 21) && shooterSpeed < (targetSpeed + 21)) {
                             camMaxHeight = 0;
                             camMaxWidth = 0;
                             camMinHeight = 0;
                             camMinWidth = 0;
                             shootState = ShootState.SERVO;
                         }
                     }
                     break;
                 case POWERSHOTS:
                     robot.shooter.setVelocity(powerShotPower);
                     robot.shooter2.setVelocity(powerShotPower);
                     break;
                 case SERVO:
                     switch (manualState) {
                         case FULL:
                         case ODOMETRY:
                             rotateOut = 0;
                             targetSpeed = threshold(targetSpeed, powerMax, 0);
                             robot.shooter.setVelocity(targetSpeed);
                             robot.shooter2.setVelocity(targetSpeed);
                             robot.pusher.setPosition(0.164);
                             sleep(115);
                             robot.pusher.setPosition(0.08);
                             sleep(115);
                             break;
                         case MANUAL:
                             robot.pusher.setPosition(0.164);
                             sleep(115);
                             robot.pusher.setPosition(0.08);
                             sleep(115);
                             break;
                     }
                     break;
                 case SPINUP:
                     headingState = HeadingState.AUTO_AIM;
                     switch (manualState) {
                         case FULL:
                         case ODOMETRY:
                             shootState = ShootState.SHOOT_ODOMETRY;
                             timer.reset();
                             break;
                         case MANUAL:
                             shootState = ShootState.SERVO;
                             break;
                     }
                     break;
             }

             if (gamepad2.b) {
                 drive.cancelFollowing();
                 shootState = ShootState.START;
                 wobbleState = WobbleState.WOBBLE_UP;
                 firstTimeWobble = true;
             }

             if (gamepad2.right_trigger < 0.5 && shootState != ShootState.POWERSHOTS) {
                 shootState = ShootState.START;
             }

             switch (wobbleState) {
                 case WOBBLE_UP:
                     if (gamepad2.x && gamepad2.left_trigger > 0.5) {
                         robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                         robot.wobbleMotor.setTargetPosition(wobbleDump);
                         timer.reset();
                         wobbleState = WobbleState.DUMP;
                     } else if (gamepad2.x && firstTimeWobble) {
                         robot.wobbleMotor.setPower(0.5);
                         wobbleState = WobbleState.RESET;
                     } else if (gamepad2.x) {
                         robot.wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                         robot.wobbleMotor.setTargetPosition(wobblePickUp);
                         wobbleState = WobbleState.PICKUP;
                     }
                     break;
                 case WOBBLE_DOWN:
                     if (gamepad2.x) {
                         robot.wobbleServo.setPosition(servoClose);
                         timer.reset();
                         wobbleState = WobbleState.CLOSE_SERVO;
                     }
                     break;
                 case PICKUP:
                     if (Math.abs(robot.wobbleMotor.getCurrentPosition() - wobblePickUp) < 10) {
                         robot.wobbleServo.setPosition(servoOpen);
                         wobbleState = WobbleState.WOBBLE_DOWN;
                     }
                     break;
                 case RAISE:
                     if (Math.abs(robot.wobbleMotor.getCurrentPosition() - wobbleRaise) < 10) {
                         wobbleState = WobbleState.WOBBLE_UP;
                     }
                     break;
                 case DUMP:
                     if (Math.abs(robot.wobbleMotor.getCurrentPosition() - wobbleDump) < 10) {
                         robot.wobbleServo.setPosition(servoOpen);
                         sleep(500);
                         robot.wobbleMotor.setTargetPosition(wobbleRaise);
                         robot.wobbleServo.setPosition(servoClose);
                         wobbleState = WobbleState.WOBBLE_UP;
                     }
                     break;
                 case CLOSE_SERVO:
                     if (timer.milliseconds() >= 700) {
                         robot.wobbleMotor.setTargetPosition(wobbleRaise);
                         wobbleState = WobbleState.RAISE;
                     }
                     break;
                 case RESET:
                     sleep(150);
                     if (Math.abs(robot.wobbleMotor.getVelocity()) <= 100) {
                         robot.wobbleMotor.setPower(0);
                         sleep(100);
                         robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                         robot.wobbleMotor.setTargetPosition(wobblePickUp);
                         robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                         robot.wobbleMotor.setPower(0.5);
                         robot.wobbleServo.setPosition(servoOpen);
                         firstTimeWobble = false;
                         wobbleState = WobbleState.WOBBLE_DOWN;
                     }
                     break;
             }

             if (targetList2.size() > 0) {
                 targetList3.clear();
                 targetList3.addAll(targetList2);
                 for (int i = 0; i < targetList3.size(); i++) {
                     if (targetList3.get(i) != null) {
                         targetDistance = Math.abs(targetList3.get(i).center.y - odomCamCenter);
                         if (i == 0) {
                             minDistance = targetDistance;
                             Target = targetList3.get(i);
                         } else {
                             minDistance = Math.min(minDistance, targetDistance);
                             if (minDistance == targetDistance) {
                                 Target = targetList3.get(i);
                             }
                         }
                     }
                 }
             }


             //set drive power to final values
             drive.setWeightedDrivePower(
                     new Pose2d(
                             finalDrivePower.getX(),
                             finalDrivePower.getY(),
                             rotateOut
                     )
             );

             if (gamepad2.back) {
                 drive.setPoseEstimate(new Pose2d(-64.5, 60.7, Math.toRadians(180)));
                 isAligned = true;
                 alignmentTimer.reset();
             }

             if (alignmentTimer.seconds() >= 30 && isAligned) {
                 isAligned = false;
             }
             drive.update();

             telemetry.addData("SHOT POWER OFFSET: ", c);
             telemetry.addData("LEFT TO RIGHT OFFSET: ", camCenterB);
             if (Target != null) {
                 camDistance = (distanceA * (Target.center.x * Target.center.x)) + distanceB * Target.center.x + distanceC;
                 if ((camMaxHeight - camMinHeight) > (camMaxWidth - camMinWidth)) {
                     telemetry.addData("Target Quality", camMaxHeight - camMinHeight);
                 }
                 if ((camMaxHeight - camMinHeight) < (camMaxWidth - camMinWidth)) {
                     telemetry.addData("Target Quality", camMaxWidth - camMinWidth);
                 }
                 telemetry.addData("Target Width", Target.size.width);
                 telemetry.addData("Target Center", Target.center.y);
                 telemetry.addData("Target Height", Target.center.x);
                 telemetry.addData("Camera Distance", camDistance);
                 telemetry.addData("Heading Increment", aimHeadingIncrement);
                 telemetry.addData("new heading", Math.toDegrees(updatedHeading));
                 telemetry.addData("proportion", Target.size.height / Target.size.width);
             }
             telemetry.addData("RingSensor", robot.ringSensor.getDistance(DistanceUnit.MM));
             telemetry.addData("LED State", ledState);
             telemetry.addData("CamCenterM", camCenterM);
             telemetry.addData("CamCenterB", camCenterB);
             telemetry.addData("Min Speed", rotateMinSpeed);
             telemetry.addData("X", poseEstimate.getX());
             telemetry.addData("Y", poseEstimate.getY());
             telemetry.addData("Heading", robotAngle);
             telemetry.addData("Servo Pos", robot.pusher.getPosition());
             telemetry.addData("LauncherPower", targetSpeed);
             telemetry.addData("Distance", distance);
             telemetry.addData("MotorSpeed", robot.shooter.getVelocity());
             telemetry.addData("Target Angle", aimHeading);
             telemetry.addData("Wobble Encoder", robot.wobbleMotor.getCurrentPosition());
             telemetry.addData("Wobble Speed", robot.wobbleMotor.getVelocity());
             telemetry.addData("Drive State", driveState);
             telemetry.addData("Heading State", headingState);
             telemetry.addData("Shoot State", shootState);
             telemetry.addData("Wobble State", wobbleState);
             telemetry.addData("A", a);
             telemetry.addData("B", b);
             telemetry.addData("C", c);
             telemetry.update();
         }

     }

     //create pipeline
     static class TargetDetectionPipeline extends OpenCvPipeline {
         //Image Buffers
         Mat hueMat = new Mat();
         Mat thresholdMat1 = new Mat();
         Mat gammaAdjustedMat = new Mat();
         Mat finalThreshold = new Mat();
         Mat morphedThreshold = new Mat();
         Mat contoursOnPlainImageMat = new Mat();

         //Noise reduction elements
         Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(6, 6));
         Mat dilateElement = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(12, 12));

         //Colors
         static final Scalar BLUE_MIN = new Scalar(100, 145, 90);
         static final Scalar BLUE_MAX = new Scalar(120, 255, 255);
         static final Scalar BLUE = new Scalar(0, 0, 255);
         static final Scalar RED = new Scalar(255, 0, 0);

         static final int CONTOUR_LINE_THICKNESS = 2;

         enum Stage {
             FINAL,
             H,
             MASK,
             MASK_NR,
             CONTOURS
         }

         Stage[] stages = Stage.values();

         //keep track of viewport stage
         int stageNum = 0;

         @Override
         public void onViewportTapped() {
             //change viewport stage
             int nextStageNum = stageNum + 1;
             if (nextStageNum >= stages.length) {
                 nextStageNum = 0;
             }

             stageNum = nextStageNum;
         }

         @Override
         public Mat processFrame(Mat input) {
             targetList2.clear();
             targetList2.addAll(targetList);
             targetList.clear();
             for (MatOfPoint contour : findContours(input)) {
                 analyzeContour(contour, input);
             }

             switch (stages[stageNum]) {
                 case H: {
                     return hueMat;
                 }

                 case FINAL: {
                     return input;
                 }

                 case MASK: {
                     return finalThreshold;
                 }

                 case MASK_NR: {
                     return morphedThreshold;
                 }

                 case CONTOURS: {
                     return contoursOnPlainImageMat;
                 }
             }
             return input;
         }

         ArrayList<MatOfPoint> findContours(Mat input) {
             //Contour storage
             contoursList.clear();

             //Gamma adjust image, convert image to HSV, and Threshold Image, then extract H channel
             Mat lookUpTable = new Mat(1, 256, CvType.CV_8U);
             byte[] lookUpTableData = new byte[(int) (lookUpTable.total() * lookUpTable.channels())];
             for (int i = 0; i < lookUpTable.cols(); i++) {
                 lookUpTableData[i] = saturate(Math.pow(i / 255.0, 1) * 255.0);
             }
             lookUpTable.put(0, 0, lookUpTableData);
             Core.LUT(input, lookUpTable, gammaAdjustedMat);
             Imgproc.cvtColor(gammaAdjustedMat, hueMat, Imgproc.COLOR_RGB2HSV);
             Core.inRange(hueMat, BLUE_MIN, BLUE_MAX, thresholdMat1);
             Core.extractChannel(thresholdMat1, finalThreshold, 0);
             //Reduce noise
             morphMask(finalThreshold, morphedThreshold);

             // Finding the contours along with its heirarchy (so we can find interior/child contours)
             Imgproc.findContours(morphedThreshold, contoursList, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);

             filteredContours.clear();
             for (int i = 0; i < contoursList.size(); i++) {
                 //Filtering out all countours that don't have children.
                 if (hierarchy.get(0, i)[2] != -1) {
                     MatOfPoint childContour = contoursList.get((int) hierarchy.get(0, i)[2]);
                     if (Imgproc.contourArea(childContour) > 10) {
                         filteredContours.add(childContour);
                     }
                 }
             }

             //draw contours on a separate image for staging purposes
             input.copyTo(contoursOnPlainImageMat);
             Imgproc.drawContours(contoursOnPlainImageMat, filteredContours, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

             return filteredContours;
         }

         void morphMask(Mat input, Mat output) {
             //expand the target borders using dilation

             Imgproc.dilate(input, output, dilateElement);
         }

         void analyzeContour(MatOfPoint contour, Mat input) {
             //convert contour to different format
             MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

             //do a rect fit to the contour
             rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
             drawRotatedRect(rotatedRectFitToContour, input);
             targetList.add(rotatedRectFitToContour);

         }


         static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
             //draws the rotated rect
             Point[] points = new Point[4];
             rect.points(points);

             for (int i = 0; i < 4; i++) {
                 Imgproc.line(drawOn, points[i], points[(i + 1) % 4], new Scalar(0, 255, 0), 2);
             }
         }

         private byte saturate(double val) {
             int iVal = (int) Math.round(val);
             iVal = iVal > 255 ? 255 : (Math.max(iVal, 0));
             return (byte) iVal;
         }
     }

         static double threshold(double value, double max, double min) {
             if (value > max)
                 value = max;
             else if (value < min)
                 value = min;
             return value;
         }
 }
