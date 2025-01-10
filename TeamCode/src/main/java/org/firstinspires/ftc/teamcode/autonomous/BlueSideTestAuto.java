package org.firstinspires.ftc.teamcode.autonomous;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public class Lift {
        private DcMotorEx liftRight;
        private DcMotorEx liftLeft;

        public Lift(HardwareMap hardwareMap) {
            liftRight = hardwareMap.get(DcMotorEx.class, "liftMotor_right");
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
            liftLeft = hardwareMap.get(DcMotorEx.class, "liftMotor_left");
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftRight.setPower(0.8);
                    liftLeft.setPower(0.8);
                    initialized = true;
                }

                double pos = liftLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftRight.setPower(-0.8);
                    liftLeft.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftRight.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class Intake {
        private CRServo intake;
        private Servo intakePivot;
        private DcMotorEx intakeArm;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
            intakePivot = hardwareMap.get(Servo.class, "intake_pivot");
            intakeArm = hardwareMap.get(DcMotorEx.class, "intake_arm");
            intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class IntakeSample implements Action {
            private boolean initialized = false;
            private double ticks = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ticks = 0.0;
                    intakePivot.setPosition(0.55);
                    intake.setPower(1.0);
                    initialized = true;
                }
                if (intakePivot.getPosition() != 0.55) {
                    return true;
                } else {
                    ticks += 1;
                }
                packet.put("liftPos", ticks);
                if (ticks < 1000) {
                    return true;
                } else {
                    intakePivot.setPosition(0.0);
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeSample() {
            return new IntakeSample();
        }

        public class DepositSample implements Action {
            private boolean initialized = false;
            private boolean deposit = false;
            private double ticks = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ticks = 0.0;
                    intakePivot.setPosition(0.0);
                    intake.setPower(0.0);
                    initialized = true;
                }
                if (intakePivot.getPosition() != 0.0) {
                    return true;
                } else {
                    ticks += 1;
                    if (!deposit) {
                        intake.setPower(-1.0);
                        deposit = true;
                    }
                }
                packet.put("liftPos", ticks);
                if (ticks < 1000) {
                    return true;
                } else {
                    intakePivot.setPosition(0.0);
                    intake.setPower(0);
                    return false;
                }
            }
        }

        public Action depositSample() {
            return new DepositSample();
        }
    }

    @Override
    public void runOpMode() {

        initAprilTag();

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
//        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
//        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    private double[] localize() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double[] x = new double[currentDetections.size()];
        double[] y = new double[currentDetections.size()];
        double[] r = new double[currentDetections.size()];
        double[] position = new double[3];
        int i = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                x[i] = detection.robotPose.getPosition().x;
                y[i] = detection.robotPose.getPosition().y;
                r[i] = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                i += 1;
            }   // end for() loop
        }
        position[0] = average(x);
        position[1] = average(y);
        position[2] = average(r);
        return position;
    }
    public static double average(double[] array) {
        double sum = 0;
        for (double value : array) {
            sum += value;
        }
        return sum/array.length;
    }
}