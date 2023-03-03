package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmode.auto.util.TurretSysAuto;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.firstinspires.ftc.teamcode.vision.util.TurretPIDF;
import org.opencv.videoio.VideoCapture;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightAutoMid extends BaseOpMode {

    ElapsedTime timer = new ElapsedTime();

    protected JunctionWithArea pipeline;
    protected TurretSys turret;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    public boolean finished = false;
    private AprilTagDetection tagOfInterest = null;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //dr4b heights for conestack
    //firstCone is the dr4b height setpoint of the topmost cone
    public static int firstCone = -172;
    public static int secondCone = -150;
    public static int thirdCone = -131;
    public static int fourthCone = -105;
    public static int fifthCone = 0;

    @Override
    public void initialize() {
        initStuff();
        timer.reset();

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //setting up pipeline for camera
        turret.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //set initial pose estimate
        rrDrive.setPoseEstimate(new Pose2d(36, -62, Math.toRadians(90)));


        turretServo.setPosition(0.43);

        schedule(
                new SequentialCommandGroup(
                        //grab and lift when the auto starts

                        new DelayedCommand(turret.goTo(0.43), 0),
                        //drive to the medium junction while doing mediumSequence
                        new ParallelCommandGroup(
                                new DetectSleeve(aprilTagDetectionPipeline, tagOfInterest),
                                new DelayedCommand(claw.grab().andThen(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB), 300)), 0),
                                new DelayedCommand(new FollowPreloadTrajectory(rrDrive), 500),
                                new DelayedCommand(new InstantCommand(() -> camera.setPipeline(pipeline)),2000),
                                //this command lets me set it to a specific angle instead of one of the setpositions
                                new DelayedCommand(new RaisedMediumSequenceWithAngle(lift, turret, arm, 0.81127), 1750)
                        ),

                        //give the camera half a second to align (isn't enough, should increase for more consistency)
                        new ParallelCommandGroup(
                                new DelayedCommand(new AlignToPoleWithCamera(turret, 288),100),
                                //releases after 0.65 seconds, the command group continues after 1
                                new DelayedCommand(claw.release(), 1250)
                        ),
                        //cycle cones
                        new CycleOneCone(rrDrive, lift, turret, arm, claw, firstCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, secondCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, thirdCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, fourthCone),
                        new NoAlignCycle(rrDrive, lift, turret, arm, claw, fifthCone),
                        new InstantCommand(() -> finished = true),

                        //reset the lift after everything finishes
                        new DownSequence(lift, turret, arm, claw)

                )
        );
    }

    @Override
    public void run(){
        super.run();
        if(finished){
            if(tagOfInterest == null){
                Log.d("park", "null");
                return;
            }
            Log.d("park", ""+tagOfInterest.id);
        }
    }

    public void initStuff(){
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        triggerGamepadEx1 = new TriggerGamepadEx(gamepad1, gamepadEx1);
        triggerGamepadEx2 = new TriggerGamepadEx(gamepad2, gamepadEx2);

        initHardware();
        setUpHardwareDevices();

        imu = new RevIMU(hardwareMap);
        imu.init();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        pipeline = new JunctionWithArea();
        pipeline.setKernel(50);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.setPipeline(aprilTagDetectionPipeline);

        turretPIDF = new TurretPIDF();

        drive = new DriveSys(fL, fR, bL, bR, imu);
        lift = new LiftSys(dr4bLeftMotor, dr4bRightMotor, limitSwitch);
        lift.goTo(Height.NONE);

        claw = new ClawSys(clawServo);
        turret = new TurretSysAuto(turretServo, turretEnc);
        arm = new ArmSys(armServo);
        flipper = new FlipperSys(flipperServo);

        rrDrive = new SampleMecanumDrive(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tad("Mode", "Done initializing");
        telemetry.update();
    }
}
