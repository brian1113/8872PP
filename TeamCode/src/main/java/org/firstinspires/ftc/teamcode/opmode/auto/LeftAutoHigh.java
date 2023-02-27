package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.group.DownSequence;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.util.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class LeftAutoHigh extends BaseOpMode {

    //dr4b heights for conestack
    //firstCone is the dr4b height setpoint of the topmost cone
    public static int firstCone = -176;
    public static int secondCone = -151;
    public static int thirdCone = -134;
    public static int fourthCone = -109;
    public static int fifthCone = 0;

    @Override
    public void initialize() {
        super.initialize();

        //setting up pipeline for camera
        turret.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //set initial pose estimate
        rrDrive.setPoseEstimate(new Pose2d(-36, -62, Math.toRadians(90)));

        //TODO: make it so the preload trajectory doesn't crash into the medium junction half the time (i think its partly cuz my roadrunner tuning is bad)
        //TODO: make the claw start in the correct position (open)
        //TODO: optimize it to make it faster, and maybe give camera more time
        //TODO: stress test for consistency (right now its not very consistent, but it would be if you increased camera time and took the 1+4)
        schedule(
                new SequentialCommandGroup(
                        //grab and lift when the auto starts


                        //drive to the medium junction while doing mediumSequence
                        new ParallelCommandGroup(
                                claw.grab().alongWith(new DelayedCommand(arm.goTo(ArmSys.Pose.GRAB),100)),
                                new DelayedCommand(new FollowLeftPreloadTrajectory(rrDrive), 150),
                                //this command lets me set it to a specific angle instead of one of the setpositions
                                new DelayedCommand(new HighSequenceWithAngle(lift, turret, arm, 0.78028), 150)
                        ),

                        //give the camera half a second to align (isn't enough, should increase for more consistency)
                        new ParallelCommandGroup(
                                new DelayedCommand(new AlignToPoleWithCamera(turret, 1.25,277),0),
                                //releases after 0.65 seconds, the command group continues after 1
                                new DelayedCommand(claw.release(), 850)
                        ),
                        //cycle cones
                        new CycleOneConeLeftHigh(rrDrive, lift, turret, arm, claw, firstCone),
                        new CycleOneConeLeftHigh(rrDrive, lift, turret, arm, claw, secondCone),
                        new CycleOneConeLeftHigh(rrDrive, lift, turret, arm, claw, thirdCone),
                        new CycleOneConeLeftHigh(rrDrive, lift, turret, arm, claw, fourthCone),
                        new CycleOneConeLeftHigh(rrDrive, lift, turret, arm, claw, fifthCone),

                        //reset the lift after everything finishes
                        new DownSequence(lift, turret, arm, claw)

                )
        );
    }
}
