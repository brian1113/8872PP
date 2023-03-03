package org.firstinspires.ftc.teamcode.util;


import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmode.auto.RightAutoMid;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class DetectSleeve extends CommandBase {

    ElapsedTime timer = new ElapsedTime();
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest;

    public DetectSleeve(AprilTagDetectionPipeline aprilTagDetectionPipeline, AprilTagDetection tagOfInterest){
        this.aprilTagDetectionPipeline = aprilTagDetectionPipeline;
        this.tagOfInterest = tagOfInterest;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute(){
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 0 || tag.id == 1 || tag.id == 2) {
                    tagOfInterest = tag;
                    Log.d("sleeve detected", ""+tag.id);
                    break;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds()>1.2;
    }
}
