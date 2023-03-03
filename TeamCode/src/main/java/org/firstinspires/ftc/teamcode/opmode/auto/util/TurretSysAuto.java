package org.firstinspires.ftc.teamcode.opmode.auto.util;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.pipelines.JunctionWithArea;
import org.firstinspires.ftc.teamcode.vision.util.TurretPIDF;
import org.opencv.core.Rect;

import java.util.function.BooleanSupplier;

@Config
public final class TurretSysAuto extends TurretSys {
    public static double pix_to_degree = 0.096;

    public TurretSysAuto(ServoEx turret, AnalogInput turretEnc) {
        super(turret, turretEnc);
    }

    @Override
    public void updateTarget(Rect rect) {
        double junctionX = rect.x + (double) rect.width / 2;
        target = (turretPosition + ((junctionX - 320) * pix_to_degree));
    }
}
