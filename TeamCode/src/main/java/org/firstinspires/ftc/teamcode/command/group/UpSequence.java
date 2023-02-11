package org.firstinspires.ftc.teamcode.command.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.subsystem.ArmSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.subsystem.TurretSys;

public final class UpSequence extends SequentialCommandGroup {

    public UpSequence(LiftSys lift, TurretSys turret, ArmSys arm, Height height, TurretSys.Pose pose) {
        addCommands(
                new ParallelCommandGroup(
                        lift.goTo(height),
                        arm.goTo(ArmSys.Pose.DEPOSIT)
                ),
                new ParallelCommandGroup(
                        turret.goTo(pose),
                        arm.goTo(ArmSys.Pose.DOWN)
                )
        );

        addRequirements(lift, turret, arm);
    }
}
