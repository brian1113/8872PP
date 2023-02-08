package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.util.ProfiledServoSubsystem;

@Config
public final class TurretSubsystem extends ProfiledServoSubsystem {
    public static double maxVelocity = 100;
    public static double maxAcceleration = 100;
    // right forward: 0.88
    // left forward*: 0
    // right back: 0.57
    // left back: 0.29
    // start position: 0.435
    // left: 0.15
    // right: 0.715

    public enum Position {
        RIGHT_FORWARD(0.88),
        LEFT_FORWARD(0),
        RIGHT_BACK(0.57),
        LEFT_BACK(0.29),
        LEFT(0.15),
        RIGHT(0.715),
        ZERO(0.435);

        private final double height;
        Position(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public Command goTo(Position position) {
        return goTo(position.getHeight());
    }


    public TurretSubsystem(ServoEx turret) {
        super(turret, maxVelocity, maxAcceleration);
    }
}
