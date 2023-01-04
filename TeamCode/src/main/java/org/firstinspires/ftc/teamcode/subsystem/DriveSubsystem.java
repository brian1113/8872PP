package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.util.AngleController;
import org.firstinspires.ftc.teamcode.util.ScuffedMecanumDrive;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

import java.util.Arrays;
import java.util.Collections;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final ScuffedMecanumDrive drive;
    private final RevIMU imu;
    public static double kP = 0.06;
    public static double kI = 0;
    public static double kD = 0.0035;
    private final AngleController controller = new AngleController(kP, kI, kD, 0);
    private double output;
    public static boolean transformed = true;

    public static int joystickTransformFactor = 30;

    public static double slowFactor = 3.5;

    public static double strafeRateLimit = 2;
    public static double forwardRateLimit = 2.5;
    public static double turnRateLimit = 2;

    private final SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(strafeRateLimit);
    private final SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(forwardRateLimit);
    private final SlewRateLimiter turnRateLimiter = new SlewRateLimiter(turnRateLimit);

    private double target;

    public DriveSubsystem(MotorEx fL, MotorEx fR, MotorEx bL, MotorEx bR, RevIMU imu){
        this.imu = imu;
        drive = new ScuffedMecanumDrive(fL, fR, bL, bR);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed){
        if(transformed){
            strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
            forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
            turnSpeed = turnRateLimiter.calculate(turnSpeed);
        }
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
    }

    public void driveRobotCentricSlowMode(double strafeSpeed, double forwardSpeed, double turnSpeed){
        drive.driveRobotCentric(strafeSpeed / slowFactor, forwardSpeed / slowFactor, turnSpeed / slowFactor);
    }

    public void driveWithJunctionRotation(double strafeSpeed, double forwardSpeed, double turnSpeed){
        double[] motorPowers = drive.getMotorPowers(strafeSpeed, forwardSpeed, turnSpeed);
        output = controller.calculate(imu.getHeading());
        double[] finalPowers;

        for(int i = 0; i < motorPowers.length; i++)
            motorPowers[i] -= output;

        double max = Math.abs(Arrays.stream(motorPowers).max().getAsDouble());
        if(max>1){
            for(int i = 0; i < motorPowers.length; i++)
                motorPowers[i]/=max;
        }
        finalPowers = motorPowers.clone();

        drive.driveWithMotorPowers(finalPowers[0], finalPowers[1], finalPowers[2], finalPowers[3]);
    }


    public void setHeading(double degrees){
        Log.d("asd", ""+degrees);
        controller.setSetPoint(degrees);
        target = degrees;
    }

    public void updatePID(){
        output = controller.calculate(imu.getHeading());
        drive.driveWithMotorPowers(-output, -output, -output, -output);
    }

    public double getOutput(){
        return output;
    }

    public double getTarget() {
        return target;
    }

    // desmos: https://www.desmos.com/calculator/j2e6yaorld
    public double joystickTransform(double input){
        return (1.0 / (joystickTransformFactor - 1))
                * Math.signum(input)
                * (Math.pow(joystickTransformFactor, Math.abs(input))-1);
    }



}
