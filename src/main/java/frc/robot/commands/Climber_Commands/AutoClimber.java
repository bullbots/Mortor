package frc.robot.commands.Climber_Commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

import java.lang.Math;


public class AutoClimber extends Command {
    private double targetTraj = 0.0;
    private final Climber climber;
    private final boolean isGrenade;
    private final boolean isDown;

    public AutoClimber(Climber climber, boolean isGrenade, boolean isDown) {
        this.climber = climber;
        this.isGrenade = isGrenade;
        this.isDown = isDown;
        if(!isGrenade) {
            if (isDown) {
                targetTraj = Constants.CLIMBER_DOWN;
            } else {
                targetTraj = Constants.CLIMBER_UP;
            }
        } else {
            targetTraj = Constants.CLIMBER_GRENADE;
        }
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (isGrenade) {
            Climber.isReleased = true;
            climber.resetEncoders();
            System.out.println("INFO: Grenade initialize");
        } else if (!Climber.isReleased) {
            System.out.println("WARNING: PULL GRENADE PIN!!!!!!!!!!");
        }
    }

    @Override
    public void execute() {
        climber.setAuto(TalonFXControlMode.MotionMagic, targetTraj);
    }

    @Override
    public boolean isFinished() {
        var traj_pose_error = targetTraj - climber.getEncoderPos();
        if (isGrenade) {
            return Math.abs(traj_pose_error) < 1000;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: Grenade end");
        climber.stop();
    }
}
