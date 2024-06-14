package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainFalcon;

class DriveForDistanceCommand extends Command {

    private DrivetrainFalcon drivetrainFalcon;
    private double speed;
    private double distance;

    public DriveForDistanceCommand(DrivetrainFalcon drivetrainFalcon, double speed, double distance) {
        this.drivetrainFalcon = drivetrainFalcon;
        this.speed = speed;
        this.distance = distance;
        addRequirements(drivetrainFalcon);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: DriveForDistanceCommand initialize");
        drivetrainFalcon.arcadeDrive(speed, 0.0, false);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        var averageDist = drivetrainFalcon.getAverageDist();
        return averageDist >= distance;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: DriveForDistanceCommand end");
        drivetrainFalcon.stop();
    }
}
