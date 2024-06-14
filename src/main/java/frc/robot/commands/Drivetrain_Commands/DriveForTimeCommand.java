package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainFalcon;

class DriveForTimeCommand extends WaitCommand {

    private DrivetrainFalcon drivetrainFalcon;

    public DriveForTimeCommand(DrivetrainFalcon drivetrainFalcon, double time) {
        super(time);
        this.drivetrainFalcon = drivetrainFalcon;
        addRequirements(drivetrainFalcon);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: DriveForTimeCommand.initialize");
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INFO: DriveForTimeCommand.end");
        super.end(interrupted);
    }
}
