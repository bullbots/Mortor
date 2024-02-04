/*----------------------------------------------------------------------------*/ /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;
import java.util.function.DoubleSupplier;

public class JoystickDrive  extends CommandBase {

    private DrivetrainFalcon m_drivetrain; 
    private DoubleSupplier joyY;
    private DoubleSupplier joyX;
    private DoubleSupplier joyZ;

    public JoystickDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY , DoubleSupplier joyX , DoubleSupplier joyZ) { 
        this.joyY = joyY;
        this.joyX = joyX;
        this.joyZ = joyZ;
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override 
    public void initialize() {}

    @Override 
    public void execute() {
        double _joyY = Math.signum(joyY.getAsDouble())*Math.pow(joyY.getAsDouble(),2) * m_drivetrain.getIsFullSpeed();
        double _joyX = Math.signum(joyX.getAsDouble())*Math.pow(joyX.getAsDouble(),2) * m_drivetrain.getIsFullSpeed();
        boolean turnInPlace = true;
        m_drivetrain.curvatureDrive(_joyY, _joyX, turnInPlace);
//        SmartDashboard.putNumber("JoyX", joyX.asDouble)
//        SmartDashboard.putNumber("JoyX", _joyX)
//        SmartDashboard.putNumber("JoyY", _joyY)
    }

    @Override 
    public void end(boolean interrupted) {
        m_drivetrain.set(0.0, 0.0);
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
