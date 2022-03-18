/*----------------------------------------------------------------------------*/ /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainFalcon
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sign

class JoystickDrive (
    var m_drivetrain: DrivetrainFalcon,
    private var joyY: DoubleSupplier,
    private var joyX: DoubleSupplier,
    private var joyZ: DoubleSupplier = DoubleSupplier { 1.0 }
) : CommandBase() {
    
    init { addRequirements(m_drivetrain) }

    override fun initialize() {}

    override fun execute() {
        var _joyY = sign(joyY.asDouble)*joyY.asDouble.pow(2) * m_drivetrain.isFullSpeed
        var _joyX = sign(joyX.asDouble)* joyX.asDouble.pow(2) * m_drivetrain.isFullSpeed
        val turnInPlace = true
        m_drivetrain.curvatureDrive(_joyY, _joyX, turnInPlace)
//        SmartDashboard.putNumber("JoyX", joyX.asDouble)
        SmartDashboard.putNumber("JoyX", _joyX)
        SmartDashboard.putNumber("JoyY", _joyY)
    }

    override fun end(interrupted: Boolean) {
        m_drivetrain.set(0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }
}