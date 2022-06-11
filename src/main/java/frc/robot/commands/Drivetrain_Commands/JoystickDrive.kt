/*----------------------------------------------------------------------------*/ /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.Drivetrain_Commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainFalcon
import java.util.function.DoubleSupplier
import kotlin.math.pow
import kotlin.math.sign

class JoystickDrive (
    private val m_drivetrain: DrivetrainFalcon,
    private val joyY: DoubleSupplier,
    private val joyX: DoubleSupplier,
    private val joyZ: DoubleSupplier = DoubleSupplier { 1.0 }
) : CommandBase() {
    
    init { addRequirements(m_drivetrain) }

    override fun initialize() {}

    override fun execute() {
        var _joyY = squareFullSpeed(joyY.asDouble)
        var _joyX = squareFullSpeed(joyX.asDouble)
        m_drivetrain.curvatureDrive(_joyY, _joyX, isQuickTurn=true)
    }

    fun squareFullSpeed(input: Double): Double {
        return sign(input)*input.pow(2) * m_drivetrain.isFullSpeed
    }

    override fun end(interrupted: Boolean) {
        m_drivetrain.set(0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }
}
