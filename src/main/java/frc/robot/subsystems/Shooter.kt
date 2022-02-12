package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax

/**
 * The shooterSpinner spins from a static velocity taking in
 * inputs from the SmartDashboard to change its values
 */
class Shooter : SubsystemBase() {
    var shooterSpinner: SafeSparkMax

    init {
        configureShuffleBoard()

        shooterSpinner = SafeSparkMax(Constants.SHOOTER_PORT)

        configurePID()

        shooterSpinner.idleMode = CANSparkMax.IdleMode.kCoast

        var inst = NetworkTableInstance.getDefault()

    }

    private fun configurePID() {
        shooterSpinner.pidController.ff = Constants.SHOOTER_FF
        shooterSpinner.pidController.p = Constants.SHOOTER_P
        shooterSpinner.pidController.i = Constants.SHOOTER_I
        shooterSpinner.pidController.d = Constants.SHOOTER_D
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() { shooterSpinner.stopMotor() }
}