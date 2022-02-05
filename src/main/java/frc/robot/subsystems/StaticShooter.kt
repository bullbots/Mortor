package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax
import frc.robot.util.SafeTalonFX

class StaticShooter : SubsystemBase() {
    private var shooter_spinner: SafeSparkMax

    init {
        configureShuffleBoard()

        shooter_spinner = SafeSparkMax(Constants.SHOOTER_PORT)

        configurePID()

        shooter_spinner.idleMode = CANSparkMax.IdleMode.kCoast


        var inst = NetworkTableInstance.getDefault()
    }

    private fun configurePID() {
        shooter_spinner.pidController.ff = Constants.SHOOTER_FF
        shooter_spinner.pidController.p = Constants.SHOOTER_P
        shooter_spinner.pidController.i = Constants.SHOOTER_I
        shooter_spinner.pidController.d = Constants.SHOOTER_D

    }

    /**
     * This sets the speed of the static Shooter
     */
    fun set(shootVel: Double) {
        shooter_spinner.set(shootVel)
    }

    fun setShooter() { set(0.5) }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() {
        shooter_spinner.stopMotor()
    }
}