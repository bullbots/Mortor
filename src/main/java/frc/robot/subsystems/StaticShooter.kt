package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

class StaticShooter : SubsystemBase() {
    private var shooter_spinner: SafeTalonFX? = null

    init {
        configureShuffleBoard()

        shooter_spinner = SafeTalonFX(Constants.SHOOTER_PORT, true)

        configurePID()

        shooter_spinner?.setNeutralMode(NeutralMode.Coast)

        var inst = NetworkTableInstance.getDefault()
    }

    private fun configurePID() {
        shooter_spinner?.config_kF(0, Constants.SHOOTER_FF)
        shooter_spinner?.config_kP(0, Constants.SHOOTER_P)
        shooter_spinner?.config_kI(0, Constants.SHOOTER_I)
        shooter_spinner?.config_kD(0, Constants.SHOOTER_D)

    }

    /**
     * This sets the speed of the static Shooter
     */
    fun set(shootVel: Double) {
        shooter_spinner?.set(shootVel)
    }

    fun setShooter() { set(0.5) }

    private fun configureShuffleBoard() {}

    override fun periodic() {}

    fun stop() {
        shooter_spinner?.stopMotor()
    }
}