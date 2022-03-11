package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkMax
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeSparkMax
import frc.robot.util.SafeTalonFX

/**
 * The shooterSpinner spins from a static velocity taking in
 * inputs from the SmartDashboard to change its values
 */
class Shooter : SubsystemBase() {
    // var shooterSpinner: SafeSparkMax
    var shooterSpinner: SafeTalonFX

    init {
        configureShuffleBoard()

        // shooterSpinner = SafeSparkMax(Constants.SHOOTER_PORT)
        shooterSpinner = SafeTalonFX(Constants.SHOOTER_PORT, isDrivetrain=false, usePID=true)

        configurePID()

        // shooterSpinner.idleMode = CANSparkMax.IdleMode.kCoast
        shooterSpinner.setNeutralMode(NeutralMode.Coast)
    }

    private fun configurePID() {
//        shooterSpinner.pidController.ff = Constants.SHOOTER_FF
//        shooterSpinner.pidController.p = Constants.SHOOTER_P
//        shooterSpinner.pidController.i = Constants.SHOOTER_I
//        shooterSpinner.pidController.d = Constants.SHOOTER_D

        shooterSpinner.config_kF(Constants.kSlotIdx, Constants.SHOOTER_FF, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kP(Constants.kSlotIdx, Constants.SHOOTER_P, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kI(Constants.kSlotIdx, Constants.SHOOTER_I, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kD(Constants.kSlotIdx, Constants.SHOOTER_D, Constants.kTIMEOUT_MS)
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {
//        SmartDashboard.putNumber("Shooter speed", shooterSpinner.selectedSensorVelocity)
    }

    fun stop() { shooterSpinner.stopMotor() }
}