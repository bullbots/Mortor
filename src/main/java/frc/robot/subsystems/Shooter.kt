package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SafeTalonFX

/**
 * The shooterSpinner spins from a static velocity taking in
 * inputs from the SmartDashboard to change its values
 */
class Shooter : SubsystemBase() {
    // var shooterSpinner: SafeSparkMax
    var shooterSpinner: SafeTalonFX
    var servo: Servo

    init {
        configureShuffleBoard()

        // shooterSpinner = SafeSparkMax(Constants.SHOOTER_PORT)
        shooterSpinner = SafeTalonFX(Constants.SHOOTER_PORT, isDrivetrain=false, usePID=true)
        servo = Servo(0)

        configurePID()

        // shooterSpinner.idleMode = CANSparkMax.IdleMode.kCoast
        shooterSpinner.setNeutralMode(NeutralMode.Coast)
    }

    private fun configurePID() {
//        shooterSpinner.pidController.ff = Constants.SHOOTER_FF
//        shooterSpinner.pidController.p = Constants.SHOOTER_P
//        shooterSpinner.pidController.i = Constants.SHOOTER_I
//        shooterSpinner.pidController.d = Constants.SHOOTER_D

        shooterSpinner.config_kF(Constants.kSlotIdx, Constants.SHOOTER_KFF, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kP(Constants.kSlotIdx, Constants.SHOOTER_KP, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kI(Constants.kSlotIdx, Constants.SHOOTER_KI, Constants.kTIMEOUT_MS)
        shooterSpinner.config_kD(Constants.kSlotIdx, Constants.SHOOTER_KD, Constants.kTIMEOUT_MS)
    }

    private fun configureShuffleBoard() {}

    override fun periodic() {
//        SmartDashboard.putNumber("Shooter speed", shooterSpinner.selectedSensorVelocity)
    }

    fun stop() { shooterSpinner.stopMotor() }
}