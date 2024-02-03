package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SafeTalonFX;

/**
 * The shooterSpinner spins from a static velocity taking in
 * inputs from the SmartDashboard to change its values
 */
public final class Shooter extends SubsystemBase {
    // var shooterSpinner: SafeSparkMax
    private SafeTalonFX shooterSpinner;
    private Servo servo;
    private int loopIdx = 0;

    public Shooter() {
        configureShuffleBoard();

        // shooterSpinner = SafeSparkMax(Constants.SHOOTER_PORT)
        shooterSpinner = SafeTalonFX(Constants.SHOOTER_PORT, false, true);
        servo = new Servo(0);

        configurePID();

        // shooterSpinner.idleMode = CANSparkMax.IdleMode.kCoast
        shooterSpinner.setNeutralMode(NeutralMode.Coast);
    }

    private void configurePID() {
//        shooterSpinner.pidController.ff = Constants.SHOOTER_FF
//        shooterSpinner.pidController.p = Constants.SHOOTER_P
//        shooterSpinner.pidController.i = Constants.SHOOTER_I
//        shooterSpinner.pidController.d = Constants.SHOOTER_D

        shooterSpinner.config_kF(Constants.kSlotIdx, Constants.SHOOTER_KFF, Constants.kTIMEOUT_MS);
        shooterSpinner.config_kP(Constants.kSlotIdx, Constants.SHOOTER_KP, Constants.kTIMEOUT_MS);
        shooterSpinner.config_kI(Constants.kSlotIdx, Constants.SHOOTER_KI, Constants.kTIMEOUT_MS);
        shooterSpinner.config_kD(Constants.kSlotIdx, Constants.SHOOTER_KD, Constants.kTIMEOUT_MS);
    }


    private void configureShuffleBoard() {}

    @Override 
    public void periodic() {
//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//            SmartDashboard.putNumber("Shooter speed", shooterSpinner.selectedSensorVelocity)
//            SmartDashboard.putNumber("Shooter Stator Current", shooterSpinner.statorCurrent)
//            SmartDashboard.putNumber("Shooter Supply Current", shooterSpinner.supplyCurrent)
//            SmartDashboard.putNumber("Shooter Output Percent", shooterSpinner.motorOutputPercent)
//            SmartDashboard.putNumber("Shooter Sensor Velocity", shooterSpinner.selectedSensorVelocity)
//        }
    }

    public void stop() { shooterSpinner.stopMotor(); }
}
