package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SafeSparkMax;
import kotlin.jvm.internal.Intrinsics;
import org.jetbrains.annotations.NotNull;


public final class Intake extends SubsystemBase {

    private SafeSparkMax intakeSpinner;
    private SafeSparkMax armSpinner;
    private SafeSparkMax raiseLowerSpinner;
    private int loopIdx;

    public Intake() {

        configureShuffleBoard();

        intakeSpinner = new SafeSparkMax(Constants.INTAKE_SPINNER_PORT, CANSparkLowLevel.MotorType.kBrushless);
        armSpinner = new SafeSparkMax(Constants.INTAKE_ARM_SPINNER_PORT, CANSparkLowLevel.MotorType.kBrushless);
        raiseLowerSpinner = new SafeSparkMax(Constants.RAISE_LOWER_ARM_PORT, CANSparkLowLevel.MotorType.kBrushless);

        var pidController = raiseLowerSpinner.getPIDController();
        pidController.setP(Constants.INTAKE_P);
        pidController.setI(Constants.INTAKE_I);
        pidController.setD(Constants.INTAKE_D);
        pidController.setFF(Constants.INTAKE_FF);
        pidController.setIZone(Constants.INTAKE_IZONE);
        pidController.setOutputRange(-1.0, 1.0);

        pidController.setSmartMotionMaxVelocity(Constants.I_MAXRPM, Constants.I_SlotIdx);
        pidController.setSmartMotionMinOutputVelocity(0.0, Constants.I_SlotIdx);
        pidController.setSmartMotionMaxAccel(Constants.I_MAXRPM, Constants.I_SlotIdx);
        pidController.setSmartMotionAllowedClosedLoopError(Constants.I_ALLOWED_ERROR, Constants.I_SlotIdx);

        intakeSpinner.setIdleMode(CANSparkBase.IdleMode.kBrake);
        armSpinner.setIdleMode(CANSparkBase.IdleMode.kBrake);
        raiseLowerSpinner.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    
    public SafeSparkMax getIntakeSpinner() {
        return intakeSpinner;
    }

    public void setIntakeSpinner(@NotNull SafeSparkMax var1) {
        Intrinsics.checkNotNullParameter(var1, "<set-?>");
        this.intakeSpinner = var1;
    }

    @NotNull
    public SafeSparkMax getArmSpinner() {
        return this.armSpinner;
    }

    public void setArmSpinner(@NotNull SafeSparkMax var1) {
        Intrinsics.checkNotNullParameter(var1, "<set-?>");
        this.armSpinner = var1;
    }

    @NotNull
    public SafeSparkMax getRaiseLowerSpinner() {
        return raiseLowerSpinner;
    }

    public final void setRaiseLowerSpinner(@NotNull SafeSparkMax var1) {
        Intrinsics.checkNotNullParameter(var1, "<set-?>");
        this.raiseLowerSpinner = var1;
    }

    public int getLoopIdx() {
        return this.loopIdx;
    }

    public void setLoopIdx(int var1) {
        this.loopIdx = var1;
    }

    private void configureShuffleBoard() {
    }

    @Override
    public void periodic() {
        //        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//            SmartDashboard.putNumber("Arm Encoder", raiseLowerSpinner.encoder.position)
//            SmartDashboard.putNumber("Arm Current", raiseLowerSpinner.outputCurrent)
//            SmartDashboard.putNumber("Arm Speed", raiseLowerSpinner.appliedOutput)
//            println("INFO: Arm Encoder Pos: ${raiseLowerSpinner.encoder.position}")
//        }
    }

    public void stopArm() {
        raiseLowerSpinner.set(0.0);
    }

    public void stop() {
        intakeSpinner.set(0.0);
        armSpinner.set(0.0);
        raiseLowerSpinner.set(0.0);
    }
}
