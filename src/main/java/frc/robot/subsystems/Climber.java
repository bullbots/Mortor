package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SafeTalonFX;

public class Climber extends SubsystemBase {

    private int loopIdx = 0;
    private SafeTalonFX climberMotor;
//    var hallEffectTop: Counter
//    var hallEffectBot: Counter
    private Counter limitSwitch;

    private double delta = 0.0;

    public enum State {
        BOTTOM,
        RELEASE,
        TOP
    }

    private State currentState = State.RELEASE;

    public static boolean isReleased = false;

    public Climber() {
//        configureShuffleBoard()

        // Initializing Motor(s)
        climberMotor = new SafeTalonFX(Constants.CLIMBER_PORT, false, false);

        climberMotor.setNeutralMode(NeutralMode.Brake);

        climberMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        climberMotor.config_kF(Constants.kSlotIdx, Constants.CLIMBER_KFF, Constants.kTIMEOUT_MS);
        climberMotor.config_kP(Constants.kSlotIdx, Constants.CLIMBER_KP, Constants.kTIMEOUT_MS);
        climberMotor.config_kI(Constants.kSlotIdx, Constants.CLIMBER_KI, Constants.kTIMEOUT_MS);
        climberMotor.config_kD(Constants.kSlotIdx, Constants.CLIMBER_KD, Constants.kTIMEOUT_MS);

        climberMotor.configMotionCruiseVelocity(21000.0, Constants.kTIMEOUT_MS);
        climberMotor.configMotionAcceleration(21000.0, Constants.kTIMEOUT_MS);

        limitSwitch = new Counter(Counter.Mode.kPulseLength);

        limitSwitch.setUpSource(0);
    }

    public void stop() {
        climberMotor.set(0.0);
    }

//    void checkHallEffectSoftLimits() {
//                // Used for the HallEffect Sensors
//        if(currentState == State.RELEASE) {
//            if (hallEffectTop.get() > 0) {
//                currentState = State.TOP
//                delta = climberMotor.selectedSensorPosition
//            } else if (hallEffectBot.get() > 0) {
//                currentState = State.BOTTOM
//                delta = climberMotor.selectedSensorPosition
//            }
//        } else if (currentState == State.BOTTOM) {
//            if(climberMotor.selectedSensorPosition - delta > Constants.CLIMBER_LIMIT_THRESHOLD) {
//                currentState = State.RELEASE
//                hallEffectBot.reset()
//            }
//        } else if (currentState == State.TOP) {
//            if (delta - climberMotor.selectedSensorPosition > Constants.CLIMBER_LIMIT_THRESHOLD) {
//                currentState = State.RELEASE
//                hallEffectTop.reset()
//            }
//        }
//    }

    public void setAuto(TalonFXControlMode controlMode, double encoderVal) {
        if (limitSwitch.get() > 0) {
            if (encoderVal > 0) {
                climberMotor.set(controlMode, encoderVal);
                limitSwitch.reset();
            } else {
                System.out.println("WARNING: THE CLIMBER IS TOO LOW!!!!!!");
                climberMotor.stopMotor();
            }
        } else {
            climberMotor.set(controlMode, encoderVal);
        }
    }

    public void setManual(double percentOutput) {
        if (limitSwitch.get() > 0) {
            if(percentOutput < 0) {
                climberMotor.stopMotor();
                System.out.println("WARNING: THE CLIMBER IS TOO LOW!!!!!!");
            } else {
                climberMotor.set(percentOutput);
                limitSwitch.reset();
            }
        } else {
            climberMotor.set(percentOutput);
        }
    }

    public void resetEncoders() {
        climberMotor.setSelectedSensorPosition(0.0);
    }

    public double getEncoderPos() {
        return climberMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx);
    }

//        override fun periodic() {
//        loopIdx++
//        if (loopIdx == 10) {
//            loopIdx = 0
//            SmartDashboard.putNumber("Climber PID Error", climberMotor.getClosedLoopError(Constants.kPIDLoopIdx))
//            SmartDashboard.putNumber("Climber Velocity", climberMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx))
//            SmartDashboard.putNumber("Climber Position", climberMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx))
//            SmartDashboard.putNumber("Climber Supply Current", climberMotor.supplyCurrent)
//            SmartDashboard.putNumber("Climber Stator Current", climberMotor.statorCurrent)
//            SmartDashboard.putNumber("Limit Switch", limitSwitch.get().toDouble())
//            SmartDashboard.putNumber("Climber Active Traj Pos", climberMotor.activeTrajectoryPosition)
//        }
//    }
}