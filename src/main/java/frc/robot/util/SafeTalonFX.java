package frc.robot.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

import java.lang.Math;


public class SafeTalonFX extends TalonFX {

    // Max RPM 6380 rpm (106.3 rps)
    private double maxSpeed = 106.3;
    private double deadBand = 0.02;
    private boolean usePID;
    private boolean isVelocity;

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    public SafeTalonFX(int deviceNumber, boolean isDrivetrain, boolean usePID, boolean isVelocity) {
        super(deviceNumber);

        var toConfigure = new TalonFXConfiguration();

        // var motorOutputConfig = new MotorOutputConfigs();
//        motorOutputConfig.NeutralMode = NeutralModeValue.Brake;

        this.usePID = usePID;
        this.isVelocity = isVelocity;

        if (isDrivetrain) {
            // Old commented-out settings
//            configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 20.0, 25.0, 1.0))
//            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 10.0, 15.0,0.5))
//            configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 25.0, 1.0))
//            configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 20.0, 15.0,0.5))
        } else {
            var currentLimitsConfig = new CurrentLimitsConfigs();

            currentLimitsConfig.StatorCurrentLimit = 100;
            currentLimitsConfig.StatorCurrentLimitEnable = true;

            currentLimitsConfig.SupplyCurrentLimit = 80;
            currentLimitsConfig.SupplyCurrentThreshold = 60;
            currentLimitsConfig.withSupplyTimeThreshold(60);
            currentLimitsConfig.SupplyCurrentLimitEnable = true;

            // Old not-commented-out settings
//            configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 75.0, 1.0));
//            configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80.0, 60.0, 0.5));
            toConfigure.CurrentLimits = currentLimitsConfig;
        }

//        motorOutputConfig.withDutyCycleNeutralDeadband(deadBand);

        if (isVelocity) {
        // Uncomment to get Velocity Closed Loop control.
        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
            toConfigure.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
            toConfigure.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
            toConfigure.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
            toConfigure.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        } else {
            toConfigure.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
            toConfigure.Slot0.kI = 0; // No output for integrated error
            toConfigure.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
        }
        
        // Peak output of 8 volts
        toConfigure.Voltage.PeakForwardVoltage = 8;
        toConfigure.Voltage.PeakReverseVoltage = -8;

        // toConfigure.MotorOutput = motorOutputConfig;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = getConfigurator().apply(toConfigure);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    //@Override 
    public void set(Double percentOutput) {
        if (usePID && isVelocity && Math.abs(percentOutput) > 0.1) {
            setControl(m_voltageVelocity.withVelocity(percentOutput * maxSpeed));
        } else if (usePID && !isVelocity && Math.abs(percentOutput) > 0.1) {
            setControl(m_positionVoltage.withPosition(percentOutput));
        } else {
            super.set(percentOutput);
        }
    }
}
