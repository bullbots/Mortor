// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.lang.reflect.Field;

/**
 * Wrapper for the PIDController setting the PID values
 * @param m_kp: Double
 * @param m_ki: Double
 * @param kd: Double
 */
public class PIDControllerDebug extends PIDController {

    public Field totalErrorField; //WARN: used lateinit in kotlin
    private boolean prev = false;

//    fun totalError(): Double {
//        var totalError = 0.0
//        try {
//            totalError = totalErrorField[this] as Double
//        } catch (e: IllegalArgumentException) {
//            e.printStackTrace()
//        } catch (e: IllegalAccessException) {
//            e.printStackTrace()
//        }
//        return totalError
//    }

    public PIDControllerDebug(double m_kp, double m_ki, double kd) {
        super(m_kp, m_ki, kd);
        try {
            totalErrorField = this.getClass().getDeclaredField("m_totalError");
            totalErrorField.trySetAccessible();
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        } catch (SecurityException e) {
            e.printStackTrace();
        }
//        SmartDashboard.putNumber("PID Position Output", positionError * p)
//        SmartDashboard.putNumber("PID Integral Output", totalErrorField.getDouble(totalErrorField) * i)
//        SmartDashboard.putNumber("PID output", 0.0)
    }

    public double calculateDebug(double measurement, double setpoint, boolean displayOutput) {
        int outputRegion = 0;
        double delta = setpoint - measurement;
        double output;
        if (Math.abs(delta) > 45) {
            output = Math.signum(delta) * 0.55;
        } else if(Math.abs(delta) > 1){
            output = Math.signum(delta) * 0.35;
        } else {
            output = 0.0;
        }

        if (displayOutput) {
            double totalError = 0.0;
            try {
                totalError = (double) totalErrorField.get(this);
            } catch (IllegalAccessException e){
                e.printStackTrace();
            }
            /* WARN: removed this chunk of code as it was difficult to easily replicate in java
            try {
                totalError = totalErrorField[this] as Double;
            } catch (e: IllegalArgumentException) {
                e.printStackTrace()
            } catch (e: IllegalAccessException) {
                e.printStackTrace()
            }
            */
            double positionOutput = getP() * getPositionError();
            double integralOutput = getI() * totalError;

//            SmartDashboard.putNumber("PID Position Output", positionOutput)
//            SmartDashboard.putNumber("PID Integral Output", integralOutput)
//            SmartDashboard.putNumber("PID Derivative Output", )
//            SmartDashboard.putNumber("PID output", output)
//            SmartDashboard.putNumber("PID Expected Output", positionOutput + integralOutput + (sign(delta) * 0.1))


        }

        return output;

    }




}
