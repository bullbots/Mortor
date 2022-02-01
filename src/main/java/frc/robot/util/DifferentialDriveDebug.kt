// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import kotlin.math.abs
import kotlin.math.withSign

/** Add your docs here.  */
class DifferentialDriveDebug(leftMotor: MotorController?, rightMotor: MotorController?) :
    DifferentialDrive(leftMotor, rightMotor) {

    private var m_rightSideInvertMultiplier = -1.0
    private val kDefaultQuickStopThreshold = 0.2
    private val kDefaultQuickStopAlpha = 0.1

    private val m_quickStopThreshold = kDefaultQuickStopThreshold
    private val m_quickStopAlpha = kDefaultQuickStopAlpha
    private val m_quickStopAccumulator = 0.0

    // Run super version but then output math for setting motors.
    override fun arcadeDrive(xSpeed: Double, zRotation: Double, squareInputs: Boolean) {

        var xSpeed = xSpeed
        var zRotation = zRotation
        super.arcadeDrive(xSpeed, zRotation, squareInputs)

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0)
        xSpeed = applyDeadband(xSpeed, m_deadband)

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0)
        zRotation = applyDeadband(zRotation, m_deadband)

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = (xSpeed * xSpeed).withSign(xSpeed)
            zRotation = (zRotation * zRotation).withSign(zRotation)
        }

        val leftMotorOutput: Double
        val rightMotorOutput: Double

        val maxInput = abs(xSpeed).coerceAtLeast(abs(zRotation)).withSign(xSpeed)
        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput
                rightMotorOutput = xSpeed - zRotation
            } else {
                leftMotorOutput = xSpeed + zRotation
                rightMotorOutput = maxInput
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation
                rightMotorOutput = maxInput
            } else {
                leftMotorOutput = maxInput
                rightMotorOutput = xSpeed - zRotation
            }
        }

        // SmartDashboard.putNumber("Left Motor - ArcadeDrive", MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
        val rightSideInvertMultiplier = if (isRightSideInverted) -1.0 else 1.0
        val maxOutput = m_maxOutput * rightSideInvertMultiplier
        // SmartDashboard.putNumber("Right Motor - ArcadeDrive", MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);
    }

    override fun curvatureDrive(xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        super.curvatureDrive(xSpeed, zRotation, isQuickTurn)

        // xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        // xSpeed = applyDeadband(xSpeed, m_deadband);

        // zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        // zRotation = applyDeadband(zRotation, m_deadband);

        // double angularPower;
        // boolean overPower;

        // if (isQuickTurn) {
        //     if (Math.abs(xSpeed) < m_quickStopThreshold) {
        //         m_quickStopAccumulator =
        //             (1 - m_quickStopAlpha) * m_quickStopAccumulator
        //                 + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
        //     }
        //     overPower = true;
        //     angularPower = zRotation;
        // } else {
        //     overPower = false;
        //     angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

        //     if (m_quickStopAccumulator > 1) {
        //         m_quickStopAccumulator -= 1;
        //     } else if (m_quickStopAccumulator < -1) {
        //         m_quickStopAccumulator += 1;
        //     } else {
        //         m_quickStopAccumulator = 0.0;
        //     }
        // }

        // double leftMotorOutput = xSpeed + angularPower;
        // double rightMotorOutput = xSpeed - angularPower;

        // // If rotation is overpowered, reduce both outputs to within acceptable range
        // if (overPower) {
        //     if (leftMotorOutput > 1.0) {
        //         rightMotorOutput -= leftMotorOutput - 1.0;
        //         leftMotorOutput = 1.0;
        //     } else if (rightMotorOutput > 1.0) {
        //         leftMotorOutput -= rightMotorOutput - 1.0;
        //         rightMotorOutput = 1.0;
        //     } else if (leftMotorOutput < -1.0) {
        //         rightMotorOutput -= leftMotorOutput + 1.0;
        //         leftMotorOutput = -1.0;
        //     } else if (rightMotorOutput < -1.0) {
        //         leftMotorOutput -= rightMotorOutput + 1.0;
        //         rightMotorOutput = -1.0;
        //     }
        // }

        // // Normalize the wheel speeds
        // double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        // if (maxMagnitude > 1.0) {
        //     leftMotorOutput /= maxMagnitude;
        //     rightMotorOutput /= maxMagnitude;
        // }

        // SmartDashboard.putNumber("Left Motor - CurvatureDrive", leftMotorOutput * m_maxOutput);
        // double rightSideInvertMultiplier = isRightSideInverted() ? -1.0 : 1.0;
        // SmartDashboard.putNumber("Right Motor - CurvatureDrive", rightMotorOutput * m_maxOutput * rightSideInvertMultiplier);
    }

    var isRightSideInverted: Boolean
        get() = m_rightSideInvertMultiplier == -1.0
        set(rightSideInverted) {
            m_rightSideInvertMultiplier = if (rightSideInverted) -1.0 else 1.0
        }
}