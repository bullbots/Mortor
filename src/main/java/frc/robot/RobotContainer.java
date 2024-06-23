// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drivetrain_Commands.JoystickDrive;
import frc.robot.commands.Intake_Commands.IntakeGroup;
import frc.robot.commands.Shooter_Commands.ShooterCargos;
import frc.robot.commands.Shooter_Commands.ShooterGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainFalcon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.concurrent.atomic.AtomicReference;


public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private static final Joystick stick = new Joystick(0);

    private static final JoystickButton[] buttons = new JoystickButton[12];
    private static final JoystickButton[] cobuttons = new JoystickButton[12];

    private static final Joystick coStick = new Joystick(1);

    public enum Color {
        UNLOADED(0),
        RED(1),
        BLUE(2);

        private int value;

        Color(int value) {
            this.value = value;
        }

        public String toString() {
            return Integer.toString(value);
        }
    }

    public enum Letter {
        UNLOADED(0),
        A(1),
        B(2);

        private int value;

        Letter(int value) {
            this.value = value;
        }

        public String toString() {
            return Integer.toString(value);
        }
    }

    private static AtomicReference pathColor = new AtomicReference<>(Color.UNLOADED);
    private static AtomicReference pathLetter = new AtomicReference<>(Letter.UNLOADED);

    // Subsystems
    private static DrivetrainFalcon drivetrain = new DrivetrainFalcon();
    private static Intake intake = new Intake();
    private static Shooter shooter = new Shooter();
//    private static Climber climber = new Climber();

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer()
    {
        configureButtons();
        configureBindings();
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain,
                () -> -stick.getY() * ((buttons[3].getAsBoolean()) ? -0.5 : 0.5),  // Because Negative Y is forward on the joysticks
                () -> stick.getX() * 0.5,
                () ->(stick.getZ() - 1) / -2.0));

        m_chooser.setDefaultOption("Path Planner Auto", new PathPlannerAuto("Auto"));
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    private void configureButtons() {
        // Index matches button numbers, buttons[0] is null
        for (int i = 1; i <= 12; i++) {
            buttons[i] = new JoystickButton(stick, i);
        }
        // Index matches button numbers, cobuttons[0] is null
        for (int i = 1; i <= 12; i++) {
            cobuttons[i] = new JoystickButton(coStick, i);
        }
    }

    private void configureBindings()
    {
        buttons[1].whileTrue(new IntakeGroup(intake, 0.3, 0.6, shooter))
                .whileFalse(new ShooterCargos(shooter, true, ()->-0.3).withTimeout(0.3));

        buttons[6].whileTrue(new ShooterGroup(intake, shooter, true, ()->0.3));
    }

    public Command getAutonomousCommand()
    {
        return m_chooser.getSelected();
    }
}
