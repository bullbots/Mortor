// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake_Commands.IntakeGroup;
import frc.robot.commands.Shooter_Commands.ShooterCargos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainFalcon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.concurrent.atomic.AtomicReference;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private static final Joystick stick = new Joystick(0);

//    private static final CommandJoystick stick = new CommandJoystick(0);
    private static final JoystickButton button1 = new JoystickButton(stick, 1);
    private static final JoystickButton button2 = new JoystickButton(stick, 2);
    private static final JoystickButton button3 = new JoystickButton(stick, 3);
    private static final JoystickButton button4 = new JoystickButton(stick, 4);
    private static final JoystickButton button5 = new JoystickButton(stick, 5);
    private static final JoystickButton button6 = new JoystickButton(stick, 6);
    private static final JoystickButton button7 = new JoystickButton(stick, 7);
    private static final JoystickButton button8 = new JoystickButton(stick, 8);
    private static final JoystickButton button9 = new JoystickButton(stick, 9);
    private static final JoystickButton button10 = new JoystickButton(stick, 10);
    private static final JoystickButton button11 = new JoystickButton(stick, 11);

    private static final Joystick coStick = new Joystick(1);
    private static final JoystickButton coButton1 = new JoystickButton(coStick, 1);
    private static final JoystickButton coButton2 = new JoystickButton(coStick, 2);
    private static final JoystickButton coButton3 = new JoystickButton(coStick, 3);
    private static final JoystickButton coButton4 = new JoystickButton(coStick, 4);
    private static final JoystickButton coButton5 = new JoystickButton(coStick, 5);
    private static final JoystickButton coButton6 = new JoystickButton(coStick, 6);
    private static final JoystickButton coButton7 = new JoystickButton(coStick, 7);
    private static final JoystickButton coButton8 = new JoystickButton(coStick, 8);
    private static final JoystickButton coButton9 = new JoystickButton(coStick, 9);
    private static final JoystickButton coButton10 = new JoystickButton(coStick, 10);
    private static final JoystickButton coButton11 = new JoystickButton(coStick, 11);

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
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
//    private final CommandXboxController driverController =
//            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Subsystems
    private static DrivetrainFalcon drivetrain = new DrivetrainFalcon();
    private static Intake intake = new Intake();
    private static Shooter shooter = new Shooter();
    private static Climber climber = new Climber();

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
//        new Trigger(exampleSubsystem::exampleCondition)
//                .onTrue(new ExampleCommand(exampleSubsystem));
//
//        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
//        // cancelling on release.
//        driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

//        driverController.button(1).whileTrue()
        button1.whileTrue(new IntakeGroup(intake, 0.3, 0.6, shooter))
                .whileFalse(new ShooterCargos(shooter, true, ()->-0.7));

//        button1.whenHeld(IntakeGroup(intake, 0.3, 0.6, shooter)).whenReleased(ShooterCargos(shooter, true) { -0.7 }
//            .withTimeout(0.3))
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
//        return Autos.exampleAuto(exampleSubsystem);
        return m_chooser.getSelected();
    }
}
