// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*import frc.robot.commands.ExampleCommand;*/
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
/*import frc.robot.subsystems.ExampleSubsystem;*/
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 /* private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();*/
  public static final Drivetrain m_drivetrain = new Drivetrain();

/*
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
*/
  public static Joystick joystick = new Joystick(0);

  /**Sendable Chooser Code to select autonomous mode version */
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  
  public static double getJoyX() {
    return joystick.getX();
  }

  public static double getJoyY() {
    return joystick.getY();
  }

  public static double getJoyZ() {
    return joystick.getZ(); 
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        
        // Setup SmartDashboard options
        m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
        m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
        SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
