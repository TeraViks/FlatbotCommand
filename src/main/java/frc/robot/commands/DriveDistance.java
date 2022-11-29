// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class DriveDistance extends InstantCommand {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_maxSpeed;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double maxSpeed, double inches, Drivetrain drive) {
    m_maxSpeed = maxSpeed;
    m_distance = inches;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.driveDistance(m_maxSpeed, m_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
