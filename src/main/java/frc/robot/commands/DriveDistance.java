// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private final PIDController m_rightPIDcontroller;
  private final PIDController m_leftPIDcontroller;
  private final double m_rightkP;
  private final double m_rightkI;
  private final double m_rightkD;
  private final double m_leftkP;
  private final double m_leftkI;
  private final double m_leftkD;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    // Potential values that worked for another flatbot -- Good starting place
    m_rightkP = 0.5;
    m_rightkI = 0.5;
    m_rightkD = 0.1;
    m_leftkP = 0.5;
    m_leftkI = 0.5;
    m_leftkD = 0.1;
    m_rightPIDcontroller = new PIDController(m_rightkP, m_rightkI, m_rightkD);
    m_leftPIDcontroller = new PIDController(m_leftkP, m_leftkI, m_leftkD);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.tankDrive(0, 0, false);
    m_drive.resetEncoders();
    m_leftPIDcontroller.reset();
    m_rightPIDcontroller.reset();
    /** 
     * Could add potential tuning paramters such as:
     * pid.setIntegratorRange(minRange, maxRange) -- The integrator will only activate once inside the range of the parameters
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(
      //Left PID Controller Calculations
      m_leftPIDcontroller.calculate(m_drive.getLeftDistanceInch(), m_distance), //Could possible use the MathUtil.clamp() if we need to keep the speed down
      //Right PID Controller Calculations
      m_rightPIDcontroller.calculate(m_drive.getRightDistanceInch(), m_distance),
      // Squared Inputs
      false
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
