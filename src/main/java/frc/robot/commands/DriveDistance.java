// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_maxSpeed;
  private final PIDController m_rightPIDcontrollerFar;
  private final PIDController m_rightPIDcontrollerClose;
  private final PIDController m_leftPIDcontrollerFar;
  private final PIDController m_leftPIDcontrollerClose;
  private final double m_rightkP;
  private final double m_rightkI;
  private final double m_rightkD;
  private final double m_leftkP;
  private final double m_leftkI;
  private final double m_leftkD;
  private final double m_iLimit;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double maxSpeed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_maxSpeed = maxSpeed;
    m_drive = drive;
    m_iLimit = 40;

    m_leftkP = 0.045;
    m_rightkP = 0.045;

    m_leftkI = 0.000;
    m_rightkI = 0.000;

    m_leftkD = 0.0048;
    m_rightkD = 0.0048;

    m_rightPIDcontrollerClose = new PIDController(m_rightkP, m_rightkI, m_rightkD);
    m_rightPIDcontrollerFar = new PIDController(m_rightkP, 0, m_rightkD);
    m_leftPIDcontrollerClose = new PIDController(m_leftkP, m_leftkI, m_leftkD);
    m_leftPIDcontrollerFar = new PIDController(m_leftkP, 0, m_leftkD);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.tankDrive(0, 0, false);
    m_drive.resetEncoders();
    m_leftPIDcontrollerFar.reset();
    m_leftPIDcontrollerClose.reset();
    System.out.println("hello");
    m_rightPIDcontrollerFar.reset();
    m_rightPIDcontrollerClose.reset();
    m_rightPIDcontrollerClose.setIntegratorRange(-1, 1);
    m_leftPIDcontrollerClose.setIntegratorRange(-1, 1);
    /** 
     * Could add potential tuning paramters such as:
     * pid.setIntegratorRange(minRange, maxRange)
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIDController leftPID, rightPID;
    double leftPIDValue, rightPIDValue;

    if (m_distance - m_drive.getAverageDistanceInch() >= m_iLimit){
       leftPID = m_leftPIDcontrollerFar;
       rightPID = m_rightPIDcontrollerFar;
    }
    else {
      leftPID = m_leftPIDcontrollerClose;
      rightPID = m_rightPIDcontrollerClose;
    }
    leftPIDValue = leftPID.calculate(m_drive.getLeftDistanceInch(), m_distance);
    rightPIDValue = rightPID.calculate(m_drive.getRightDistanceInch(), m_distance);


    SmartDashboard.putNumber("leftPValue", leftPID.getP());
    SmartDashboard.putNumber("leftIValue", leftPID.getI());
    SmartDashboard.putNumber("leftDValue", leftPID.getD());
    SmartDashboard.putNumber("rightPValue", rightPID.getP());
    SmartDashboard.putNumber("rightIValue", rightPID.getI());
    SmartDashboard.putNumber("rightDValue", rightPID.getD());
    SmartDashboard.putNumber("leftPIDValue", leftPIDValue);
    SmartDashboard.putNumber("rightPIDValue", rightPIDValue);
    SmartDashboard.putNumber("leftDistanceInches", m_drive.getLeftDistanceInch());
    SmartDashboard.putNumber("rightDistanceInches", m_drive.getRightDistanceInch());
    SmartDashboard.putNumber("leftError", m_distance - m_drive.getLeftDistanceInch());
    SmartDashboard.putNumber("RightError", m_distance - m_drive.getRightDistanceInch());
    SmartDashboard.putNumber("setpoint", m_distance);

    m_drive.tankDrive(
      //Left PID Controller Calculations
      MathUtil.clamp(leftPIDValue, -m_maxSpeed, m_maxSpeed),
      //Right PID Controller Calculations
      MathUtil.clamp(rightPIDValue, -m_maxSpeed, m_maxSpeed),
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
    return Math.abs(m_distance - m_drive.getAverageDistanceInch()) <= 1;
  }
}
