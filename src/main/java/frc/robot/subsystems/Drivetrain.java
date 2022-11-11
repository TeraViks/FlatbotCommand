// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotContainer;

import java.lang.Math;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonSRX rightFrontTalon = new WPI_TalonSRX(Constants.RIGHT_FRONT_CAN_ID);
  private final WPI_TalonSRX rightRearTalon = new WPI_TalonSRX(Constants.RIGHT_REAR_CAN_ID);
  private final WPI_TalonSRX leftFrontTalon = new WPI_TalonSRX(Constants.LEFT_FRONT_CAN_ID);
  private final WPI_TalonSRX leftRearTalon = new WPI_TalonSRX(Constants.LEFT_REAR_CAN_ID);

  //Create motor groups
  private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightFrontTalon, rightRearTalon);
  private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftFrontTalon, leftRearTalon);

  // Drivetrain
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);

  public Drivetrain() {
    diffDrive.setDeadband(Constants.DEADBAND_SIZE);
    rightRearTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    leftRearTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    resetEncoders();
  }

  // public void init() {
  //   rightFrontTalon.setInverted(false);
  //   rightRearTalon.setInverted(false);
  //   // leftFrontTalon.setInverted(true);
  //   // leftRearTalon.setInverted(true);
  // // }
  
  // public void Drive(double x, double y) {
  //   rightFrontTalon.set(ControlMode.PercentOutput, y);
  //   rightRearTalon.set(ControlMode.PercentOutput, y);
  //   leftFrontTalon.set(ControlMode.PercentOutput, y);
  //   leftRearTalon.set(ControlMode.PercentOutput, y);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   if (Math.random() <= 0.1){
      System.out.println("--------");
      System.out.println(getLeftEncoderCount());
      System.out.println(getRightEncoderCount());
    }
      diffDrive.arcadeDrive(
      RobotContainer.getJoyX(),
      RobotContainer.getJoyY()*Constants.SPEED_FACTOR
    );
  }

public void arcadeDrive(double m_speed, double m_speed2) {
  diffDrive.arcadeDrive(
    m_speed,
    m_speed2
  );
}

public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
  diffDrive.tankDrive(
    leftSpeed,
    rightSpeed,
    squaredInputs
   );
}

public double getRightEncoderCount() {
    return -rightRearTalon.getSelectedSensorPosition();
}

public double getLeftEncoderCount() {
    return leftRearTalon.getSelectedSensorPosition();
}

public double getLeftEncoderVelocity() {
  return leftRearTalon.getSelectedSensorVelocity();
}

public double getRightEncoderVelocity() {
  return -rightRearTalon.getSelectedSensorVelocity();
}

public void resetEncoders() {
  leftRearTalon.setSelectedSensorPosition(0.0);
  rightRearTalon.setSelectedSensorPosition(0.0);
}

public double getRightDistanceInch() {
  return Constants.WHEEL_DIAMETER * Math.PI * getRightEncoderCount() / Constants.TICK_P_ROT;
}

public double getLeftDistanceInch() {
  return Constants.WHEEL_DIAMETER * Math.PI * getLeftEncoderCount() / Constants.TICK_P_ROT;
}

public double getAverageDistanceInch() {
  return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
}
}
