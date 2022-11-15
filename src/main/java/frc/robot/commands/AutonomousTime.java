// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousTime extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousTime(Drivetrain drivetrain) {
    addCommands(
        new DriveTime(0.1, 5.0, drivetrain)
        //new DriveTime(-0.6, 0.5, drivetrain),
        //new TurnTime(0.5, 1.9, drivetrain),
        //new DriveTime(-0.6, 0.5, drivetrain),
        //new TurnTime(0.5, 1.9, drivetrain)
        );
  }
}
/*
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousTime extends CommandBase {
  //* Creates a new AutonomousTime. 
  public AutonomousTime() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/
