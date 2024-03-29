// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class HoldArm extends Command {
  private final Arm arm;
  private double pos;
  /** Creates a new HoldArm. */
  public HoldArm(Arm ar) {
    arm = ar;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = arm.getPos();
    System.out.println("holding arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.RunArm(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Hold interupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}