// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmToTarget extends Command {
  private final Arm arm;
  private double pos;
  private double setpoint;
  private boolean end;
  /** Creates a new HoldArm. */
  public ArmToTarget(Arm ar, double target) {
    arm = ar;
    setpoint = target;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = arm.getPos();
    System.out.println("holding arm");
    end = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      arm.runToPosition(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("arm at target");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (MathUtil.isNear(setpoint, arm.getPos(), 0.01)) {
      return true;
    } else {
      return false;
    }
  }
}