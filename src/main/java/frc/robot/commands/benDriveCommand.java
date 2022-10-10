// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class benDriveCommand extends CommandBase {
  
  DriveSystem driveSystem;
  PS4Controller controller;
  /*
  double forward;
  double backward;
  double turn;*/

  public benDriveCommand(DriveSystem driveSystem, PS4Controller controller) {
    this.driveSystem = driveSystem;
    this.controller = controller;
    addRequirements(driveSystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //cotroller.getR2 getL2 getRightX
    driveSystem.bensDriveCommand(controller.getR2Axis(), controller.getL2Axis(), controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
