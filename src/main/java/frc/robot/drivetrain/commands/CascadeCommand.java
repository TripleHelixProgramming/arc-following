/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.PIDController;

import static frc.robot.drivetrain.Drivetrain.getDrivetrain;

public class CascadeCommand extends Command {

  private final double PERIOD = 0.01;

  PIDController throttleController = new PIDController(0, 0, 0, PERIOD);
  PIDController turnController = new PIDController(0, 0, 0, PERIOD);

  Notifier notifier = new Notifier(this::calculate);

  public CascadeCommand() {
    requires(getDrivetrain());
  }

  @Override
  protected void initialize() {
    throttleController.setReference(10); // 10 FPS
    turnController.setReference(0); // 0 degrees
    notifier.startPeriodic(PERIOD);
  }

  private void calculate() {
    double velocity = (getDrivetrain().getLeftVelocity() + getDrivetrain().getRightVelocity()) / 2;
    double velocityOutput = throttleController.calculate(velocity);
    double turnOutput = turnController.calculate(getDrivetrain().getYaw());

    getDrivetrain().tankDrive(velocityOutput + turnOutput, velocityOutput - turnOutput);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    notifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    notifier.stop();
  }
}
