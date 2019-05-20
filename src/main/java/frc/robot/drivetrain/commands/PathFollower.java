/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import static frc.robot.drivetrain.Drivetrain.ENCODER_TICKS_PER_REVOLUTION;
import static frc.robot.drivetrain.Drivetrain.MAX_VELOCITY;
import static frc.robot.drivetrain.Drivetrain.WHEEL_DIAMETER;
import static frc.robot.drivetrain.Drivetrain.getDrivetrain;
import static jaci.pathfinder.Pathfinder.boundHalfDegrees;
import static jaci.pathfinder.Pathfinder.r2d;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower extends Command {

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;

  private Trajectory leftTrajectory;
  private Trajectory rightTrajectory;

  private Notifier notifier = new Notifier(this::followPath);

  public PathFollower(String pathName) {
    requires(getDrivetrain());

    importPath(pathName);
    leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);
    rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY, 0);
  }

  @Override
  protected void initialize() {
    leftFollower.configureEncoder(getDrivetrain().getLeftPosition(), ENCODER_TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
    rightFollower.configureEncoder(getDrivetrain().getRightPosition(), ENCODER_TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
    
    notifier.startPeriodic(leftTrajectory.get(0).dt);
  }

  @Override
  protected boolean isFinished() {
    return leftFollower.isFinished() || rightFollower.isFinished();
  }

  @Override
  protected void end() {
    notifier.stop();
  }

  @Override
  protected void interrupted() {
    notifier.stop();
  }

  private void importPath(String pathName) {
    try {
      leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
      rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");

      leftFollower = new EncoderFollower(leftTrajectory);
      rightFollower = new EncoderFollower(rightTrajectory);
    } catch (IOException e) {
		  e.printStackTrace();
	  }
  }

  private void followPath() {
    if (leftFollower.isFinished() || rightFollower.isFinished()) {
      notifier.stop();
      return;
    } 
    
    double left_speed = leftFollower.calculate(getDrivetrain().getLeftPosition());
    double right_speed = rightFollower.calculate(getDrivetrain().getRightPosition());
    double heading = getDrivetrain().getYaw();
    double desired_heading = r2d(leftFollower.getHeading());
    double heading_difference = boundHalfDegrees(desired_heading - heading);
    double turn =  0.8 * (-1.0/80.0) * heading_difference;
    getDrivetrain().tankDrive(left_speed + turn, right_speed - turn);
  }
}
