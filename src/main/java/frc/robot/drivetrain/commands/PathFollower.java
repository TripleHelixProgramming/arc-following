/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import static frc.robot.drivetrain.Drivetrain.MAX_VELOCITY_IN_FPS;
import static frc.robot.drivetrain.Drivetrain.getDrivetrain;
import static jaci.pathfinder.Pathfinder.boundHalfDegrees;
import static jaci.pathfinder.Pathfinder.r2d;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

public class PathFollower extends Command {

  private DistanceFollower leftFollower;
  private DistanceFollower rightFollower;

  private Notifier notifier = new Notifier(this::followPath);

  // The starting positions of the left and right sides of the drivetrain
  private double leftStartingDistance;
  private double rightStartingDistance;

  private double period;

  public PathFollower(String pathName) {
    requires(getDrivetrain());

    importPath(pathName);
    leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY_IN_FPS, 0);
    rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / MAX_VELOCITY_IN_FPS, 0);
  }

  @Override
  protected void initialize() {
    // Set the starting positions of the left and right sides of the drivetrain
    leftStartingDistance = getDrivetrain().getLeftPosition();
    rightStartingDistance = getDrivetrain().getRightPosition();

    //Make sure we're starting at the beginning of the path
    leftFollower.reset();
    rightFollower.reset();

    // Start running the path
    notifier.startPeriodic(period);
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
      // Read the path files from the file system
      Trajectory leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");
      Trajectory rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");

      // Set the two paths in the followers
      leftFollower = new DistanceFollower(leftTrajectory);
      rightFollower = new DistanceFollower(rightTrajectory);

      period = leftTrajectory.get(0).dt;

    } catch (IOException e) {
		  e.printStackTrace();
	  }
  }

  private void followPath() {
    // If either of the followers have finished their paths we need to stop the notifier
    if (leftFollower.isFinished() || rightFollower.isFinished()) {
      notifier.stop();
      return;
    } 
    
    // Get the left and right power output from the distance calculator
    double left_speed = leftFollower.calculate(getDrivetrain().getLeftPosition() - leftStartingDistance);
    double right_speed = rightFollower.calculate(getDrivetrain().getRightPosition() - rightStartingDistance);

    // Calculate any correction we need based on the current and desired heading
    double heading = getDrivetrain().getYaw();
    double desired_heading = r2d(leftFollower.getHeading());
    double heading_difference = boundHalfDegrees(desired_heading - heading);
    double turn =  0.8 * (-1.0/80.0) * heading_difference;

    // Send the % outputs to the drivetrain
    getDrivetrain().tankDrive(left_speed + turn, right_speed - turn);
  }
}
