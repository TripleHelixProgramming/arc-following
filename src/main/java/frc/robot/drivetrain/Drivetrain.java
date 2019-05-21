/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import static com.ctre.phoenix.motorcontrol.ControlMode.Velocity;
import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;
import static java.lang.Math.PI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team319.models.BobTalonSRX;
import com.team319.models.LeaderBobTalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {

  private static Drivetrain INSTANCE = new Drivetrain();

  /**
   * @return the singleton instance of the Drivetrain subsystem
   */
  public static Drivetrain getDrivetrain() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private static final double WHEEL_DIAMETER_IN_INCHES = 5;
  private static final int ENCODER_TICKS_PER_REVOLUTION = 120;
  public static final double MAX_VELOCITY_IN_FPS = 10;

  private static final int VELOCITY_CONTROL_SLOT = 0;

  private static int RIGHT_MASTER_ID = 11;
  private static int RIGHT_SLAVE_1_ID = 12;
  private static int RIGHT_SLAVE_2_ID = 13;
  private static int LEFT_MASTER_ID = 14;
  private static int LEFT_SLAVE_1_ID = 15;
  private static int LEFT_SLAVE_2_ID = 16;

  private static int PIGEON_ID = 17;

  //  Competition & Practice Bot  Talon Masters with Victors as Slaves.
  private BaseMotorController rightSlave1 = new BobTalonSRX(RIGHT_SLAVE_1_ID);
  private BaseMotorController rightSlave2 = new BobTalonSRX(RIGHT_SLAVE_2_ID);
  private BaseMotorController leftSlave1 = new BobTalonSRX(LEFT_SLAVE_1_ID);
  private BaseMotorController leftSlave2 = new BobTalonSRX(LEFT_SLAVE_2_ID);

  private LeaderBobTalonSRX left = new LeaderBobTalonSRX(LEFT_MASTER_ID,
      leftSlave1, leftSlave2);
  private LeaderBobTalonSRX right = new LeaderBobTalonSRX(RIGHT_MASTER_ID,
      rightSlave1, rightSlave2);

  PowerDistributionPanel pdp = new PowerDistributionPanel();
  private PigeonIMU pigeon = new PigeonIMU(PIGEON_ID);

  private Drivetrain() {
    setPIDFValues();
    setBrakeMode(Brake);
    setupSensors();
    setupLogs();

    left.setSensorPhase(false);
    right.setSensorPhase(false);
    left.setInverted(true);
    right.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new NormalizedArcadeDrive());
  }

  public void tankDrive(double leftPercent, double rightPercent) {
    left.set(PercentOutput, convertFromFeetToTicks(leftPercent * MAX_VELOCITY_IN_FPS));
    right.set(PercentOutput, convertFromFeetToTicks(rightPercent * MAX_VELOCITY_IN_FPS));
  }

  private void setPIDFValues() {
    left.configPIDF(VELOCITY_CONTROL_SLOT, 0, 0, 0, 0);
    right.configPIDF(VELOCITY_CONTROL_SLOT, 0, 0, 0, 0);
  }

  private void setupSensors() {
    left.configPrimaryFeedbackDevice(FeedbackDevice.QuadEncoder);
    right.configPrimaryFeedbackDevice(FeedbackDevice.QuadEncoder);
  }

  private void setBrakeMode(NeutralMode neutralMode) {
    left.setNeutralMode(neutralMode);
    right.setNeutralMode(neutralMode);
  }

  private void setupLogs() {

    // HelixLogger.getInstance().addDoubleSource("TOTAL CURRENT", pdp::getTotalCurrent);
    // HelixLogger.getInstance().addDoubleSource("DT LM Current", left::getOutputCurrent);
    // HelixLogger.getInstance().addDoubleSource("DT RM Current", right::getOutputCurrent);

    // //  This logging format should work for Talons OR Victor SLAVES.
    // HelixLogger.getInstance().addDoubleSource("DT LS1 Current", () -> pdp.getCurrent(LEFT_SLAVE_1_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT LS2 Current", () -> pdp.getCurrent(LEFT_SLAVE_2_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT RS1 Current", () -> pdp.getCurrent(RIGHT_SLAVE_1_PDP));
    // HelixLogger.getInstance().addDoubleSource("DT RS2 Current", () -> pdp.getCurrent(RIGHT_SLAVE_1_PDP));

    // HelixLogger.getInstance().addDoubleSource("PIGEON HEADING", () -> this.getYaw());

    // HelixLogger.getInstance().addDoubleSource("DRIVETRAIN LEFT Voltage", left::getMotorOutputVoltage);
    // HelixLogger.getInstance().addIntegerSource("DRIVETRAIN LEFT Velocity", this::getLeftVelocity);
    // HelixLogger.getInstance().addDoubleSource("DRIVETRAIN RIGHT Voltage", right::getMotorOutputVoltage);
    // HelixLogger.getInstance().addIntegerSource("DRIVETRAIN RIGHT Velocity", this::getRightVelocity);
  }

  public void resetEncoders() {
    left.getSensorCollection().setQuadraturePosition(0, 0);
    right.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public void resetHeading() {
    pigeon.setYaw(0.0);
  }

  public double getYaw() {
    double [] ypr = {0, 0, 0};
    pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getLeftVelocity() {
    try {
      return convertFromTicksToFeet(left.getSelectedSensorVelocity());
    } catch(Exception e) {
      SmartDashboard.putString("Exception", e.getMessage());
      return 100000000;
    } catch(Throwable t) {
      SmartDashboard.putString("Exception", t.getMessage());
      return 1999999999;
    }
  }

  public double getRightVelocity() {
    return convertFromTicksToFeet(right.getSelectedSensorVelocity());
  }

  public double getLeftPosition() {
    return convertFromTicksToFeet(left.getSelectedSensorPosition());
  }

  public double getRightPosition() {
    return  convertFromTicksToFeet(right.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon Yaw", getYaw());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
  }

  private double convertFromTicksToFeet(int ticks) {
    return PI * WHEEL_DIAMETER_IN_INCHES * ticks / ENCODER_TICKS_PER_REVOLUTION;
  }

  private double convertFromFeetToTicks(double feet) {
    return ENCODER_TICKS_PER_REVOLUTION * feet / (PI * WHEEL_DIAMETER_IN_INCHES);
  }
}
