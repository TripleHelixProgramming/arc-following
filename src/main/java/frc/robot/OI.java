/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI INSTANCE;

  /**
   * @return retrieves the singleton instance of the Operator Interface
   */
  public static OI getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new OI();
    }
    return INSTANCE;
  }

  private final Joystick driver = new Joystick(ControllerMap.DRIVER_PORT);
  private final Joystick operator = new Joystick(ControllerMap.OPERATOR_PORT);
  
  private OI() { }

  /**
   * @return the raw controller throttle
   */
  public double getThrottle () {
    // return -driver.getRawAxis(ControllerMap.LEFT_STICK_Y); 
    return -driver.getRawAxis(ControllerMap.LEFT_TRIGGER) + driver.getRawAxis(ControllerMap.RIGHT_TRIGGER);
	}
	
	/**
   * @return the raw controller turn
   */
	public double getTurn() {
		return driver.getRawAxis(ControllerMap.RIGHT_STICK_X);
	}
}
