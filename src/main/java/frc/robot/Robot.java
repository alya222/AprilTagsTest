// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  //@Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    LRearWheel.follow(LFrontWheel);
    RRearWheel.follow(RFrontWheel);
  }

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.01;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  XboxController xboxController = new XboxController(0);

  // Drive motors
  private CANSparkMax LFrontWheel = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax RFrontWheel = new CANSparkMax(3, MotorType.kBrushless);

  private CANSparkMax LRearWheel = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax RRearWheel = new CANSparkMax(4, MotorType.kBrushless);

  private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

  //@Override
  public void teleopPeriodic() {
      double forwardSpeed;
      double rotationSpeed;

      forwardSpeed = xboxController.getRightY();

      if (xboxController.getAButton()) {
          // Vision-alignment mode
          // Query the latest result from PhotonVision
          var result = camera.getLatestResult();

          if (result.hasTargets()) {
              // Calculate angular turn power
              // -1.0 required to ensure positive PID controller effort _increases_ yaw
              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
              forwardSpeed = forwardController.calculate(result.getBestTarget().getYaw(), 0);
          } else {
              // If we have no targets, stay still.
              rotationSpeed = 0;
          }
      } else {
          // Manual Driver Mode
          rotationSpeed = xboxController.getLeftX();
      }

      // Use our forward/turn speeds to control the drivetrain
      roboDrive.arcadeDrive(rotationSpeed, forwardSpeed);
  }
}



/* 
public class Robot extends TimedRobot {
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(4);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(0);

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.01;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.01;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    XboxController xboxController = new XboxController(0);

    // Drive motors
    private CANSparkMax LFrontWheel = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax RFrontWheel = new CANSparkMax(3, MotorType.kBrushless);

    private CANSparkMax LRearWheel = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax RRearWheel = new CANSparkMax(4, MotorType.kBrushless);

    private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

    @Override
    public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    LRearWheel.follow(LFrontWheel);
    RRearWheel.follow(RFrontWheel);
  }

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

    //D
    forwardSpeed = xboxController.getRightY();


        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range*/
               /* D double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));*/

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                //D forwardSpeed = forwardController.calculate(range, GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                /*rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                // If we have no targets, stay still.
                //D forwardSpeed = 0;
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            //D forwardSpeed = xboxController.getRightY();
            rotationSpeed = xboxController.getLeftX();
        }

        // Use our forward/turn speeds to control the drivetrain
        roboDrive.arcadeDrive(rotationSpeed, forwardSpeed);
    }
}
*/