// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final  Shooter shooter = new Shooter();          
  private final Intake intake = new Intake();                                                      
    // CommandJoystick rotationController = new CommandJoystick(1);

  private final Photonvision m_photonvision = new Photonvision();                                                                      
  // CommandJoystick rotationController = new CommandJoystick(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();



    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    Command driveFieldOrientedDirectAngleSim = m_drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.start().onTrue((new InstantCommand(drivebase::zeroGyro)));
    driverXbox.a().whileTrue(new ParallelCommandGroup(
      new InstantCommand(()->shooter.shoot()).handleInterrupt(() -> shooter.stop()),
      new InstantCommand(()->intake.in()).handleInterrupt(() -> intake.stop()))
    );
    driverXbox.b().onTrue(new InstantCommand(()->shooter.stop()));

    /** temporary intake */
    driverXbox.leftBumper().whileTrue(
      new InstantCommand(()->shooter.shooterIntake()).handleInterrupt(() -> shooter.stop())
    );
      //   driverXbox.leftBumper().onFalse(new InstantCommand(()->shooter.stop()));
    //intake
     driverXbox.y().whileTrue(new InstantCommand(()->intake.in()).handleInterrupt(() -> intake.stop()));
     driverXbox.x().whileTrue(new InstantCommand(()->intake.in()).handleInterrupt(() -> intake.stop()));
     driverXbox.y().onFalse(new InstantCommand(()->intake.stop()));
     driverXbox.x().onFalse(new InstantCommand(()->intake.stop()));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return m_drivebase.getAutonomousCommand("New Path");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }

  public void periodic(){

    // Check to see if photonvision can see Apriltag targets. if so, get an estimated robot pose based on target and update the odometry
	if (m_photonvision.hasTargets()) {
			Optional<EstimatedRobotPose> estimatedPose = m_photonvision.getEstimatedGlobalPose(m_drivebase.getPose());
			if (estimatedPose.isPresent()) {
				Pose2d robotPose2d = estimatedPose.get().estimatedPose.toPose2d();
				double distance = m_photonvision.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

				//Scale confidence in Vision Measurements based on distance
			//	m_drivebase.setVisionMeasurementStdDevs(MatBuilder.fill(Nat.N3(), Nat.N1(), distance * 1.2, distance * 1.2, 0.01));//TODO: find and fix!
        //Add VisionMeasurement to odometry
				m_drivebase.addVisionMeasurement(new Pose2d(robotPose2d.getTranslation(), m_drivebase.getHeading()), estimatedPose.get().timestampSeconds);
			}
		}

  }
}
