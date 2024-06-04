// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.PositionForShot;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpitBackOut;
import frc.robot.commands.StopShooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve")); // "swerve" if on robot 5. "swerve-practice" if on red-bot
  private static final Shooter shooter = new Shooter();
  // CommandJoystick rotationController = new CommandJoystick(1);

  private final Photonvision m_photonvision = new Photonvision();
  // CommandJoystick rotationController = new CommandJoystick(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController = new CommandXboxController(0);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  CommandXboxController operatorController = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;
  private final Trigger intaking;

  // Trigger intakeHasNote = new Trigger(shooter::hasNote);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    namedCommandsConfig();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    intaking = new Trigger(shooter::intaking);

    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRightX());

    Command driveFieldOrientedDirectAngleSim = m_drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRawAxis(2));

    shooter.setDefaultCommand(new StopShooter(shooter));
    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    driverController.start().onTrue((new InstantCommand(m_drivebase::zeroGyro))); 
    operatorController.a().whileTrue(new Shoot(shooter));
    operatorController.b().onTrue(new InstantCommand(() -> shooter.stop(), shooter));
    intaking.whileTrue(new RunCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1))
        .unless(() -> !DriverStation.isTeleopEnabled()));
    intaking.whileFalse(new RunCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));

    driverController.rightBumper().whileTrue(
        getPOVTurnCommand((int) (Math.round(Math.toDegrees(-m_photonvision.getSpeakerTarget()))))
            .unless(() -> !m_photonvision.hasSpeakerTarget()));

    // turn based on D-Pad input
    driverController.povUp().whileTrue(getPOVTurnCommand(0));
    driverController.povUpLeft().whileTrue(getPOVTurnCommand(55));
    driverController.povLeft().whileTrue(getPOVTurnCommand(90));
    driverController.povDownLeft().whileTrue(getPOVTurnCommand(135));
    driverController.povDown().whileTrue(getPOVTurnCommand(180));
    driverController.povDownRight().whileTrue(getPOVTurnCommand(-135));
    driverController.povRight().whileTrue(getPOVTurnCommand(-90));
    driverController.povUpRight().whileTrue(getPOVTurnCommand(-55));

    // intake
    operatorController.x().onTrue(new Intake(shooter)
        .andThen(new PositionForShot(shooter))
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))));
    operatorController.y().whileTrue(new ManualIntake(shooter));
    operatorController.leftBumper().whileTrue(new SpitBackOut(shooter));
    // new InstantCommand(()->shooter.stop()).handleInterrupt(() ->
    // shooter.stop())));

    // intakeHasNote.onTrue(new WaitCommand(.3).andThen(new
    // InstantCommand(()->intake.stop())));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));
  }

  public Command getPOVTurnCommand(int direction) {
    return m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> Math.sin(Math.toRadians(direction)),
        () -> Math.cos(Math.toRadians(direction)));
  }

  // Registers the commands shoot and intake in autonumous for use in path planner
  public static void namedCommandsConfig() {
    NamedCommands.registerCommand("shoot",
        // new SequentialCommandGroup(
        // new RunCommand(shooter::shoot),
        // new WaitCommand(1),
        // new InstantCommand(shooter::stop)
        // )
        // new RunCommand(shooter::shoot).withTimeout(2)
        new Shoot(shooter).withTimeout(.75));
    NamedCommands.registerCommand("intake",
        new InstantCommand(shooter::intakeForAutonomous)
    // new Intake(shooter).withTimeout(.1)
    );
    NamedCommands.registerCommand("levelIntake",
        new RunCommand(shooter::spitBackOut).until(() -> !shooter.hasNote()).withTimeout(0.1));
    NamedCommands.registerCommand("intakeStop",
        // new InstantCommand(shooter::stop)
        new StopShooter(shooter).withTimeout(0.0));
    NamedCommands.registerCommand("readyShot",
        // new RunCommand(shooter::spitBackOut).withTimeout(0.06).andThen(new
        // InstantCommand(shooter::stop)).andThen(new WaitCommand(0.5))
        new SpitBackOut(shooter).withTimeout(0.06).andThen(new WaitCommand(0.5))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      m_drivebase.setLastAngleScalar(180); // DEFAULT
    } else {
      m_drivebase.setLastAngleScalar(0); // DEFAULT
    }
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  public void periodic() {

    /**
     * Update the drivetrain odometry with vision measurements when we have april tags in our view
     */
    if (m_photonvision.hasTargets()) {
      Optional<EstimatedRobotPose> estimatedPose = m_photonvision.getEstimatedGlobalPose(m_drivebase.getPose());
      if (estimatedPose.isPresent()) {
        Pose2d robotPose2d = estimatedPose.get().estimatedPose.toPose2d();
    //    double distance = m_photonvision.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        var estimatedStandardDevs = m_photonvision.getEstimationStdDevs(robotPose2d);
        m_drivebase.addVisionMeasurement(robotPose2d, estimatedPose.get().timestampSeconds, estimatedStandardDevs); 

        SmartDashboard.putNumber("camera X", (new Pose2d(robotPose2d.getTranslation(),
            m_drivebase.getHeading()).getX()));
        SmartDashboard.putNumber("camera Y", (new Pose2d(robotPose2d.getTranslation(),
            m_drivebase.getHeading()).getY()));
      }
    }
  }

}
