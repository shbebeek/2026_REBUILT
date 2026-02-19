// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // TODO: configure the swerve modules and get the deploy folder
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  private final HoodSubsystem hood = new HoodSubsystem();

  private final Superstructure superstructure = new Superstructure(intake, hopper, feeder, turret, shooter, climber, hood);

  private final SendableChooser<Command> autoChooser;

  // track current alliance for change detection
  private Alliance currentAlliance = Alliance.Red;

  // contains subsystems, i/o devices, and commands
  public RobotContainer(){
    // configure trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    // intialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // set up trigger to detect alliance changes
    new Trigger(() -> getAlliance() != currentAlliance)
      .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoreDisable(true);
    
    // triggers for auto aim/pass poses
    new Trigger(() -> isInAllianceZone())
      .onChange(Commands.runOnce(() -> onZoneChanged()).ignoredDisable(true));
    
    new Trigger(() -> isOnAllianceOutpostSide())
      .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));
      
    if(!Robot.isReal() || true){
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // have autoChooser pull all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // set default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // add a simple auto option to have the robot drive backward for 1 second then stop
    autoChooser.addOption("Drive Backward", drivebase.driveBackwards().withTimeout(1));
    
    // put autoChooser on SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings(){
    // set up controllers
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, drivebase, superstructure);
    OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, drivebase, superstructure);
    PoseControls.configure(Constants.ControllerConstants.kPoseControllerPort,drivebase);
  }

  private void buildNamedAutoCommands(){
    // add any auto commands to NamedCommands here
    NamedCommands.registerCommand("driveBackwards", drivebase.driveBackwards().withTimeout(1).withName("Auto.driveBackwards"));
    NamedCommands.registerCommand("driveForwards", drivebase.driveForward().withTimeout(1).withName("Auto.driveForwards"));
  
    



    // without superstructure
    NamedCommands.registerCommand("deployIntake", intake.deployAndRollCommand().withName("Auto.deployIntake"));
    NamedCommands.registerCommand("reverseIntake", intake.backFeedAndRollCommand().withName("Auto.reverseIntake"));

    NamedCommands.registerCommand("hopperFeed", hopper.feedCommand().withName("Auto.hopperFeed"));
    NamedCommands.registerCommand("hopperReverse", hopper.backFeedCommand().withName("Auto.hopperEject"));

    NamedCommands.registerCommand("feederFeed", feeder.feedCommand().withName("Auto.feederFeed"));

    NamedCommands.registerCommand("turretCenter", turret.center().withName("Auto.turretCenter"));
    NamedCommands.registerCommand("reverseIntake", intake.backFeedAndRollCommand().withName("Auto.reverseIntake"));
  }
}
