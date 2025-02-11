// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.moveAimerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aimerSubsystem;


public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser;

  private final aimerSubsystem AimerSubsystem = new aimerSubsystem();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final Joystick driver = new Joystick(0);
  private final JoystickButton brakebButton = new JoystickButton(driver, 3);
  private final JoystickButton pointButton = new JoystickButton(driver, 4);
  private final JoystickButton feildButton = new JoystickButton(driver, 1);
  private final JoystickButton andButton1 = new JoystickButton(driver, 8);
  private final JoystickButton andButton2 = new JoystickButton(driver, 10);
  private final JoystickButton slowBotForward = new JoystickButton(driver, 5);
    private final JoystickButton slowBotBackward = new JoystickButton(driver, 6);
    private final JoystickButton autoDriveAim = new JoystickButton(driver, 2);


  private final POVButton slowModea1 = new POVButton(driver, 0);
    private final POVButton slowModea2 = new POVButton(driver, 180);
  private final POVButton slowModea3 = new POVButton(driver, 270);
  private final POVButton slowModea4 = new POVButton(driver, 90);







  private final Joystick operator = new Joystick(1);
      private final JoystickButton aimup = new JoystickButton(operator, 5);
      private final JoystickButton aimdown = new JoystickButton(operator, 6);
      private final JoystickButton setaim = new JoystickButton(operator, 3);
      private final JoystickButton setaimLong = new JoystickButton(operator, 4);



        public void createFrontUsbCamera() {
    CameraServer.startAutomaticCapture(); //Camera stuff :3
  }

      
      





  


  // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
  .withDriveRequestType((DriveRequestType.OpenLoopVoltage));

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);


  private final moveAimerCommand m_moveAimerup = new moveAimerCommand(AimerSubsystem, 0.25);
  private final moveAimerCommand m_moveAimerdown = new moveAimerCommand(AimerSubsystem, -0.25);


  private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle();

  private final PhoenixPIDController autoTurnPID = new PhoenixPIDController(3.2, 0, 0.2);

  
  

  private void configureBindings() {

    autoAim.HeadingController = autoTurnPID;
    autoAim.HeadingController.enableContinuousInput(-180, 180);

    



    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(driver.getY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(driver.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    brakebButton.whileTrue(drivetrain.applyRequest(() -> brake));
    pointButton.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(driver.getY(), driver.getX()))));

    // reset the field-centric heading on left bumper press
    feildButton.onFalse(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));


    slowModea1.whileTrue(drivetrain.applyRequest(()-> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    slowModea2.whileTrue(drivetrain.applyRequest(()-> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    slowModea3.whileTrue(drivetrain.applyRequest(()-> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
    slowModea4.whileTrue(drivetrain.applyRequest(()-> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    slowBotForward.and(andButton1).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    slowBotBackward.and(andButton1).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    slowBotForward.and(andButton2).whileTrue(drivetrain.sysIdQuaistatic(Direction.kForward));
    slowBotBackward.and(andButton2).whileTrue(drivetrain.sysIdQuaistatic(Direction.kReverse));

    aimup.whileTrue(m_moveAimerup);
    aimdown.whileTrue(m_moveAimerdown);



   new Trigger(setaim).onTrue(new InstantCommand(AimerSubsystem::calcAngle));
   new Trigger(setaimLong).onTrue(new InstantCommand(AimerSubsystem::posLong));

   autoDriveAim.whileTrue( // Auto target for driver
        drivetrain.applyRequest(() -> autoAim.withTargetDirection(drivetrain.directionToGoal())
            .withVelocityX(-driver.getY() * MaxSpeed)
            .withVelocityY(-driver.getX() * MaxSpeed)

        )
    );

    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("CalcAngle", new InstantCommand(AimerSubsystem::calcAngle));

    NamedCommands.registerCommand("driveAimSpeaker", drivetrain.applyRequest(() -> autoAim.withTargetDirection(drivetrain.directionToGoal())
    .withVelocityX(driver.getY() * MaxSpeed)
    .withVelocityY(driver.getX() * MaxSpeed)));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    createFrontUsbCamera();


  }

  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();
  }
}
