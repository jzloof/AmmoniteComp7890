// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.ShooterFull;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Hopper.Pivot;
import frc.robot.subsystems.Hopper.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Shooter.Feeder;
import frc.robot.subsystems.Hopper.Intake;
import frc.robot.commands.LocalSwerve;
import frc.robot.commands.TrenchShotAuto;
// import frc.robot.commands.PhotonDrive;
import com.pathplanner.lib.events.EventTrigger;


public class RobotContainer {

    CommandXboxController xboxDriver = new CommandXboxController(0);

    private final Supplier<Double> translationAxis = xboxDriver::getLeftY;
    private final Supplier<Double> strafeAxis = xboxDriver::getLeftX;
    private final Supplier<Double> rotationAxis = xboxDriver::getRightX;
    private double MaxSpeed = 1.0 * Constants.Swerve.maxSpeed;


    // Make a swerve subsystem
    private final Swerve s_Swerve = new Swerve();

    // === SUBSYSTEM OBJECTS === \\
    private final Shooter objShooter = new Shooter();
    private final Feeder objFeeder = new Feeder();
    private final Indexer objIndexer = new Indexer();
    private final Intake objIntake = new Intake();
    private final Pivot objPivot = new Pivot();
    private final Field2d field;
  
    public Swerve getSwerve() {
        return s_Swerve;
    }

    // === PathPlanner === \\
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        NamedCommands.registerCommand("stop Shooter", getAutonomousCommand());
        NamedCommands.registerCommand("Intake Pivot", getAutonomousCommand());
        NamedCommands.registerCommand("Print Message", getAutonomousCommand());
        NamedCommands.registerCommand("Run Intake", getAutonomousCommand());
        NamedCommands.registerCommand("stop Shooter", new ShooterFull(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Run Intake", new RunCommand(()-> objIntake.runIntake(MaxSpeed)).withTimeout(5.0));
        NamedCommands.registerCommand("Auto shots", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        NamedCommands.registerCommand("Auto Shoot", new TrenchShotAuto(objShooter, MotorSpeeds.dShooter3M, objFeeder, objIndexer, objIntake, objPivot));
        
        new EventTrigger("Run Intake").whileTrue(new RunCommand(()-> objIntake.runIntake(MotorSpeeds.dIntakeSpeed), objIntake));
        //new EventTrigger("Auto Shoot").whileTrue(new RunCommand(()-> objShooter.runShooterRPM(MotorSpeeds.dShooter3M), objShooter));
        new EventTrigger("Intake Pivot").whileTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));
        new EventTrigger("Shooter Run").whileTrue(new ShooterFull(objShooter, 2500, objFeeder, objIndexer, objIntake, objPivot));
        new EventTrigger("Shoot Slower").whileTrue(new ShooterFull(objShooter, 3300, objFeeder, objIndexer, objIntake, objPivot));

        configureBindings();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        field = new Field2d();
            SmartDashboard.putData("Field", field);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    () -> -translationAxis.get(),
                    () -> -strafeAxis.get(),
                    () -> -rotationAxis.get(),
                    () -> false,
                    () -> false
            )
        );
   
        objFeeder.setDefaultCommand(
            new RunCommand(()-> objFeeder.stopFeeder(), objFeeder)
        );

        objIndexer.setDefaultCommand(
            new RunCommand(()->objIndexer.stopIndexer(), objIndexer)
        );

        objPivot.setDefaultCommand(
            new RunCommand(() -> objPivot.stopPivot(), objPivot)
        );

        objIntake.setDefaultCommand(
            new RunCommand(() -> objIntake.stopIntake(), objIntake)
        );

    }

    private void configureBindings() {

        // === OFFICIAL CONTROLS === \\
        // Left bumper:         Shoot
        // Right bumper:        Intake
        // A button:            Pivot intake
        // DPAD down:           Reverse intake
        // DPAD up:             Reverse feeder

        /*  State machine description for shooting
            1) spool up shooter
            2) run feeder
            3) run indexer
            4) bump in intake
        */

        xboxDriver.rightBumper().whileTrue(
            Commands.sequence(
                Commands.parallel(
                    Commands.run(() -> new LocalSwerve(s_Swerve, s_Swerve.getTargetAngle()).withTimeout(0.5).schedule()),
                    Commands.run(() -> objShooter.runShooter(s_Swerve.getShotSpeed()))
                ),
                Commands.run(() -> new WaitCommand(0.5)),
                Commands.parallel(
                    Commands.run(() -> objFeeder.runFeeder(Constants.MotorSpeeds.dFeederSpeed)),
                    Commands.run(() -> objIndexer.runIndexer(Constants.MotorSpeeds.dIndexerSpeed)),
                    Commands.run(() -> objPivot.agitatePivot())
                )
            )
        )
        .onFalse(
            Commands.parallel(
                Commands.run(() -> objShooter.stopShooter()),
                Commands.run(() -> objFeeder.stopFeeder()),
                Commands.run(() -> objIndexer.stopIndexer()),
                Commands.run(() -> objPivot.stopPivot())
            )
        );

        // Intake control
        xboxDriver.leftBumper().whileTrue(
            Commands.run(() -> objIntake.runIntake(Constants.MotorSpeeds.dIntakeSpeed))  
        )
        .onFalse(
            Commands.run(() -> objIntake.stopIntake())
        );

        // === Intake === \\ 

        xboxDriver.a().toggleOnTrue(new PivotIntake(objPivot, MotorSpeeds.dPivotSpeed));

        xboxDriver.povDown().whileTrue(new RunCommand(
                () -> objIntake.runIntake(-MotorSpeeds.dIntakeSpeed), objIntake));

        xboxDriver.povUp().whileTrue(new RunCommand(
                () -> objFeeder.runFeeder(-MotorSpeeds.dFeederSpeed), objFeeder));
             
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
