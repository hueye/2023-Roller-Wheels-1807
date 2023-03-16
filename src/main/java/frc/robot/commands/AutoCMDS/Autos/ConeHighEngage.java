// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.commands.ArmCMDS.*;
import frc.robot.commands.AutoCMDS.AutoLevel;
import frc.robot.commands.AutoCMDS.FollowPath;
import frc.robot.commands.AutoCMDS.ResetOdometrytoTrajectory;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class ConeHighEngage extends SequentialCommandGroup {
    public ConeHighEngage(DriveTrain driveSubsystem, Arm armSubsystem, Claw clawSubsystem, RobotContainer robotContainer)
    {
      addCommands(
        new SetClawState(clawSubsystem, ClawState.Closed),
        new AutoPlace(armSubsystem, clawSubsystem, 180.182),
        new ResetOdometrytoTrajectory("ConeHighEngage", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("ConeHighEngage", 3, 3, driveSubsystem).getCommand(),
          new ResetArm(robotContainer)),
        new AutoLevel(driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.setX(), driveSubsystem));
    }
}
