// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetCubeOrCone;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitForPlace extends ParallelDeadlineGroup {

  /**
   * Parallel deadline group which rotates the arm to its placement position and waits for the operator to place
   * the currently held game piece. Allows for the operator to switch between cube and cone placement angles.
   * Ends once the operator places the piece
   */
  public WaitForPlace(Arm arm, SetArmAngle angles, CommandXboxController controller) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(Commands.waitUntil(arm::getNOTHolding));

    addCommands(Commands.repeatingSequence(angles, new SetCubeOrCone(arm, controller)));
  }
}
