// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDS.LowLevelCMDS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.Enums.WristState;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristState extends InstantCommand {
  Claw claw;

  WristState state;

  /**
   * Instant command which sets the wrist state then instantly ends
   * @param claw Claw subsystem
   * @param setState State to set the wrist to
   */
  public SetWristState(Claw claw, WristState setState) {
    this.claw = claw;

    this.state = setState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setWristState(state);
  }
}
