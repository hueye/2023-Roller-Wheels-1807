package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.SpindexerConstants;

public class Roller extends SubsystemBase {

    private CANSparkMax RollerMotor = new CANSparkMax(SpindexerConstants.ROLLER_WHEEL_CAN_ID, MotorType.kBrushless);

    RelativeEncoder RollEncoder;
    
  /** Creates a new Roller. */
  public Roller() {
      RollerMotor.setIdleMode(IdleMode.kBrake);

    RollEncoder = RollerMotor.getEncoder();
    RollEncoder.setPosition(0);
  }
 
  public void Roll(){
    RollerMotor.set(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
