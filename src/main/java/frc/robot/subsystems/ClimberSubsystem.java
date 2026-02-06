package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    // variables go here
    // TODO: put real motor id
    private SparkFlex climbMotor = new SparkFlex(0, MotorType.kBrushless);

    //constructor. run initialization here.
    public ClimberSubsystem(){

    }

    //put code that needs to run continuously here
    @Override
    public void periodic() {
        
    }
}
