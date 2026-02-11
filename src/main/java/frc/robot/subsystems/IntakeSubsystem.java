package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//put code for the intake in here.
public class IntakeSubsystem extends SubsystemBase {

    // put variables here.
    // TODO: put real motor ids
    private SparkMax linkageMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkMax spinMotor = new SparkMax(0, MotorType.kBrushless);

    // the constructor. do intake initialization here.
    public IntakeSubsystem(){

    }

    // the periodic method gets called every time the command scheduler runs. put code that needs to run continuously here.
    @Override
    public void periodic() {
        
    }
}
