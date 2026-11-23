package frc.robot.subsystems.arm;

public interface ArmIO {

    public class ArmIOInputs{

    }

    public default void setVoltage(double volts){}

    public default void updateInputs(double inputs){}

}
