//This code is unused, stop looking here >:c

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blower;

public class Spin_blow_motor extends Command {
    private Blower s_Blower;
    private DoubleSupplier left_joystick;

    public Spin_blow_motor (Blower s_Blower, DoubleSupplier left_joystick){
        this.s_Blower = s_Blower;
        this.left_joystick = left_joystick;
        addRequirements(s_Blower);

    

    }

    public void execute(){
        s_Blower.SetMotorSpeed(left_joystick.getAsDouble()); 
    }
}

