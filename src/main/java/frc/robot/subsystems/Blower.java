
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blower extends SubsystemBase {
    TalonFX blower_motor;
    double motor_speed = 0.2;
    int device_id = 0;

    private static Blower mInstance;
    public static Blower getInstance() {
        if (mInstance == null) {
            mInstance = new Blower();
        }
        return mInstance;
    }

    private Blower(){
        blower_motor = new TalonFX(device_id);
    }

    public void SetMotorSpeed(double newSpeed) {
        blower_motor.set(newSpeed);
    }
}

