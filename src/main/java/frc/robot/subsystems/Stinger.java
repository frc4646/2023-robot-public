package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.team4646.SmartSubsystem;

public class Stinger extends SmartSubsystem {
    final Servo flipperServo;
    final double STARTING_ANGLE = 180.0;
    final double FINAL_ANGLE = 0.0;
    
    public Stinger() {
        flipperServo = new Servo(0);
    }

    public void flip() {
        flipperServo.set(FINAL_ANGLE);
    }

    public void reset() {
        flipperServo.set(STARTING_ANGLE);
    }
}
