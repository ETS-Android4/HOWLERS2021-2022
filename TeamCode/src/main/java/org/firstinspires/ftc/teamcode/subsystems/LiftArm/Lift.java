package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.CommandBase;

public class Lift extends CommandBase {

    private final LiftArm liftArm;
    private final LiftHeight height;

    public Lift(LiftArm liftArm, LiftHeight height) {
        this.liftArm = liftArm;
        this.height = height;
        addRequirements(liftArm);
    }

    @Override
    public void execute() {
        liftArm.setHeight(height);
    }

    @Override
    public boolean isFinished() {
        return liftArm.liftPID.atSetPoint();
    }
}
