package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.jvm.Volatile;

public class OpenClaw extends CommandBase {

    private ElapsedTime timer = new ElapsedTime();
    LiftArm liftArm;

    public OpenClaw(LiftArm liftArm) {
        this.liftArm = liftArm;
    }

    @Override
    public void initialize() {
        liftArm.openClaw();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.time() >= 0.3;
    }

}
