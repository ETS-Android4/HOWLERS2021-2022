package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

public class AutoTurn extends CommandBase {

    DriveTrain driveTrain;
    double angle;
    boolean isSet = false;

    public AutoTurn(DriveTrain driveTrain, double angle) {
        this.driveTrain = driveTrain;
        this.angle = angle;
    }

    @Override
    public void execute() {
        if(isSet) return;
        driveTrain.autoTurn(angle);
        isSet = true;
    }

}
