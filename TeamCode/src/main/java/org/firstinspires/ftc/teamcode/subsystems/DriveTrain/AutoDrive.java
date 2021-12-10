package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

public class AutoDrive extends CommandBase {

    DriveTrain driveTrain;
    double distance;
    boolean isSet = false;

    public AutoDrive(DriveTrain driveTrain, double distance) {
        this.driveTrain = driveTrain;
        this.distance = distance;
    }

    @Override
    public void execute() {
        if(isSet) return;
        driveTrain.autoDrive(distance);
        isSet = true;
    }

}
