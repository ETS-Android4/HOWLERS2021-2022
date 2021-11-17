package org.firstinspires.ftc.teamcode.subsystems.DuckWheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.motors.HerbergerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class DuckWheel extends SubsystemBase {

    public DuckWheel(HardwareMap hwMap) {
        Robot robot = Robot.getInstance();
        robot.duckServo = new Motor(hwMap, "Duck Wheel", Motor.GoBILDA.RPM_1150);
    }

}
