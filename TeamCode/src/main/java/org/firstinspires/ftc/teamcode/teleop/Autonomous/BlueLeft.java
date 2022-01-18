package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.CloseClaw;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftArm;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;


@Autonomous(name="Blue Left")
public class BlueLeft extends OpMode {

    GamepadEx driverOp;
    GamepadEx toolOp;
    SequentialCommandGroup auto;
    Robot robot;

    @Override
    public void init() {
        robot = Robot.resetInstance();
        robot.init(hardwareMap, DriveTrain.DriveMode.AUTONOMOUS);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(new RedRightAuto(robot.driveTrain, robot.liftArm, robot.duckWheel));
        telemetry.addData("Auto Status", "STARTED");
        telemetry.speak("AUTONOMOUS HAS STARTED");
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}

class BlueLeftAuto extends SequentialCommandGroup {

    DriveTrain driveTrain;
    LiftArm liftArm;
    DuckWheel duckWheel;

    BlueLeftAuto(DriveTrain driveTrain, LiftArm liftArm, DuckWheel duckWheel) {
        this.driveTrain = driveTrain;
        this.liftArm = liftArm;
        this.duckWheel = duckWheel;

        addCommands(
                new CloseClaw(liftArm),
                new Lift(liftArm, LiftHeight.ZERO),
                new AutoDrive(driveTrain, 1000),
                new Lift(liftArm, LiftHeight.PICKUP)
        );
    }

}

