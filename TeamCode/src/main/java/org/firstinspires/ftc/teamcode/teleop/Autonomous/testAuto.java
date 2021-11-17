package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;


@Autonomous(name="testAuto")
public class testAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;


    @Override
    public void init()
    {
        robot = robot.resetInstance();

        robot.init(hardwareMap, DriveTrain.DriveMode.AUTONOMOUS);
        telemetry.addLine("Initialized");

        robot.liftArm.setHeight(LiftHeight.ZERO);
    }

    @Override
    public void init_loop(){}

    @Override
    public void start()
    {
        runtime.reset();
        robot.box.setPosition(0.3);

    }

    @Override
    public void loop()
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop()
    {
        robot.driveTrain.stop();
        robot.lift.set(0);
        robot.intakeMotor.set(0);



    }

}




