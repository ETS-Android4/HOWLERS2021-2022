 /* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode.teleop;


 import com.acmerobotics.dashboard.FtcDashboard;
 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.arcrobotics.ftclib.command.Command;
 import com.arcrobotics.ftclib.command.CommandScheduler;
 import com.arcrobotics.ftclib.command.ConditionalCommand;
 import com.arcrobotics.ftclib.command.InstantCommand;
 import com.arcrobotics.ftclib.command.SequentialCommandGroup;
 import com.arcrobotics.ftclib.gamepad.GamepadEx;
 import com.arcrobotics.ftclib.gamepad.GamepadKeys;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.util.ElapsedTime;


 import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DefaultDrive;
 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.CloseClaw;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.Lift;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftArm;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.OpenClaw;

 import java.util.function.DoubleSupplier;

 @TeleOp(name="Herberger Drive", group="Iterative Opmode")
 public class HerbergerDrive extends OpMode
 {
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     Robot robot;

     GamepadEx driverOp = null;
     GamepadEx toolOp = null;

     @Override
     public void init() {
         robot = robot.getInstance();
         robot.init(hardwareMap, DriveTrain.DriveMode.MANUAL);

         //Gamepad Initialization
         driverOp = new GamepadEx(gamepad1);
         toolOp = new GamepadEx(gamepad2);

         robot.claw.setInverted(true);

         // Tell the driver that initialization is complete.
         telemetry.addData("Status", "Initialized");

     }

     @Override
     public void init_loop() {
     }

     @Override
     public void start() {
         runtime.reset();
         robot.claw.setPosition(0.6);
         robot.liftArm.liftPID.reset();
         robot.liftArm.setHeight(LiftHeight.ZERO);

         new DefaultDrive(robot.driveTrain, () -> driverOp.getLeftY() * robot.driveTrain.getSpeed(), () -> driverOp.getRightX()).schedule();
     }

     @Override
     public void loop() {
         CommandScheduler scheduler = CommandScheduler.getInstance();
         TelemetryPacket packet = new TelemetryPacket();

         driverOp.getGamepadButton(GamepadKeys.Button.A)
                 .whenPressed(new SequentialCommandGroup(
                         new OpenClaw(robot.liftArm),
                         new Lift(robot.liftArm, LiftHeight.PICKUP),
                         new CloseClaw(robot.liftArm),
                         new Lift(robot.liftArm, LiftHeight.ZERO)
                 ));
         driverOp.getGamepadButton(GamepadKeys.Button.B)
                 .whenPressed(new InstantCommand(robot.driveTrain::slow, robot.driveTrain))
                 .whenReleased(new InstantCommand(robot.driveTrain::fast, robot.driveTrain));

         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.ZERO));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.BOTTOM));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.MIDDLE));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.TOP));
         toolOp.getGamepadButton(GamepadKeys.Button.B)
                 .whenPressed(new InstantCommand(robot.duckWheel::run, robot.duckWheel))
                 .whenReleased(new InstantCommand(robot.duckWheel::stop, robot.duckWheel));
         toolOp.getGamepadButton(GamepadKeys.Button.A)
                 .whenPressed(new OpenClaw(robot.liftArm));
         toolOp.getGamepadButton(GamepadKeys.Button.Y)
                 .whenPressed(new InstantCommand(robot.duckWheel::runInverted, robot.duckWheel))
                 .whenReleased(new InstantCommand(robot.duckWheel::stop, robot.duckWheel));

         scheduler.run();
         packet.put("liftPID Error",robot.liftArm.currentError);
         packet.put("liftPID Target", robot.liftArm.getPIDTarget());
         packet.put("lift Position", robot.lift.getEncoderCount());
         packet.put("0", 0);
         packet.put("5000", 0);
         FtcDashboard.getInstance().sendTelemetryPacket(packet);
     }

     public void driveTrainController() {
         double speed = robot.driveTrain.getSpeed();
         double turn = driverOp.getLeftX() * -speed;
         double forward = driverOp.getLeftY() * speed;
         robot.driveTrain.drive(-forward, turn);
     }

     /*
      * Code to run ONCE after the driver hits STOP
      */
     @Override
     public void stop() {

         robot.driveTrain.stop();
         robot.duckMotor.stopMotor();
         robot.lift.stopMotor();

     }


 }