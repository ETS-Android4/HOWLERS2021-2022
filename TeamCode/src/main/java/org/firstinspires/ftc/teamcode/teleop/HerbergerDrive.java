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


 import com.arcrobotics.ftclib.gamepad.GamepadEx;
 import com.arcrobotics.ftclib.gamepad.GamepadKeys;
 import com.arcrobotics.ftclib.hardware.motors.Motor;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.util.ElapsedTime;


 import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm;

 @TeleOp(name="HerbergerDrive", group="Iterative Opmode")
 public class HerbergerDrive extends OpMode
 {
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     Robot robot;

     //BasicDrive basicDrive;
     //ManualTurretController manualTurretController;

     GamepadEx driverOp = null;
     GamepadEx toolOp = null;
     boolean rightButtonPressed = false;





     /*
      * Code to run ONCE when the driver hits INIT
      */
     @Override
     public void init() {
         robot = robot.resetInstance();
         robot.init(hardwareMap, DriveTrain.DriveMode.MANUAL);

         //Gamepad Initialization
         driverOp = new GamepadEx(gamepad1);
         toolOp = new GamepadEx(gamepad2);

         robot.box.setInverted(true);


         // Tell the driver that initialization is complete.
         telemetry.addData("Status", "Initialized");

     }

     /*
      * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
      */
     @Override
     public void init_loop() {
     }

     /*
      * Code to run ONCE when the driver hits PLAY
      */
     @Override
     public void start() {
         runtime.reset();
         robot.box.setPosition(0.3);
         robot.liftArm.setHeight(0);
     }


     /*
      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
      */

     @Override
     public void loop() {


         driveTrainController();
         robot.liftArm.liftController();
         liftArm();
         boxFlip();
         duckWheel();
         intake(0.5);


     }


     public void driveTrainController() {
         double speed = robot.driveTrain.getSpeed();
         double turn = driverOp.getLeftX() * -speed;
         double forward = driverOp.getLeftY() * speed;
         robot.driveTrain.drive(forward, turn);
     }


     public void liftArm() {
         if(toolOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) robot.liftArm.liftHeight = LiftArm.LiftHeight.ZERO;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_DOWN)) robot.liftArm.liftHeight = LiftArm.LiftHeight.BOTTOM;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_LEFT)) robot.liftArm.liftHeight = LiftArm.LiftHeight.MIDDLE;
         else if(toolOp.getButton(GamepadKeys.Button.DPAD_UP)) robot.liftArm.liftHeight = LiftArm.LiftHeight.TOP;
     }

     public void boxFlip()
     {
         boolean aButton = toolOp.getButton(GamepadKeys.Button.A);
         boolean bButton = toolOp.getButton(GamepadKeys.Button.B);
         boolean xButton = toolOp.getButton(GamepadKeys.Button.X);

         if(aButton)
         {
             robot.box.setPosition(0.73);

         }
         if(bButton)
         {
             robot.box.setPosition(0.3);

         }
         if(xButton)
         {
             robot.box.setPosition(0.35);

         }

     }


     public void duckWheel()
     {
         boolean rightBumper = toolOp.getButton(GamepadKeys.Button.RIGHT_BUMPER);
         boolean leftBumper = toolOp.getButton(GamepadKeys.Button.LEFT_BUMPER);

         if(rightBumper)
         {
             robot.duckWheel.set(1);

         }else if(leftBumper)
             {
                 robot.duckWheel.set(-1);
             }else
                 {
                     robot.duckWheel.set(0);
                 }



     }
     public void intake(double speed)
     {
         double intakeValue = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * speed;
         double spitOut = driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * speed;

         if(intakeValue > 0.05 )
         {
             robot.intake.set(intakeValue);

         }else if(spitOut > 0.05)
         {
             robot.intake.set(-spitOut);
         }else
             {
                 robot.intake.set(0);
             }

     }





     /*
      * Code to run ONCE after the driver hits STOP
      */
     @Override
     public void stop() {


         robot.driveTrain.stop();
         robot.duckWheel.stop();
         robot.lift.stopMotor();


     }


 }