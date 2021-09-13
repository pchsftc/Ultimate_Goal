package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/* Autonomous progam made by Tristan. This file is starting with @Autonomous simply
 stating this file as autonomous instead of teleop */



@Autonomous(name = "FrankAutobot", group="Autonomous")
public class FrankAutobot extends LinearOpMode {
    private HardwarePushbot lowercaseFrank = new HardwarePushbot();
    double DRIVE_POWER = .7;
     /* Fetching/stating/getting the motors.
       Right motor is stated as "right_drive", and left motor is "left_drive"*/
     public void runOpMode() throws InterruptedException {
         lowercaseFrank.init(hardwareMap);
         /* initialize motors */

         // wait for game to start.
         waitForStart();

         // Robot moving
         // The First number stands for power while the second number stands for time

         DriveForwardTime( DRIVE_POWER, 4000);
         TurnLeftTime(DRIVE_POWER, 500);
         DriveForwardTime(DRIVE_POWER, 4000);
         TurnRightTime(DRIVE_POWER, 500);
         DriveForwardTime(DRIVE_POWER,4000);
         StopDriving();

     }

/* Establishing the motors*/




    public void DriveForward(double power) {
       // lowercaseFrank.leftDrive.setPower(power);
      //  lowercaseFrank.rightDrive.setPower(power);
    }

    public void DriveForwardTime(double power, long naptime) throws InterruptedException {
        DriveForward(power);
        sleep(naptime);
        DriveForward(0);
    }


    public void TurnLeft(double power) {
      //  lowercaseFrank.leftDrive.setPower(-power);
        //lowercaseFrank.rightDrive.setPower(power);
    }

    public void TurnRight(double power) {
        TurnLeft(-power);
    }

    public void TurnLeftTime(double power, long naptime) throws InterruptedException {
        TurnLeft(power);
        sleep(naptime);
        DriveForward(0);
    }


    public void TurnRightTime(double power, long naptime) throws InterruptedException {
        TurnRight(power);
        sleep(naptime);
        DriveForward(0);
    }

    public void StopDriving() {
        DriveForward(0);
    }

    private double Rotatetime(double degrees, double power){
        return((degrees)/((1.3646*(Math.pow(10,7))*Math.pow(power,1.47891))-(1.3645*(Math.pow(10,7))*(Math.pow(power,1.47893)))));
    }

    public void MoveServo() {

       // lowercaseFrank.bishop.setPosition(x); // x === to the decimals found on the telemetry when in the up position
        sleep(1000);
      //  lowercaseFrank.bishop.setPosition(y); // y === to the decimals found on the telemetry when in the down position
    }
}
