/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.ghghbjkkj
 * ayy lma
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor leftFrontDrive    = null;
    public DcMotor rightFrontDrive   = null;
    public DcMotor leftBackDrive     = null;
    public DcMotor rightBackDrive    = null;
    //public DcMotor blake        = null; //blake is the left intake (portmanteau)
  //  public DcMotor drake        = null; // drake is the right intake (portmanteau)
  // public BNO055IMU imu;

   // public Servo  liftandLower  = null;
    //public DcMotor midDrive = null;
    //public DcMotor  leftArm     = null;
  //  public Servo    bishop     = null;






//    public final static double bishop_HOME      = 0.0;
//    public final static double bishop_MIN_RANGE = 0.0;
//    public final static double bishop_MAX_RANGE = 5.00;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){


    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Define and Initialize Motors & Servos
       // imu = hwMap.get(BNO055IMU.class, "imu");
        leftFrontDrive  = hwMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hwMap.get(DcMotor.class, "front_right");
        leftBackDrive   = hwMap.get(DcMotor.class, "back_left");
        rightBackDrive  = hwMap.get(DcMotor.class, "back_right");
      //  blake = hwMap.get(DcMotor.class, "blake");
      //  drake = hwMap.get(DcMotor.class, "drake");

        // liftandLower = hwMap.get(Servo.class, "liftandLower");
        //mineralCollection = hwMap.get(Servo.class,"Mineral_Collection");
       // bishop = hwMap.get(Servo.class, "bishop");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
      //  blake.setDirection(DcMotor.Direction.FORWARD);
        // drake.setDirection(DcMotor.Direction.REVERSE);
       // bat.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                            //Naturally when the robot is pushed while its wheels are set to zero power, the robot
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);                           //wheels will spin. This means that another robot would be able to push the robot out
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // blake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  drake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // bat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //are given a power value of "0" then they will both stop and actively resist movement.


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
       // drake.setPower(0);
      //  blake.setPower(0);
      //  bat.setPower(0);
        //Set All the Motors Modes and the Servos Positions
                      // liftandLower.setPosition(Range.clip(0,0,1));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // blake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  drake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // bishop.setPosition(bishop_HOME);
    }
}