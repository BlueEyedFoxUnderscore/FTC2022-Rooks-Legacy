package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.ColorSpace;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.StreamType;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import ma.phoenix.ftc.realsensecamera.ConfigurableRealSenseCamera;
import ma.phoenix.ftc.realsensecamera.FrameData;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStartException;
import ma.phoenix.ftc.realsensecamera.exceptions.CameraStopException;
import ma.phoenix.ftc.realsensecamera.exceptions.DisconnectedCameraException;
import ma.phoenix.ftc.realsensecamera.exceptions.FrameQueueCloseException;
import ma.phoenix.ftc.realsensecamera.exceptions.NoFrameSetYetAcquiredException;
import ma.phoenix.ftc.realsensecamera.exceptions.StreamTypeNotEnabledException;
import ma.phoenix.ftc.realsensecamera.exceptions.UnsupportedStreamTypeException;

@TeleOp(name = "Rooks Legacy Smart Move")
public class RooksLegacySmartMove extends LinearOpMode {

  private DcMotor lift;
  private Servo claw;
  private SampleMecanumDrive drive;

  float requestedLinearXTranslation;
  float requestedLinearYTranslation;
  double requestedRadialTranslation;

  double encoderTicksPerRotation;
  double circumferenceInInches;

  boolean rightTriggerAlreadyPressed = false;
  boolean leftTriggerAlreadyPressed  = false;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    try (ConfigurableRealSenseCamera camera = new ConfigurableRealSenseCamera(hardwareMap, () -> isStopRequested())) {

      ElapsedTime delay;
      int liftPosition;
      drive = new SampleMecanumDrive(hardwareMap);

      lift = hardwareMap.get(DcMotor.class, "lift");
      claw = hardwareMap.get(Servo.class, "claw");

      ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 12.411);
      ((DcMotorEx) lift).setPositionPIDFCoefficients(15);
      // Put initialization blocks here.

      circumferenceInInches = 4.409;
      encoderTicksPerRotation = 384.5;

      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lift.setPower(-0.5);

      delay = new ElapsedTime();

      while (delay.seconds() < 0.5) {
        telemetry.addData("key", delay);
        telemetry.update();
      }

      Config findCone = new Config();

      findCone.enableStream(StreamType.COLOR);
      findCone.enableStream(StreamType.DEPTH);
      findCone.enableStream(StreamType.INFRARED);

      camera.switchConfig(findCone);

      lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift.setPower(0.3);

      liftPosition = 0;
      waitForStart();

      if (opModeIsActive()) {
        while (opModeIsActive()) {
          drive.updatePoseEstimate();
          // Put loop blocks here.
          if (gamepad1.right_bumper) {
            if (!rightTriggerAlreadyPressed) {
              liftPosition += 1;
              rightTriggerAlreadyPressed = true;
            }
          }
          else rightTriggerAlreadyPressed = false;

          if (gamepad1.left_bumper)  {
            if(!leftTriggerAlreadyPressed){
              liftPosition -= 1;
              leftTriggerAlreadyPressed = true;
            }
          }
          else leftTriggerAlreadyPressed  = false;

          if (gamepad1.dpad_up)    liftPosition = 4;
          if (gamepad1.dpad_right) liftPosition = 3;
          if (gamepad1.dpad_down)  liftPosition = 2;
          if (gamepad1.dpad_left)  liftPosition = 1;
          if (gamepad1.y)          liftPosition = 0;

          if (liftPosition < 0) liftPosition = 0;
          if (liftPosition > 4) liftPosition = 4;

          if (liftPosition < 2){
            lift.setPower(0.5);
          } else {
            lift.setPower(1);
          }

          if (liftPosition == 0) moveLift(0.00);
          if (liftPosition == 1) moveLift(2.75);
          if (liftPosition == 2) moveLift(14.5);
          if (liftPosition == 3) moveLift(24.5);
          if (liftPosition == 4) moveLift(34.5);

          if ((gamepad1.right_trigger > 0.5) || gamepad1.a) claw.setPosition(25.0 / 190); // In pressed pos
          else claw.setPosition(12.0 / 190); // In release pos

          if (gamepad1.left_trigger > 0.5 || gamepad1.b) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 0.25f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 0.25f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.25f;
          } else if (liftPosition == 3 || liftPosition == 4) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 0.5f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 0.5f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.5f;
          } else {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 1f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 1f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 1f;
          }

          //EXPERIMENTAL
          if (gamepad1.x) {
            int i;
            if (!camera.updateFrameSet()) continue;
            FrameData data = camera.getImageFrame(StreamType.DEPTH);
            int middleCone = -1;
            float depth = 100000000;
            int x = -1;
            float[] hsv = new float[3];
            int scanlineY = (int) (data.getHeight() * 0.75);
            for (i = (int) (data.getWidth() * 0.2); i < data.getWidth() * 0.8; i++) {
              Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
              double hue = hsv[0];
              double sat = hsv[1];
              double val = hsv[2];
              if (camera.getDistance(i, scanlineY) != 0 && camera.getDistance(i, scanlineY) < depth) {
                if ((hueRange(hue, 220, 260) || hueRange(hue, 320, 10)) && sat > 0.3) {
                  x = i;
                  depth = camera.getDistance(i, scanlineY);
                }
              }
            }


            int leftSideCone = -1;
            int rightSideCone = -1;
            int notConeTolerance = 5;
            int notConeCountdown = notConeTolerance;


            boolean redCone;

            {
              Color.colorToHSV(camera.getARGB(x, scanlineY), hsv);
              double hue = hsv[0];
              redCone = !hueRange(hue, 220, 260);
            }

            StringBuilder hueString = new StringBuilder();
            StringBuilder satString = new StringBuilder();
            StringBuilder valString = new StringBuilder();


            for(i = x; i >= 0; i-=2){
              Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
              double hue = hsv[0];
              double sat = hsv[1];
              double val = hsv[2];
              if(((hueRange(hue, 220, 260) && !redCone) || (hueRange(hue, 330, 10) && redCone)) && sat > 0.3) {
                if(notConeCountdown == notConeTolerance){
                  hueString.insert (0, String.format("%03.0f ", hue));
                  satString.insert (0, String.format("%.2f ", sat).substring(1));
                  valString.insert (0, String.format("%.2f ", val).substring(1));
                  leftSideCone = i;
                } else notConeCountdown++;
              }else{
                if(notConeCountdown == 0){
                  break;
                }
                assert(notConeCountdown > 0);
                notConeCountdown--;
              }
            }
            notConeCountdown = notConeTolerance;
            if(leftSideCone == -1) continue;

            for(i = x; i <= data.getWidth(); i+=2){
              Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
              double hue = hsv[0];
              double sat = hsv[1];
              double val = hsv[2];
              if(((hueRange(hue, 220, 260) && !redCone) || (hueRange(hue, 330, 10) && redCone)) && sat > 0.3) {
                if(notConeCountdown == notConeTolerance){
                  hueString.append(String.format("%03.0f ", hue));
                  satString.append (String.format("%.2f ", sat).substring(1));
                  valString.append (String.format("%.2f ", val).substring(1));
                  rightSideCone = i;
                } else notConeCountdown++;
              }else{
                if(notConeCountdown == 0){
                  break;
                }
                assert(notConeCountdown > 0);
                notConeCountdown--;
              }
            }

            hueString.insert (0, "Hue: ");
            satString.insert (0, "Sat: ");
            valString.insert (0, "Val: ");
            System.out.println(hueString);
            System.out.println(satString);
            System.out.println(valString);

            if(rightSideCone == -1) continue;
            middleCone = (leftSideCone + rightSideCone) / 2;
            int coneWidth = rightSideCone - leftSideCone;
            System.out.println("CONEWIDTH: " + coneWidth);
            int distanceFromMiddle = data.getWidth() / 2 - middleCone - 40;
            double degreesPerPixel = 90.0 / data.getWidth();

            // Best fit equation
            double pseudoDistance = 0.571 * Math.tan(Math.toRadians(-0.046 * (coneWidth - 53) + 90.0)) - 3.3;

            drive.updatePoseEstimate();
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(pseudoDistance).build());

            if(middleCone < 241){
              System.out.println("Closer than should be");
            }else{
              System.out.println("Farther than should be");
            }

            camera.drawHorizontalLine(scanlineY);
            camera.drawVerticalLine(middleCone);
            camera.drawVerticalLine(leftSideCone);
            camera.drawVerticalLine(rightSideCone);
            camera.transmitMonochromeImage();

            drive.turn(Math.toRadians(degreesPerPixel * distanceFromMiddle));
            System.out.println(-Math.toRadians(degreesPerPixel * distanceFromMiddle));
          } else {
            Vector2d requestedLinearInput = requestedLinearDriveInput();
            drive.setWeightedDrivePower(new Pose2d(requestedLinearInput.getX(), requestedLinearInput.getY(), -requestedRadialTranslation));
          }
          telemetry.addData("position of claw", Double.parseDouble(JavaUtil.formatNumber(claw.getPosition(), 2)));
          telemetry.addData("Joystick Y", Double.parseDouble(JavaUtil.formatNumber(requestedLinearYTranslation, 2)));
          telemetry.addData("Joystick X", Double.parseDouble(JavaUtil.formatNumber(requestedLinearXTranslation, 2)));
          telemetry.update();
        }
      }
    } catch (NoFrameSetYetAcquiredException |
            UnsupportedStreamTypeException  |
            StreamTypeNotEnabledException   |
            FrameQueueCloseException        |
            DisconnectedCameraException     |
            CameraStopException             |
            InterruptedException            |
            CameraStartException e) {
      e.printStackTrace();
    }
  }

  /**
   * Describe this function....
   */
  private void moveLift(double lift_distance) {
    lift.setTargetPosition((int) ((lift_distance / circumferenceInInches) * encoderTicksPerRotation));
  }

  private Vector2d requestedLinearDriveInput(){
    return new Vector2d(
            -requestedLinearYTranslation,
            -requestedLinearXTranslation
    ).rotated(-drive.getPoseEstimate().getHeading());
  }

  private boolean range(double x, double min, double max){
    return (min < x && x < max);
  }
  private boolean hueRange(double x, double from, double to){
    if(from < to) return range(x, from, to);
    return (range(x, from, 360) || range(x, 0, to));
  }
}