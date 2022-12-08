package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.intel.realsense.librealsense.Config;
import com.intel.realsense.librealsense.StreamType;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.exceptions.NoObjectFoundException;

import kotlin.Triple;
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

  private DistanceSensor dist;
  private ConfigurableRealSenseCamera camera = null;

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

  boolean goingUp;
  int previousLiftPosition;
  int liftPosition;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    try {
      camera = new ConfigurableRealSenseCamera(hardwareMap, () -> isStopRequested());
      dist = hardwareMap.get(DistanceSensor.class, "dist");
      ElapsedTime delay;

      liftPosition = 1;
      previousLiftPosition = 0;
      goingUp = true;

      drive = new SampleMecanumDrive(hardwareMap);

      lift = hardwareMap.get(DcMotor.class, "lift");
      claw = hardwareMap.get(Servo.class, "claw");


      //INFO Previous f was 12.411
      ((DcMotorEx) lift).setVelocityPIDFCoefficients(0, 0, 0, 12.411);
      //INFO Previous P was 15
      ((DcMotorEx) lift).setPositionPIDFCoefficients(10);
      // Put initialization blocks here.

      circumferenceInInches = 4.409;
      encoderTicksPerRotation = 384.5;

      lift.setDirection(DcMotorSimple.Direction.REVERSE);
      lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      lift.setPower(-0.5);

      delay = new ElapsedTime();

      while (delay.seconds() < 1) {
        telemetry.addData("key", delay);
        telemetry.update();
      }

      Config findCone = new Config();

      findCone.enableStream(StreamType.COLOR);
      findCone.enableStream(StreamType.DEPTH);
      findCone.enableStream(StreamType.INFRARED);

      camera.switchConfig(findCone);

      lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift.setPower(0.3);
      lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      waitForStart();

      if (opModeIsActive()) {
        while (opModeIsActive()) {
          drive.updatePoseEstimate();
          // Put loop blocks here.

          if (gamepad1.left_bumper) {
            if (!rightTriggerAlreadyPressed) {
              liftPosition += 1;
              rightTriggerAlreadyPressed = true;
            }
          } else rightTriggerAlreadyPressed = false;

          if (gamepad1.right_bumper) {
            if (!leftTriggerAlreadyPressed) {
              liftPosition -= 1;
              leftTriggerAlreadyPressed = true;
            }
          } else leftTriggerAlreadyPressed = false;

          if (gamepad1.dpad_up) liftPosition    = 5;
          if (gamepad1.dpad_right) liftPosition = 4;
          if (gamepad1.dpad_down) liftPosition  = 3;
          if (gamepad1.dpad_left) liftPosition  = 2;
          if (gamepad1.a) liftPosition          = 1;
          if (gamepad1.y) liftPosition          = 0;

          if (liftPosition < 0) liftPosition = 0;
          if (liftPosition > 5) liftPosition = 5;

          if(goingUp){
            if(lift.getCurrentPosition() > ((7.5 / circumferenceInInches) * encoderTicksPerRotation)){
              lift.setPower(1.0);
            } else {
              lift.setPower(0.5);
            }
          } else {
            if (lift.getCurrentPosition() > ((15 / circumferenceInInches) * encoderTicksPerRotation)) {
              lift.setPower(0.1);
            } else {
              if (liftPosition == 0) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(-0.2);
              } else lift.setPower(0.15);
            }
          }

          if (liftPosition == 0) moveLift(0.00);
          if (liftPosition == 1) moveLift(2.75);
          if (liftPosition == 2) moveLift(8.00);
          if (liftPosition == 3) moveLift(14.5);
          if (liftPosition == 4) moveLift(24.5);
          if (liftPosition == 5) moveLift(34.5);

          if (liftPosition < previousLiftPosition) goingUp = false;
          if (liftPosition > previousLiftPosition) goingUp = true ;
          previousLiftPosition = liftPosition;

          if ((gamepad1.right_trigger > 0.5)) claw.setPosition(30.0 / 190); // In pressed pos
          else claw.setPosition(12.0 / 190); // In release pos

          if (gamepad1.left_trigger > 0.5 || gamepad1.b) {
            requestedLinearXTranslation = gamepad1.left_stick_x  * 0.25f;
            requestedLinearYTranslation = gamepad1.left_stick_y  * 0.25f;
            requestedRadialTranslation  = gamepad1.right_stick_x * 0.25f;
          } else if (liftPosition == 5) {
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
            if (!camera.updateFrameSet()) continue;
            FrameData data = camera.getImageFrame(StreamType.DEPTH);
            int scanlineY = (int) (data.getHeight() * 0.75);

            int nearestPointGuess;


            Pair<Float, Float>[] coneColourRanges = new Pair[]{
                    new Pair<>(220f, 260f),
                    new Pair<>(320f, 10f ) };

            Triple<Integer, Integer, Float> positionAndHueRangeIndex;

            try {
              positionAndHueRangeIndex = getNearestObjectWithHueRange(scanlineY, coneColourRanges);
            } catch (NoObjectFoundException e) {
              continue;
            }

            nearestPointGuess = positionAndHueRangeIndex.getFirst();
            float depth = positionAndHueRangeIndex.getThird();


            ObjectExtentParameters extentParameters = null;
            try {
              extentParameters = findObjectExtentUsingHueRanges(nearestPointGuess, scanlineY, coneColourRanges, false);
            } catch (NoObjectFoundException e) {
              continue;
            }

            int rightSideCone = extentParameters.getRightSide();
            int leftSideCone = extentParameters.getLeftSide();
            int middleCone = extentParameters.getMiddle();
            int coneWidth = extentParameters.getWidth();

            camera.drawHorizontalLine((int)(data.getHeight()*.05), true);

            System.out.println("CONEWIDTH: " + coneWidth);
            int distanceFromMiddle = data.getWidth() / 2 - middleCone - 40;
            double degreesPerPixel = 90.0 / data.getWidth();

            // INFO: We can't see a cone out that far.
            if(coneWidth < 90) continue;

            camera.drawHorizontalLine(scanlineY, true);
            camera.drawHorizontalLine((int)(data.getHeight()*.05), true);

            //DISABLED:
            // camera.drawVerticalLine(middleCone);
            // camera.drawVerticalLine(leftSideCone);
            // camera.drawVerticalLine(rightSideCone);
            // camera.drawVerticalLine(nearestPointGuess);
            System.out.println("distance to detected cone point: "+depth);

            camera.transmitMonochromeImage();

            double turnAngle = Math.toRadians(degreesPerPixel * distanceFromMiddle);

            drive.turn(turnAngle);
            System.out.println(-Math.toRadians(degreesPerPixel * distanceFromMiddle));

            double[] distances = new double[5];

            for(int i1 = 1; i1 <= 5; i1++){
              distances[i1 - 1] = dist.getDistance(DistanceUnit.INCH);
            }

            double averageDist = 0;

            for(double dist : distances){
              averageDist += dist;
            }

            averageDist /= distances.length;

            drive.updatePoseEstimate();

            if(Math.abs(turnAngle) < 10) {
              drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).forward(averageDist - 3.34646).build());
            }
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
            DisconnectedCameraException     |
            CameraStopException             |
            InterruptedException            |
            CameraStartException e) {
      e.printStackTrace();
    } finally {
      try {
        camera.close();
      } catch (FrameQueueCloseException e) {
        e.printStackTrace();
      } catch (CameraStopException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Describe this function....
   */
  private void moveLift(double liftDistance) {
    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lift.setTargetPosition((int) ((liftDistance / circumferenceInInches) * encoderTicksPerRotation));
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

  private Triple<Integer, Integer, Float> getNearestObjectWithHueRange(int scanlineY, Pair<Float, Float>[] hueRanges) throws NoFrameSetYetAcquiredException, UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoObjectFoundException {
    int i;
    FrameData data = camera.getImageFrame(StreamType.DEPTH);
    int middleCone = -1;
    float depth = 100000000;
    int x = -1;

    float[] hsv = new float[3];

    int associatedColourRange = -1;

    int avgCount=0;
    for (i = (int) (data.getWidth() * 0.2); i < data.getWidth() * 0.8; i++) {
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);

      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];

      float newDepth=camera.getDistance(i, scanlineY);
      if (newDepth != 0 && newDepth <= depth) {
        int m = 0;
        for (Pair<Float, Float> hueRange : hueRanges) {
          if (hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) {
            x = i;
            depth = newDepth;
            associatedColourRange = m;
          }
          m++;
        }
      }
    }

    if(associatedColourRange == -1) throw new NoObjectFoundException();
    return new Triple<>(x, associatedColourRange, depth);
  }

  private ObjectExtentParameters findObjectExtentUsingHueRanges(int startX, int scanlineY, Pair<Float, Float>[] hueRanges, boolean debug) throws NoFrameSetYetAcquiredException, UnsupportedStreamTypeException, StreamTypeNotEnabledException, NoObjectFoundException {
    FrameData data = camera.getImageFrame(StreamType.COLOR);

    int i;
    float[] hsv = new float[3];

    int notConeTolerance = 5;
    int notConeCountdown = notConeTolerance;

    StringBuilder hueString = null;
    StringBuilder satString = null;
    StringBuilder valString = null;

    int leftSideCone  = -1;
    int rightSideCone = -1;
    int coneWidth     = -1;
    int middleCone    = -1;

    if(debug){
      hueString = new StringBuilder("HUE: ");
      satString = new StringBuilder("SAT: ");
      valString = new StringBuilder("VAL: ");
    }

    boolean hueFound;

    for(i = startX; i >= 0; i-=2){
      hueFound = false;
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];
      for (Pair<Float, Float> hueRange: hueRanges) {
        if (hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) hueFound = true;
      }
      if (hueFound){
        if(notConeCountdown == notConeTolerance){
          if(debug) {
            hueString.insert(0, String.format("%03.0f ", hue));
            satString.insert(0, String.format("%.2f ", sat).substring(1));
            valString.insert(0, String.format("%.2f ", val).substring(1));
          }

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

    if(debug){
      System.out.println(hueString + "\n" + satString + "\n" + valString);
    }

    notConeCountdown = notConeTolerance;

    if(debug){

      hueString = new StringBuilder("HUE: ");
      satString = new StringBuilder("SAT: ");
      valString = new StringBuilder("VAL: ");
    }

    for(i = startX; i < data.getWidth(); i+=2){
      hueFound = false;
      Color.colorToHSV(camera.getARGB(i, scanlineY), hsv);
      double hue = hsv[0];
      double sat = hsv[1];
      double val = hsv[2];
      for (Pair<Float, Float> hueRange: hueRanges) if (hueRange(hue, hueRange.fst, hueRange.snd) && sat > 0.3) hueFound = true;
      if (hueFound) {
        if(notConeCountdown == notConeTolerance){
          if(debug) {
            hueString.append(String.format("%03.0f ", hue));
            satString.append(String.format("%.2f ", sat).substring(1));
            valString.append(String.format("%.2f ", val).substring(1));
          }
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

    if(debug){
      System.out.println(hueString + "\n" + satString + "\n" + valString);
    }

    if(rightSideCone == -1 || leftSideCone == -1) throw new NoObjectFoundException();

    middleCone = (leftSideCone + rightSideCone) / 2;
    coneWidth  =  rightSideCone - leftSideCone;

    return new ObjectExtentParameters(leftSideCone, rightSideCone, middleCone, coneWidth);
  }

  class ObjectExtentParameters{

    private final int leftSide;
    private final int rightSide;
    private final int middle;
    private final int width;

    public ObjectExtentParameters(int leftSide, int rightSide, int middle, int width){
      this.leftSide  =  leftSide;
      this.rightSide = rightSide;
      this.middle    =    middle;
      this.width     =     width;
    }

    public  int getLeftSide(){
      return leftSide;
    }

    public int getRightSide() {
      return rightSide;
    }

    public int getMiddle() {
      return middle;
    }

    public int getWidth() {
      return width;
    }
  }
}