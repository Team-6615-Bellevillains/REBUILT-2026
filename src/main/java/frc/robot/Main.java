// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Main {
  private Main() {}


  static Translation2d centerTranslation(Pose3d left, Pose3d right, Pose3d top, Pose3d bottom) {
    return new Translation2d(
      (right.getX() - left.getX()) / 2 + left.getX(), 
      (top.getY() - bottom.getY()) / 2 + bottom.getY()
    );
  }

  public static void main(String... args) {
    var fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
    var blueLeft = fieldLayout.getTagPose(26).get();
    var blueRight = fieldLayout.getTagPose(20).get();
    var blueTop = fieldLayout.getTagPose(21).get();
    var blueBottom = fieldLayout.getTagPose(18).get();

    System.out.println("Center of Blue Hub: " + centerTranslation(blueLeft, blueRight, blueTop, blueBottom));

    var redLeft = fieldLayout.getTagPose(4).get();
    var redRight = fieldLayout.getTagPose(10).get();
    var redTop = fieldLayout.getTagPose(2).get();
    var redBottom = fieldLayout.getTagPose(5).get();

    System.out.println("Center of Red Hub: " + centerTranslation(redLeft, redRight, redTop, redBottom));

    // RobotBase.startRobot(Robot::new);
  }
}
