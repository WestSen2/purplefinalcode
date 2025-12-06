// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {> Task :compileJava FAILED
/Users/rushilchopra/purpleteamfinalcode/purplefinalcode/src/main/java/frc/robot/RobotContainer.java:71: error: cannot find symbol
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                ^
  symbol:   method share()
  location: variable joystick of type CommandPS5Controller
/Users/rushilchopra/purpleteamfinalcode/purplefinalcode/src/main/java/frc/robot/RobotContainer.java:72: error: cannot find symbol
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                ^
  symbol:   method share()
  location: variable joystick of type CommandPS5Controller
2 errors
Compilation Error!
GradleRIO detected this build failed due to a Compile Error (compileJava).
Check that all your files are saved, then scroll up in this log for more information.
[Incubating] Problems report is available at: file:///Users/rushilchopra/purpleteamfinalcode/purplefinalcode/build/reports/problems/problems-report.html

FAILURE: Build failed with an exception.

* What went wrong:
Execution failed for task ':compileJava'.
> Compilation failed; see the compiler output below.
  /Users/rushilchopra/purpleteamfinalcode/purplefinalcode/src/main/java/frc/robot/RobotContainer.java:71: error: cannot find symbol
          joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                  ^
    symbol:   method share()
    location: variable joystick of type CommandPS5Controller
  /Users/rushilchopra/purpleteamfinalcode/purplefinalcode/src/main/java/frc/robot/RobotContainer.java:72: error: cannot find symbol
          joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                  ^
    symbol:   method share()
    location: variable joystick of type CommandPS5Controller
  2 errors

* Try:
> Check your code and dependencies to fix the compilation error(s)
> Run with --scan to get full insights.

BUILD FAILED in 497ms
1 actionable task: 1 executed

 *  The terminal process "/bin/zsh '-l', '-c', './gradlew build   -Dorg.gradle.java.home="/Users/rushilchopra/wpilib/2025/jdk"'" terminated with exit code: 1. 
 *  Terminal will be reused by tasks, press any key to close it. 

    RobotBase.startRobot(Robot::new);
  }
}
