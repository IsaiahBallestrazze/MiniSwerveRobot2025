// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuadEncoders extends SubsystemBase {
  /** Creates a new QuadEncoders. */

  private final Encoder FRAzimuthEncoder = new Encoder(13, 12, false, Encoder.EncodingType.k4X);
  private final Encoder FLAzimuthEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private final Encoder BRAzimuthEncoder = new Encoder(11, 10, false, Encoder.EncodingType.k4X);
  private final Encoder BLAzimuthEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
  double scaleFactor = .36;




  public QuadEncoders() {}

  @Override
  public void periodic() { //.8905 for 0-1000
    // This method will be called once per scheduler run
    //converts it back to -180 to 180

    FRAzimuthEncoder.setDistancePerPulse(0.8905);
    FLAzimuthEncoder.setDistancePerPulse(.8905);
    BRAzimuthEncoder.setDistancePerPulse(0.8905);
    BLAzimuthEncoder.setDistancePerPulse(0.8905);  }

    public double getFRAzimuthEncoder() {
      double rawDistance = FRAzimuthEncoder.getDistance();
      //double correctedDistance = rawDistance;
      while(!((rawDistance < 1000) && (rawDistance > -1000))){
        if(rawDistance > 1000) rawDistance -= 1000;
        if(rawDistance < -1000) rawDistance +=1000;
        //System.out.println("IN WHILE LOOP");

      }
      //System.out.println("SENT ANGLE " + rawDistance);
      return rawDistance * scaleFactor;
    }
  
  public double getFLAzimuthEncoder() {
    double rawDistance = FLAzimuthEncoder.getDistance();
    //double correctedDistance = rawDistance;
    while(!((rawDistance < 1000) && (rawDistance > -1000))){
      if(rawDistance > 1000) rawDistance -= 1000;
      if(rawDistance < -1000) rawDistance +=1000;
      //System.out.println("IN WHILE LOOP");
    }
    //System.out.println("SENT ANGLE " + rawDistance);
    System.out.println((rawDistance * scaleFactor));
    return (rawDistance * scaleFactor);
  }
  
  public double getBRAzimuthEncoder() {
      return BRAzimuthEncoder.getDistance();
  }
  
  public double getBLAzimuthEncoder() {
      return BLAzimuthEncoder.getDistance();
  }
}
