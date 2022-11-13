// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cybercatclasses;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonRootName;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
@JsonRootName(value = "path")
@JsonIgnoreProperties(ignoreUnknown = true)
public class PathFromJson {
    private @JsonIgnore() double acceleration;
    private @JsonIgnore() double curvature;


    private @JsonProperty(required = true, value="pose") Pose2d pose;
    private @JsonIgnore() double time;
    private @JsonIgnore() double velocity;

    public PathFromJson(){}

    public PathFromJson(double acc, double cur, Pose2d pose, double t, double vel){
        this.acceleration = acc;
        this.curvature = cur;
        this.pose = pose;
        this.time = t;
        this.velocity = vel;
    }

    // public double getAcceleration() {
    //     return this.acceleration;
    // }

    // public void setAcceleration(double a){
    //     this.acceleration = a;
    // }
    // public double getCurvature() {
    //     return curvature;
    // }

    // public void setCurvature(double curvature) {
    //     this.curvature = curvature;
    // }

    // public Pose2d getPose() {
    //     return pose;
    // }

    // public void setPose(Pose2d pose) {
    //     this.pose = pose;
    // }

    // public double getTime() {
    //     return time;
    // }

    // public void setTime(double time) {
    //     this.time = time;
    // }

    // public double getVelocity() {
    //     return velocity;
    // }

    // public void setVelocity(double velocity) {
    //     this.velocity = velocity;
    // }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return String.format("The path starting point is (%.1f, %.1f) with heading (%.1f) degrees",
            this.pose.getX(), this.pose.getY(), this.pose.getRotation().getDegrees());
    }

    public String toJson() {
        try {
            ObjectMapper objectMapper = new ObjectMapper();
            objectMapper.enable(SerializationFeature.WRAP_ROOT_VALUE);
            return objectMapper.writeValueAsString(this);
        } catch (JsonProcessingException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return null;
    }

    

}
