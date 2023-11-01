// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */



public class Leg extends Segment{

    Translation2d totalVector;

    final Translation2d velDirection;

    public Leg(Translation2d p1, Translation2d p2)
    {
        super(p1,p2);
        totalVector = p2.minus(p1);

        velDirection = totalVector.div(totalVector.getNorm());
    }

    @Override
    public Translation2d calc(Translation2d position, double velocity)
    {
        Translation2d relativePos = position.minus(p1);

        double distance = distancePassed(position);

        Translation2d distanceVec = velDirection.times(distance);

        Translation2d unitFixVec = distanceVec.minus(relativePos);
        unitFixVec = unitFixVec.div(unitFixVec.getNorm());

        return (velDirection.plus(unitFixVec.times(0.5/*kP*/))).times(velocity);

    }

    @Override
    public double distancePassed(Translation2d position)
    {
        Translation2d relativePos = position.minus(p1);

        double distanceMoved = (relativePos.getX() * velDirection.getX()) + (relativePos.getY()*velDirection.getY());
        return distanceMoved;
    }

    @Override
    public double getLength()
    {
        return totalVector.getNorm();
    }

    @Override
    public String toString() {
        return "\n~Leg~\np1 : " + p1 + "\np2 : " + p2 + "\nTotalVector : " + totalVector;
    }
}
