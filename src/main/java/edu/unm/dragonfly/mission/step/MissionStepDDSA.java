package edu.unm.dragonfly.mission.step;

import edu.unm.dragonfly.Walk;

/**
 * @author John Ericksen
 */
public class MissionStepDDSA implements MissionStep{

    private final String drone;
    private final float radius;
    private final float stepLength;
    private final float altitude;
    private final int loops;
    private final int stacks;
    private final Walk walk;
    private final float waittime;
    private final float distanceThreshold;

    public MissionStepDDSA(String drone, float radius, float stepLength, float altitude, int loops, int stacks, Walk walk, float waittime, float distanceThreshold) {
        this.drone = drone;
        this.radius = radius;
        this.stepLength = stepLength;
        this.altitude = altitude;
        this.loops = loops;
        this.stacks = stacks;
        this.walk = walk;
        this.waittime = waittime;
        this.distanceThreshold = distanceThreshold;
    }

    @Override
    public String toString() {
        return "DDSA{" +
                "drone='" + drone + '\'' +
                ", radius=" + radius +
                ", stepLength=" + stepLength +
                ", altitude=" + altitude +
                ", loops=" + loops +
                ", stacks=" + stacks +
                ", walk=" + walk +
                ", waittime=" + waittime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
