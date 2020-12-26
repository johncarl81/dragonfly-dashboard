package edu.unm.dragonfly.mission.step;

import edu.unm.dragonfly.mission.step.MissionStep;

/**
 * @author John Ericksen
 */
public class MissionStepLawnmower implements MissionStep {

    private final String drone;
    public final double steplength;
    public final boolean walkBoundary;
    public final int walk;
    public final int stacks;
    public final double altitude;
    public final double waittime;
    public final double distanceThreshold;

    public MissionStepLawnmower(String drone, double steplength, double altitude, int stacks, boolean walkBoundary, int walk,  double waittime, double distanceThreshold) {
        this.drone = drone;
        this.steplength = steplength;
        this.walkBoundary = walkBoundary;
        this.walk = walk;
        this.stacks = stacks;
        this.altitude = altitude;
        this.waittime = waittime;
        this.distanceThreshold = distanceThreshold;
    }

    @Override
    public String toString() {
        return "Lawnmower{" +
                "drone='" + drone + '\'' +
                ", steplength=" + steplength +
                ", walkBoundary=" + walkBoundary +
                ", walk=" + walk +
                ", stacks=" + stacks +
                ", altitude=" + altitude +
                ", waittime=" + waittime +
                ", distanceThreshold=" + distanceThreshold +
                '}';
    }
}
