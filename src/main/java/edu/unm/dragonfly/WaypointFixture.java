package edu.unm.dragonfly;

import com.esri.arcgisruntime.mapping.view.SceneView;

/**
 * @author John Ericksen
 */
public class WaypointFixture implements Fixture {
    private final String name;
    private final NavigateWaypoint waypoint;
    private final RemoveCallback removeCallback;

    public WaypointFixture(String name, NavigateWaypoint waypoint, RemoveCallback removeCallback) {
        this.name = name;
        this.waypoint = waypoint;
        this.removeCallback = removeCallback;
    }

    @Override
    public String getIcon() {
        return "images/pin_blue.png";
    }

    @Override
    public void remove() {
        removeCallback.remove();
    }

    @Override
    public void center(SceneView sceneView) {
        sceneView.setViewpointAsync(ViewpointUtil.create(waypoint.getWaypoint()));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        return name;
    }
}
