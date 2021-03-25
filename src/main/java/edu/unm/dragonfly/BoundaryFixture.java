package edu.unm.dragonfly;

import com.esri.arcgisruntime.mapping.view.SceneView;
import edu.unm.dragonfly.mission.Waypoint;

import java.util.List;

/**
 * @author John Ericksen
 */
public class BoundaryFixture implements Fixture {
    private final String name;
    private final List<Waypoint> boundary;
    private final RemoveCallback removeCallback;

    public BoundaryFixture(String name, List<Waypoint> boundary, RemoveCallback removeCallback) {
        this.name = name;
        this.boundary = boundary;
        this.removeCallback = removeCallback;
    }

    @Override
    public String getIcon() {
        return "images/draw_polygon.png";
    }

    @Override
    public void remove() {
        removeCallback.remove();
    }

    @Override
    public void center(SceneView sceneView) {
        sceneView.setViewpointAsync(ViewpointUtil.create(boundary));
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
