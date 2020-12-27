package edu.unm.dragonfly;

import com.google.inject.AbstractModule;
import javafx.stage.Stage;
import ros.RosBridge;

public class DashboardModule extends AbstractModule {

    private final RosBridge bridge;
    private final Stage stage;

    public DashboardModule(Stage stage) {
        bridge = new RosBridge();
        bridge.connect("ws://0.0.0.0:9090", true);

        this.stage = stage;
    }

    @Override
    protected void configure() {
        bind(RosBridge.class).toInstance(bridge);
        bind(Stage.class).toInstance(stage);
    }
}
