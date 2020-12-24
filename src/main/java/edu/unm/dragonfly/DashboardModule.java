package edu.unm.dragonfly;

import com.google.inject.AbstractModule;
import ros.RosBridge;

public class DashboardModule extends AbstractModule {

    private final RosBridge bridge;

    public DashboardModule() {
        bridge = new RosBridge();
        bridge.connect("ws://0.0.0.0:9090", true);
    }

    @Override
    protected void configure() {
        bind(RosBridge.class).toInstance(bridge);
    }
}
