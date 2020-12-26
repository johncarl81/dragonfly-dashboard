package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.Walk;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepDDSA;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class DDSACreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private TextField radiusField;
    private TextField stepLengthField;
    private TextField loopsField;
    private TextField stacksField;
    private TextField altitudeField;
    private ComboBox<Walk> walkComboBox;
    private TextField waitTimeField;
    private TextField distanceThreshold;

    public DDSACreator(List<String> drones) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {

        radiusField = new TextField();
        stepLengthField = new TextField();
        loopsField = new TextField();
        stacksField = new TextField();
        altitudeField = new TextField();
        walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        waitTimeField = new TextField();
        distanceThreshold = new TextField();

        // Set Defaults
        radiusField.setText("1");
        stepLengthField.setText("1");
        loopsField.setText("5");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkComboBox.getSelectionModel().select(Walk.WALK);
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        grid.add(new Label("Drone:"), 1, 2);
        grid.add(droneSelection, 2, 2);
        grid.add(new Label("Radius: "), 1, 3);
        grid.add(radiusField, 2, 3);
        grid.add(new Label("Step Length: "), 1, 4);
        grid.add(stepLengthField, 2, 4);
        grid.add(new Label("Altitude: "), 1, 5);
        grid.add(altitudeField, 2, 5);
        grid.add(new Label("Loops: "), 1, 6);
        grid.add(loopsField, 2, 6);
        grid.add(new Label("Stacks: "), 1, 7);
        grid.add(stacksField, 2, 7);
        grid.add(new Label("Walk: "), 1, 8);
        grid.add(walkComboBox, 2, 8);
        grid.add(new Label("Wait Time: "), 1, 9);
        grid.add(waitTimeField, 2, 9);
        grid.add(new Label("Distance Threshold: "), 1, 10);
        grid.add(distanceThreshold, 2, 10);
    }

    @Override
    public MissionStep build() {
        return new MissionStepDDSA(droneSelection.getSelectionModel().getSelectedItem(),
                Float.parseFloat(radiusField.getText()),
                Float.parseFloat(stepLengthField.getText()),
                Float.parseFloat(altitudeField.getText()),
                Integer.parseInt(loopsField.getText()),
                Integer.parseInt(stacksField.getText()),
                walkComboBox.getSelectionModel().getSelectedItem(),
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }
}
