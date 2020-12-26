package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.LawnmowerDialogFactory;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepLawnmower;
import javafx.collections.FXCollections;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LawnmowerCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private TextField stepLengthField;
    private TextField stacksField;
    private TextField altitudeField;
    private CheckBox walkBoundaryField;
    private ComboBox<LawnmowerDialogFactory.Walk> walkComboBox;
    private TextField waitTimeField;
    private TextField distanceThreshold;

    public LawnmowerCreator(List<String> drones) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {

        stepLengthField = new TextField();
        stacksField = new TextField();
        altitudeField = new TextField();
        walkBoundaryField = new CheckBox();
        walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(LawnmowerDialogFactory.Walk.values());
        waitTimeField = new TextField();
        distanceThreshold = new TextField();

        // Set Defaults
        stepLengthField.setText("1");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkBoundaryField.setSelected(true);
        walkComboBox.getSelectionModel().select(LawnmowerDialogFactory.Walk.WALK);
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        grid.add(new Label("Drone:"), 1, 2);
        grid.add(droneSelection, 2, 2);
        grid.add(new Label("Step Length: "), 1, 3);
        grid.add(stepLengthField, 2, 3);
        grid.add(new Label("Altitude: "), 1, 4);
        grid.add(altitudeField, 2, 4);
        grid.add(new Label("Stacks: "), 1, 5);
        grid.add(stacksField, 2, 5);
        grid.add(new Label("Walk Boundary: "), 1, 6);
        grid.add(walkBoundaryField, 2, 6);
        grid.add(new Label("Walk: "), 1, 7);
        grid.add(walkComboBox, 2, 7);
        grid.add(new Label("Wait Time: "), 1, 8);
        grid.add(waitTimeField, 2, 8);
        grid.add(new Label("Distance Threshold: "), 1, 9);
        grid.add(distanceThreshold, 2, 9);
    }

    @Override
    public MissionStep build() {
        return new MissionStepLawnmower(
                droneSelection.getSelectionModel().getSelectedItem(),
                Float.parseFloat(stepLengthField.getText()),
                Float.parseFloat(altitudeField.getText()),
                Integer.parseInt(stacksField.getText()),
                walkBoundaryField.isSelected(),
                walkComboBox.getSelectionModel().getSelectedItem().id,
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }
}
