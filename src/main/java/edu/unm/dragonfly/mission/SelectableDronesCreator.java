package edu.unm.dragonfly.mission;

import javafx.collections.FXCollections;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.CheckComboBox;

import java.util.ArrayList;
import java.util.List;

/**
 * @author John Ericksen
 */
public class SelectableDronesCreator implements GridUtil.GridBuilderInput {
    private final CheckComboBox<String> droneSelection;

    public SelectableDronesCreator(List<String> drones) {
        droneSelection = new CheckComboBox<>(FXCollections.observableList(drones));
    }

    public void create(GridUtil.GridBuilder gridBuilder) {
        gridBuilder.add("Drones:", droneSelection);
    }

    public void create(GridPane grid, int row) {
        grid.add(new Label("Drones:"), 0, row);
        grid.add(droneSelection, 1, row);
    }

    public List<String> getSelectedDrones() {
        List<String> selected = new ArrayList<>();
        for(int i = 0; i < droneSelection.getCheckModel().getItemCount(); i++) {
            if(droneSelection.getCheckModel().isChecked(i)) {
                selected.add(droneSelection.getItems().get(i));
            }
        }
        return selected;
    }
}
