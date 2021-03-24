package edu.unm.dragonfly.mission;

import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;

/**
 * @author John Ericksen
 */
public class GridUtil {

    public static class GridBuilder {

        private final GridPane grid;
        private int index = 1;

        private GridBuilder(GridPane grid) {
            this.grid = grid;
        }

        public GridBuilder add(Node node) {
            grid.add(node, 1, index, 2, index);

            index++;

            return this;
        }

        public GridBuilder add(String labelName, Node node) {
            grid.add(new Label(labelName), 1, index);
            grid.add(node, 2, index);

            index++;

            return this;
        }

        public GridBuilder increment() {
            index++;

            return this;
        }
    }

    public static GridBuilder builder(GridPane grid) {
        return new GridBuilder(grid);
    }
}
