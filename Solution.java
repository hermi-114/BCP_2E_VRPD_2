import java.util.ArrayList;
import java.util.List;

public class Solution {
    public final List<Route> routes;
    double objectiveValue;        // -1 means "not yet computed"

    public Solution() {
        this.routes = new ArrayList<>();
        this.objectiveValue = -1;
    }

    public Solution(List<Route> routes) {
        this.routes = routes;
        this.objectiveValue = -1;
    }

    /** Sum over routes (matches the paper's objective). */
    public double objectiveValue() {
        if (objectiveValue >= 0) return objectiveValue;

        double sum = 0.0;
        for (Route route : routes) sum += route.totalTime;
        return objectiveValue = sum;
    }

    @Override
    public String toString() {
        int maxDrones = 0, totalDrones = 0;
        for (Route r : routes) {
            maxDrones   = Math.max(maxDrones, r.getNumDrone());
            totalDrones += r.getNumDrone();
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Solution")
          .append(" | objectiveValue = ").append(objectiveValue())
          .append(" | trucks used = ").append(routes.size())
          .append(" | total drones = ").append(totalDrones)
          .append(" | max drones on a route = ").append(maxDrones)
          .append("\n");
        for (Route r : routes) sb.append(r).append("\n");
        return sb.toString();
    }
}