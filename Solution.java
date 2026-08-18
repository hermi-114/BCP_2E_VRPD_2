import java.util.ArrayList;
import java.util.List;

public class Solution {
    public final List<Route> routes;
    double objectiveValue;

    public Solution() {
        this.routes = new ArrayList<>();
        this.objectiveValue = -1;
    }

    public Solution(List<Route> routes) {
        this.routes = routes;
        objectiveValue = -1;
    }

    public void addRoute(Route route) {
        this.routes.add(route);
        if(route.totalTime > objectiveValue) objectiveValue = route.totalTime;
    }

    public double objectiveValue() {
        if(objectiveValue != -1) return this.objectiveValue;

        double max = Double.MIN_VALUE;
        for(Route route : routes) {
            if(route.totalTime > max) max = route.totalTime;
        }

        return objectiveValue = max;
    }
}
