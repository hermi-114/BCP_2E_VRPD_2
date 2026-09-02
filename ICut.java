
import java.util.Set;

public interface ICut {
    double getDual();
    void setDual(double dual);
    double getRHS();
    Set<Integer> getSubsetC();

    double getCoefficientForRoute(Route route);
    
    // Penalties for pricing subproblem (arc reduced costs)
    double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones);
    double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones);
}