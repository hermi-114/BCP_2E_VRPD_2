import java.util.ArrayList;
import java.util.List;

import com.gurobi.gurobi.*;

public class CuttingPlanes {

    public final List<ICut> cuts = new ArrayList<>();

    public CuttingPlanes() {}

    public void addCut(ICut cut) {
        cuts.add(cut);
    }

    /** Needed for branching: reset all cuts and their duals. */
    public void clear() {
        cuts.clear();
    }

    public void updateDuals(MasterProblem master) {
        List<GRBConstr> cutsConstr = master.cutsConstr;

        if (cutsConstr.size() != cuts.size()) {
            throw new IllegalStateException(
                "Cut/constraint size mismatch: " + cuts.size()
                + " cuts vs " + cutsConstr.size() + " constraints");
        }

        for (int i = 0; i < cuts.size(); i++) {
            try {
                double dual = cutsConstr.get(i).get(GRB.DoubleAttr.Pi);
                cuts.get(i).setDual(dual);
            } catch (GRBException e) {
                throw new RuntimeException("Failed to read cut dual at index " + i, e);
            }
        }
    }

    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule,
                                                   int numDrones) {
        double penalty = 0.0;
        for (ICut cut : cuts) {
            penalty += cut.getReducedCostPenaltyForDroneArc(park, schedule, numDrones);
        }
        return penalty;
    }

    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) {
        double penalty = 0.0;
        for (ICut cut : cuts) {
            penalty += cut.getReducedCostPenaltyForTruckArc(src, dst, numDrones);
        }
        return penalty;
    }
}