
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.gurobi.gurobi.*;

public class CuttingPlanes {

    public final List<ICut> cuts = new ArrayList<>();

    public CuttingPlanes() {}


    public void addCut(ICut cut) {
        cuts.add(cut);
    }

    public void updateDuals(MasterProblem master) {

        List<GRBConstr> cutsConstr = master.cutsConstr;

        for(int i = 0; i < cuts.size(); i++) {

            try {
                double dual = cutsConstr.get(i).get(GRB.DoubleAttr.Pi);
                cuts.get(i).setDual(dual);
                

            } catch (GRBException e) {
                e.printStackTrace();
            }
        }
    }

    public double getReducedCostPenaltyForDroneArc(int park, DroneSchedule schedule, int numDrones) {

        double penalty = 0;

        for(ICut cut : cuts)
            penalty += cut.getReducedCostPenaltyForDroneArc(park, schedule, numDrones);

        return penalty;
    }

    public double getReducedCostPenaltyForTruckArc(int src, int dst, int numDrones) { // src and dst must be the original

        double penalty = 0;

        for(ICut cut : cuts)
            penalty += cut.getReducedCostPenaltyForTruckArc(src, dst, numDrones);
        
        return penalty;
    }


}
