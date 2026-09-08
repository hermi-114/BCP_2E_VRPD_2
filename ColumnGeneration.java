import java.util.Arrays;

import com.gurobi.gurobi.GRBException;
import java.util.ArrayList;
import java.util.List;

public class ColumnGeneration {

    // public static final Set<String> existedSequences = new HashSet<>();

    public void solve() throws GRBException {
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes  = new CuttingPlanes();
        PricingProblem pricing = new PricingProblem(cuttingPlanes);
        CutGeneration cutGeneration = new CutGeneration();

        double[] lambda;

        int loopTime;
        while(true) {

            // ========== COLUMN GENERATION ========
            loopTime = 0;

            while(true) {
    
                System.out.println("========== Column Generation Iteration " + loopTime++ + " ==========");
    
                try {
                    master.solve();
                } catch (GRBException e) {
                    System.err.println("Master infeasible");
                    master.dispose();
                    return;
                }
                
                cuttingPlanes.updateDuals(master);
    
                double[] pi = master.getDuals();
                double dualTruck = master.getDualVehicle();
                double dualDrone = master.getDualDrone();
                Route bestRoute = pricing.findBestRoute(pi, Constant.MAX_DRONE_PER_VEHICLE, dualTruck, dualDrone);

                if(bestRoute == null) break;
    
                if (bestRoute.reducedCost >= -Constant.EPSILON) {
                    break;
                }
    
                master.addRealColumn(bestRoute, cuttingPlanes);
    
                VRPInstance.routePool.add(bestRoute);

                logIteration(bestRoute, dualDrone, pi);
            }



            // ======== CUT GENERATION ========
            lambda = master.getReals();
            // List<ICut> newCuts = cutGeneration.separate(VRPInstance.routePool, lambda);
            List<ICut> newCuts = new ArrayList<>();

            if(newCuts.isEmpty()) break;

        }

        double[] dummyVals = master.artificialValues;
        lambda = master.getReals();
        master.dispose();

        for (var val : dummyVals) {
            if (val > Constant.EPSILON) {
                System.out.println("Infeasible! There is customer cannot be served");
                System.out.println(val);
            }
        }

        System.out.println(Arrays.toString(lambda));
        System.out.println(lambda.length);


        System.out.println("Generated " + VRPInstance.routePool.size() + " real routes");
    }

    private void logIteration(
            Route route,
            double masterObjective,
            double[] duals) {

        if (route == null) {
            System.out.println("Route      : null");
        } else {
            System.out.println("Route      : " + route);
            System.out.printf("Reduced Cost: %.10f%n", route.reducedCost);
        }

        System.out.printf("Master Obj : %.10f%n", masterObjective);

        System.out.print("Duals : [");
        for (int i = 0; i < duals.length; i++) {
        if (i > 0) {
        System.out.print(", ");
        }
        System.out.printf("%.6f", duals[i]);
        }
        System.out.println("]\n\n");
    }
}
