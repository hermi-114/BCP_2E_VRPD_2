import java.util.Arrays;

import com.gurobi.gurobi.GRBException;
import java.rmi.StubNotFoundException;

public class ColumnGeneration {

    private final int maxLoopTime = 1000; // for test
    // public static final Set<String> existedSequences = new HashSet<>();

    public void solve() throws GRBException {
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes = null;
        cuttingPlanes = new CuttingPlanes();
        PricingProblem pricing = new PricingProblem(cuttingPlanes);

        int loopTime = 0;
        while (true && (loopTime++ < maxLoopTime)) {
            // while(true) {

            System.out.println("========== Column Generation Iteration " + loopTime + " ==========");

            master.solve();
            
            if(cuttingPlanes != null) cuttingPlanes.updateDuals(master);

            double[] pi = master.getDualVariables();
            double dualTruck = master.getDualVehicle();
            double dualDrone = master.getDualDrone();
            Route bestRoute = pricing.findBestRoute(pi, Constant.MAX_DRONE_PER_VEHICLE, dualTruck, dualDrone);

            if (bestRoute == null) {
                break;
            }

            if (bestRoute.reducedCost >= -Constant.EPSILON) {
                break;
            }

            master.addRealColumn(bestRoute, cuttingPlanes);

            VRPInstance.routePool.add(bestRoute);

            logIteration(bestRoute, master.objectiveValue, pi);
        }

        // get all dummies remaining in the routePool
        // if one found, the customer is cannot be served

        double[] dummyVals = master.artificialValues;
        double[] weights = master.getPrimeVariables();
        master.dispose();

        System.out.println(Arrays.toString(weights));
        System.out.println(weights.length);

        for (var val : dummyVals) {
            if (val > Constant.EPSILON) {
                System.out.println("Infeasible! There is customer cannot be served");
                System.out.println(val);
            }
        }

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

        // System.out.print("Duals : [");
        // for (int i = 0; i < duals.length; i++) {
        // if (i > 0) {
        // System.out.print(", ");
        // }
        // System.out.printf("%.6f", duals[i]);
        // }
        System.out.println("]\n\n");
    }
}
