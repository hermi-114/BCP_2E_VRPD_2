import java.util.Arrays;
import java.util.HashSet;

import com.gurobi.gurobi.GRBException;
import java.util.List;
import java.util.Set;

public class ColumnGeneration {

    // public static final Set<String> existedSequences = new HashSet<>();

    
    public void solve() throws GRBException {
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes  = new CuttingPlanes();
        PricingProblem pricing = new PricingProblem(cuttingPlanes);
        CutGeneration cutGeneration = new CutGeneration();
        
        Set<String> setUsedRoutes = new HashSet<>();
        double[] lambda;

        int colGen = 0;
        int cutRound = 0;
        final int MAX_CUT_ROUNDS = 5;

        while(true) {

            // ========== COLUMN GENERATION ========
            boolean masterFeasible = true;
            
            while(true) {
                
                colGen++;
                System.out.println("========== Column Generation Iteration " + colGen + " ==========");
    
                try {
                    master.solve();
                    cuttingPlanes.updateDuals(master);
                } catch (GRBException e) {
                    System.err.println("Master infeasible");
                    masterFeasible = false;
                    break;
                }

                double[] pi = master.getDuals();
                double dualTruck = master.getDualVehicle();
                double dualDrone = master.getDualDrone();

                List<Route> bestRoutes = pricing.findBestRoutes(pi, Constant.MAX_DRONE_PER_VEHICLE, dualTruck, dualDrone, setUsedRoutes);

                System.out.println("Pricing returned " + bestRoutes.size() + " routes.");
                
                if(bestRoutes.isEmpty()) break;

                boolean addedNewColumn = false;
                
                for(Route route : bestRoutes) {
                    // System.out.println(route.reducedCost);
                    if (route.reducedCost >= -Constant.EPSILON) continue;

                    // System.out.println("Calling master.addColumn for route with customerServed size: " + route.customerServed.size());
                    // System.out.println("Route customerServed: " + route.customerServed);
                    // System.out.println("Route totalTime: " + route.totalTime);
                    // System.out.println("Route reducedCost: " + route.reducedCost);

                    master.addColumn(route, cuttingPlanes);
                    VRPInstance.routePool.add(route);

                    addedNewColumn = true;
                    
                }

                if(!addedNewColumn) {
                    System.out.println("No negative reduced cost routes added");
                    break;
                }
            }
            
            if(!masterFeasible) {
                master.dispose();
                return;
            }

            double[] dummyVals = master.artificialValues;
            boolean hasArtificial = false;
            for (double val : dummyVals) {
                if (val > Constant.EPSILON) {
                    hasArtificial = true;
                    break;
                }
            }

            if (hasArtificial) {
                System.out.println("LP solution still has artificial variables. Skipping cut generation.");
                break;

            }
            // ======== CUT GENERATION ========

            if (cutRound >= MAX_CUT_ROUNDS) {
                System.out.println("Reached maximum cut rounds (" + MAX_CUT_ROUNDS + "). Stopping.");
                break;
            }


            lambda = master.getPrimes();
            List<ICut> newCuts = cutGeneration.separateCuts(VRPInstance.routePool, lambda);
                
            if(newCuts.isEmpty()) break;
    
            for(ICut cut : newCuts) {
                cuttingPlanes.addCut(cut);
                master.addCut(cut);
            }

            cutRound++;
            // System.out.println("Added " + newCuts.size() + " cuts. Cut round " + cutRound + ".");

        }


        // get all dummies remaining in the routePool
        // if one found, the customer is cannot be served

        double[] dummyVals = master.artificialValues;
        lambda = master.getPrimes();
        master.dispose();

        for (var val : dummyVals) {
            if (val > Constant.EPSILON) {
                System.out.println("Infeasible! There is customer cannot be served");
                System.out.println(val);
            }
        }

        if(lambda != null) {
            System.out.println(Arrays.toString(lambda));
            System.out.println(lambda.length);
        }



        System.out.println("Generated " + VRPInstance.routePool.size() + " real routes");
    }
}
