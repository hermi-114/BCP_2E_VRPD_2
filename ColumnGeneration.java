import java.util.List;
import java.util.Set;
import java.util.HashSet;
import java.util.Arrays;

import com.gurobi.gurobi.*;
import java.util.ArrayList;

public class ColumnGeneration {

    private static final int MAX_CUT_ROUNDS = 5;

    public void solve() throws GRBException {

        MasterProblem   master         = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes   cuttingPlanes  = new CuttingPlanes();
        Set<String> setUsedRoutes = new HashSet<>();
        PricingProblem  pricing        = new PricingProblem(cuttingPlanes, setUsedRoutes);
        CutGeneration   cutGeneration  = new CutGeneration();

        // Only kept to guard against re-adding identical columns (defensive;
        // pricing already de-duplicates via its own `signatures` set).

        int colGen    = 0;
        int cutRound  = 0;

        outer:
        while (true) {

            // =================================================================
            // COLUMN GENERATION
            // =================================================================
            while (true) {

                colGen++;
                System.out.println("========== Column Generation Iteration "
                                   + colGen + " ==========");

                // --- 1. solve master LP ---
                try {
                    master.solve();
                    cuttingPlanes.updateDuals(master);
                } catch (GRBException e) {
                    System.err.println("Master LP infeasible: " + e.getMessage());
                    master.dispose();
                    return;
                }

                // --- 2. build the dual vector for pricing ---
                double[] pi         = master.getDuals();          // length TOTAL_CUSTOMER
                double dualTruck    = master.getDualVehicle();
                double dualDrone    = master.getDualDrone();

                // --- 3. run pricing ---
                pricing.runThreeStagePricing(pi, dualTruck, dualDrone);
                List<Route> bestRoutes = pricing.getNewRoutes();
                System.out.println("Pricing returned " + bestRoutes.size()
                                   + " candidate routes.");

                if (bestRoutes.isEmpty()) {
                    System.out.println("No candidate routes, column generation converged.");
                    break;
                }

                // --- 4. add the negative-reduced-cost ones to the master ---
                boolean columnsAdded = false;
                for (Route route : bestRoutes) {

                    if (route.reducedCost >= -Constant.EPSILON) continue;

                    String sig = route.getSignature();
                    // if (setUsedRoutes.contains(sig)) continue;
                    setUsedRoutes.add(sig);

                    master.addColumn(route, cuttingPlanes);
                    VRPInstance.routePool.add(route);
                    columnsAdded = true;
                }

                if (!columnsAdded) {
                    System.out.println("All candidate routes had non-negative reduced cost.");
                    break;
                }
            }

            // =================================================================
            // COLUMN PRUNING
            // =================================================================
            try {
                int removed = master.pruneColumns(1e-6);
                if (removed > 0) {
                    System.out.println("Pruned " + removed + " columns with λ ≈ 0.");

                    // keep pool and signatures consistent
                    VRPInstance.routePool.clear();
                    VRPInstance.routePool.addAll(master.getRealRoutes());


                    // re-populate cached primal/dual values on the trimmed model
                    master.solve();
                    cuttingPlanes.updateDuals(master);
                }
            } catch (GRBException e) {
                System.err.println("Pruning/re-solve failed: " + e.getMessage());
                master.dispose();
                return;
            }

            // =================================================================
            // ARTIFICIAL VARIABLE CHECK
            // =================================================================
            double[] dummyVals = master.artificialValues;
            boolean hasArtificial = false;
            if (dummyVals != null) {
                for (double val : dummyVals) {
                    if (val > Constant.EPSILON) { hasArtificial = true; break; }
                }
            }

            if (hasArtificial) {
                System.out.println("LP still has artificial variables. Stopping.");
                break outer;
            }

            // =================================================================
            // CUT GENERATION (placeholder)
            // =================================================================
            if (cutRound >= MAX_CUT_ROUNDS) {
                System.out.println("Reached maximum cut rounds ("
                                   + MAX_CUT_ROUNDS + "). Stopping.");
                break outer;
            }

            double[] lambda = master.getPrimes();
            List<ICut> newCuts = cutGeneration.separateCuts(VRPInstance.routePool, lambda);

            if (newCuts.isEmpty()) {
                System.out.println("No new cuts found. Stopping.");
                break outer;
            }

            for (ICut cut : newCuts) {
                cuttingPlanes.addCut(cut);
                master.addCut(cut);
            }
            cutRound++;
            System.out.println("Added " + newCuts.size() + " cuts. Cut round "
                               + cutRound + ".");
        }

        // =====================================================================
        // REPORT
        // =====================================================================
        double[] dummyVals = master.artificialValues;
        double[] lambda    = master.getPrimes();
        master.dispose();

        if (dummyVals != null) {
            for (int i = 0; i < dummyVals.length; i++) {
                if (dummyVals[i] > Constant.EPSILON) {
                    System.out.println("Infeasible! Customer " + (i + 1)
                                       + " is not served (dummy = " + dummyVals[i] + ")");
                }
            }
        }
        
        List<Integer> chosen = new ArrayList<>();

        if (lambda != null) {
            // System.out.println("Final lambda: " + Arrays.toString(lambda));
            for(int i = 0; i < lambda.length; i++) {
                if(lambda[i] > 0) chosen.add(i);
            }
        }

        System.out.println("Generated " + VRPInstance.routePool.size() + " real routes.");


        System.out.println("Select " + chosen.size() + " routes.");
        for(int id : chosen) {
            System.out.printf("%.3f  |   %s\n", lambda[id], VRPInstance.routePool.get(id));
        }
    }
}