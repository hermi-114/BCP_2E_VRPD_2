import java.math.BigInteger;
import java.util.*;

import com.gurobi.gurobi.GRBException;

public class ColumnGeneration {

    private static final int MAX_CUT_ROUNDS = 5;

    public static class NodeResult {
        public double       objective;    // forcedCost + LP value
        public double[]     lambda;       // primal values, same order as columns
        public List<Route>  columns;      // parallel to lambda
        public double[]     artificial;   // artificial values (should be ~0)
        public boolean      feasible;     // false if any artificial > EPS
    }

    /** Root-only wrapper (backwards compat). */
    public void solve() throws GRBException {
        NodeResult res = solveForNode(new ArrayList<>(), new HashSet<>());
        System.out.println("Root LP objective = " + res.objective
                         + "  feasible = " + res.feasible);
    }

    /**
     * Solve the LP for one B&P node.
     * @param forcedRoutes  routes already decided to be in the solution
     * @param forbiddenSigs signatures of routes that must not be used at this node
     */
    public NodeResult solveForNode(List<Route> forcedRoutes,
                                   Set<String> forbiddenSigs) throws GRBException {

        // ---------- 1. forced state ----------
        BigInteger forcedCustomers = BigInteger.ZERO;
        double forcedCost = 0.0;
        int forcedVehicles = 0, forcedDrones = 0;
        for (Route r : forcedRoutes) {
            forcedCustomers = forcedCustomers.or(r.customerServedHashed);
            forcedCost     += r.totalTime;
            forcedVehicles += 1;
            forcedDrones   += r.getNumDrone();
        }

        int vehCap   = Constant.MAX_VEHICLE - forcedVehicles;
        int droneCap = Constant.MAX_DRONE   - forcedDrones;
        if (vehCap < 0 || droneCap < 0) {
            NodeResult dead = new NodeResult();
            dead.feasible = false;
            dead.objective = Double.POSITIVE_INFINITY;
            dead.lambda   = new double[0];
            dead.columns  = new ArrayList<>();
            dead.artificial = new double[Constant.TOTAL_CUSTOMER];
            return dead;
        }

        // ---------- 2. fresh master, cuts, pricing ----------
        MasterProblem master = new MasterProblem(Constant.TOTAL_CUSTOMER);
        CuttingPlanes cuttingPlanes = new CuttingPlanes();
        Set<String> signatures = new HashSet<>(forbiddenSigs);
        PricingProblem pricing = new PricingProblem(cuttingPlanes, signatures);
        CutGeneration cutGeneration = new CutGeneration();

        master.setResourceCaps(vehCap, droneCap);
        master.setUncoveredMask(forcedCustomers);

        // ---------- 3. column generation ----------
        int maxColRounds = 200;
        for (int it = 0; it < maxColRounds; it++) {
            try {
                master.solve();
            } catch (GRBException e) {
                master.dispose();
                NodeResult dead = new NodeResult();
                dead.feasible = false;
                dead.objective = Double.POSITIVE_INFINITY;
                dead.lambda = new double[0];
                dead.columns = new ArrayList<>();
                dead.artificial = new double[Constant.TOTAL_CUSTOMER];
                return dead;
            }

            cuttingPlanes.updateDuals(master);

            double[] pi = master.getDuals();
            double dualTruck = master.getDualVehicle();
            double dualDrone = master.getDualDrone();

            pricing.runThreeStagePricing(pi, dualTruck, dualDrone,
                                         forcedCustomers, forbiddenSigs);
            List<Route> candidates = pricing.getNewRoutes();
            if (candidates.isEmpty()) break;

            boolean added = false;
            int cap = 200;
            for (Route r : candidates) {
                if (cap-- <= 0) break;
                if (r.reducedCost >= -Constant.EPSILON) continue;
                String sig = r.getSignature();
                if (signatures.contains(sig)) continue;
                master.addColumn(r, cuttingPlanes);
                signatures.add(sig);
                added = true;
            }
            if (!added) break;
        }

        // ---------- 4. cut generation (bounded) ----------
        for (int cr = 0; cr < MAX_CUT_ROUNDS; cr++) {
            double[] lambda = master.getPrimes();
            List<ICut> newCuts = cutGeneration.separateCuts(master.getRealRoutes(), lambda);
            if (newCuts.isEmpty()) break;
            for (ICut cut : newCuts) {
                cuttingPlanes.addCut(cut);
                master.addCut(cut);
            }
            // re-run a short CG round after adding cuts
            try { master.solve(); } catch (GRBException e) { break; }
            cuttingPlanes.updateDuals(master);
            double[] pi = master.getDuals();
            double dT = master.getDualVehicle();
            double dD = master.getDualDrone();
            pricing.runThreeStagePricing(pi, dT, dD, forcedCustomers, forbiddenSigs);
            List<Route> cand = pricing.getNewRoutes();
            boolean added = false;
            for (Route r : cand) {
                if (r.reducedCost >= -Constant.EPSILON) continue;
                String sig = r.getSignature();
                if (signatures.contains(sig)) continue;
                master.addColumn(r, cuttingPlanes);
                signatures.add(sig);
                added = true;
            }
            if (!added) continue;
        }

        // ---------- 5. package result ----------
        try { master.solve(); } catch (GRBException e) { /* leave as-is */ }

        NodeResult res = new NodeResult();
        res.lambda     = master.getPrimes();
        res.columns    = new ArrayList<>(master.getRealRoutes());
        res.artificial = master.artificialValues;
        res.objective  = forcedCost + master.getObjectiveValue();

        res.feasible = true;
        for (double a : res.artificial) {
            if (a > Constant.EPSILON) { res.feasible = false; break; }
        }

        master.dispose();
        return res;
    }
}