import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

import com.gurobi.gurobi.*;

public class MasterProblem {

    private GRBEnv env;
    private GRBModel model;

    private GRBConstr[] coverConstr;
    private GRBConstr   truckConstr;
    private GRBConstr   droneConstr;
    public  List<GRBConstr> cutsConstr;

    private List<GRBVar> artificialVars;
    private List<GRBVar> realVars;
    private List<Route>  realVarRoutes;

    public double[] artificialValues;
    public double   objectiveValue;

    public MasterProblem(int totalCustomer) throws GRBException {
        env = new GRBEnv(true);
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set("LogFile", "gurobi.log");
        env.start();
        model = new GRBModel(env);

        coverConstr = new GRBConstr[totalCustomer];
        for (int i = 0; i < totalCustomer; i++) {
            coverConstr[i] = model.addConstr(new GRBLinExpr(),
                                             GRB.EQUAL, 1.0, "cover_" + (i + 1));
        }
        truckConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,
                                      Constant.MAX_VEHICLE, "c_truck");
        droneConstr = model.addConstr(new GRBLinExpr(), GRB.LESS_EQUAL,
                                      Constant.MAX_DRONE,   "c_drone");

        artificialVars   = new ArrayList<>();
        realVars         = new ArrayList<>();
        realVarRoutes    = new ArrayList<>();
        cutsConstr       = new ArrayList<>();
        artificialValues = new double[totalCustomer];

        initializeArtificialColumns(totalCustomer);
        model.update();
    }

    private void initializeArtificialColumns(int totalCustomer) throws GRBException {
        // Lower than 9999 to keep the LP numerically stable relative to route costs.
        final double M = 1000.0;
        for (int i = 0; i < totalCustomer; i++) {
            GRBColumn col = new GRBColumn();
            col.addTerm(1, coverConstr[i]);
            GRBVar dummy = model.addVar(0, 1, M, GRB.CONTINUOUS, col, "dummy_" + i);
            artificialVars.add(dummy);
        }
        model.update();
    }

    public void setResourceCaps(int vehCap, int droneCap) throws GRBException {
        truckConstr.set(GRB.DoubleAttr.RHS, vehCap);
        droneConstr.set(GRB.DoubleAttr.RHS, droneCap);
        model.update();
    }

    public void setUncoveredMask(BigInteger forcedCustomers) throws GRBException {
        for (int i = 0; i < coverConstr.length; i++) {
            if (forcedCustomers.testBit(i + 1)) {
                coverConstr[i].set(GRB.CharAttr.Sense, GRB.LESS_EQUAL);
                coverConstr[i].set(GRB.DoubleAttr.RHS, 0.0);
            }
        }
        model.update();
    }

    private final java.util.Set<String> columnSignatures = new java.util.HashSet<>();

    public void addColumn(Route route, CuttingPlanes cuttingPlanes) throws GRBException {
        // guard against duplicate columns
        String sig = route.getSignature();
        if (columnSignatures.contains(sig)) return;
        columnSignatures.add(sig);

        double cost = route.totalTime;
        GRBColumn col = new GRBColumn();

        // coverage
        for (int customer : route.customerServed) {
            if (customer == 0 || customer > Constant.TOTAL_CUSTOMER) continue;
            col.addTerm(1, coverConstr[customer - 1]);
        }
        // resource caps
        col.addTerm(1,                  truckConstr);
        col.addTerm(route.getNumDrone(), droneConstr);

        // cuts
        if (cuttingPlanes != null && cuttingPlanes.cuts != null) {
            for (int i = 0; i < cuttingPlanes.cuts.size(); i++) {
                double coef = cuttingPlanes.cuts.get(i).getCoefficientForRoute(route);
                if (Math.abs(coef) > Constant.EPSILON)
                    col.addTerm(coef, cutsConstr.get(i));
            }
        }

        // ---- FIX: branch row coefficients go into the column BEFORE the var exists ----
        for (BranchRowHandle h : branchRows) {
            double coef = h.dec.candidate.coefficient(route);
            if (Math.abs(coef) > Constant.EPSILON) {
                col.addTerm(coef, h.constr);
            }
        }

        GRBVar realVar = model.addVar(0, 1, cost, GRB.CONTINUOUS, col,
                                    "real_" + realVars.size());
        realVars.add(realVar);
        realVarRoutes.add(route);
        model.update();
    }

    /**
     * Adds a cut and returns the Gurobi constraint so the caller can roll
     * it back if the subsequent re-solve becomes infeasible.
     */
    public GRBConstr addCutAndReturn(ICut cut) throws GRBException {
        GRBLinExpr lhs = new GRBLinExpr();
        char sense = (cut instanceof ARCCut) ? GRB.GREATER_EQUAL
                  : (cut instanceof R1Cut)  ? GRB.LESS_EQUAL
                  : (char) 0;
        if (sense == 0) throw new IllegalArgumentException("Unknown cut type");

        GRBConstr constr = model.addConstr(lhs, sense, cut.getRHS(),
                                           "cut_" + cutsConstr.size());
        cutsConstr.add(constr);

        for (int i = 0; i < realVars.size(); i++) {
            double coef = cut.getCoefficientForRoute(realVarRoutes.get(i));
            if (Math.abs(coef) > Constant.EPSILON)
                model.chgCoeff(constr, realVars.get(i), coef);
        }
        model.update();
        return constr;
    }

    /** Backwards-compatible: adds a cut and discards the handle. */
    public void addCut(ICut cut) throws GRBException {
        addCutAndReturn(cut);
    }

    /** Removes a cut constraint from the model and from cutsConstr. */
    public void removeConstraint(GRBConstr c) throws GRBException {
        model.remove(c);
        cutsConstr.remove(c);
        model.update();
    }

    public void solve() throws GRBException {
        model.set(GRB.IntParam.Method, 1);
        model.optimize();

        int status = model.get(GRB.IntAttr.Status);
        if (status == GRB.Status.OPTIMAL) {
            objectiveValue   = model.get(GRB.DoubleAttr.ObjVal);
            artificialValues = extractArtificialVariableValues();
        } else if (status == GRB.Status.INFEASIBLE) {
            throw new GRBException("Master LP infeasible", status);
        } else {
            throw new GRBException("Master LP status " + status, status);
        }
    }

    public double[] extractArtificialVariableValues() throws GRBException {
        double[] v = new double[artificialVars.size()];
        for (int i = 0; i < artificialVars.size(); i++)
            v[i] = artificialVars.get(i).get(GRB.DoubleAttr.X);
        return v;
    }

    public double[] getDuals() throws GRBException {
        double[] pi = new double[coverConstr.length];
        for (int i = 0; i < coverConstr.length; i++)
            pi[i] = coverConstr[i].get(GRB.DoubleAttr.Pi);
        return pi;
    }

    public double[] getPrimes() throws GRBException {
        double[] lambda = new double[realVars.size()];
        for (int i = 0; i < realVars.size(); i++)
            lambda[i] = realVars.get(i).get(GRB.DoubleAttr.X);
        return lambda;
    }

    public double getDualVehicle() throws GRBException {
        return truckConstr.get(GRB.DoubleAttr.Pi);
    }
    public double getDualDrone() throws GRBException {
        return droneConstr.get(GRB.DoubleAttr.Pi);
    }

    public List<Route> getRealRoutes() { return realVarRoutes; }
    public double getObjectiveValue() { return objectiveValue; }

    public void dispose() throws GRBException {
        model.dispose();
        env.dispose();
    }

    public void setInteger(boolean asInteger) throws GRBException {
        char type = asInteger ? GRB.BINARY : GRB.CONTINUOUS;
        for (GRBVar v : realVars)     v.set(GRB.CharAttr.VType, type);
        for (GRBVar v : artificialVars) v.set(GRB.CharAttr.VType, type);
    }

    public void setMipGapTolerance(double gap) throws GRBException {
        model.set(GRB.DoubleParam.MIPGap, gap);
    }

    public boolean hasIntegerSolution() throws GRBException {
        int status = model.get(GRB.IntAttr.Status);
        if (status != GRB.Status.OPTIMAL && status != GRB.Status.SUBOPTIMAL
                && status != GRB.Status.TIME_LIMIT && status != GRB.Status.INTERRUPTED)
            return false;
        return model.get(GRB.IntAttr.SolCount) > 0;
    }

    public GRBConstr addTempRow(double[] coefs, char sense, double rhs) throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();
        for (int i = 0; i < realVars.size(); i++) {
            if (Math.abs(coefs[i]) > Constant.EPSILON)
                expr.addTerm(coefs[i], realVars.get(i));
        }
        GRBConstr c = model.addConstr(expr, sense, rhs, "temp_branch");
        model.update();
        return c;
    }

    /** Removes a constraint created by addTempRow. */
    public void removeTempRow(GRBConstr c) throws GRBException {
        model.remove(c);
        model.update();
    }

    /**
     * Solve the current LP and return its objective.
     * Returns Double.POSITIVE_INFINITY when infeasible.
     */
    public double solveReturnObjective() throws GRBException {
        model.set(GRB.IntParam.Method, 1);   // barrier — fast for repeated LP solves
        model.optimize();
        int status = model.get(GRB.IntAttr.Status);
        if (status == GRB.Status.OPTIMAL)
            return model.get(GRB.DoubleAttr.ObjVal);
        return Double.POSITIVE_INFINITY;
    }

    /** Total number of real (route) variables currently in the master. */
    public int getNumRealVars() { return realVars.size(); }

    /** Underlying routes parallel to realVars; needed for coefficient computation. */
    public List<Route> getRealVarRoutesList() { return realVarRoutes; }

    public void addPermanentRow(double[] coefs, char sense, double rhs, String name) throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();
        for (int i = 0; i < realVars.size(); i++) {
            if (Math.abs(coefs[i]) > Constant.EPSILON)
                expr.addTerm(coefs[i], realVars.get(i));
        }
        model.addConstr(expr, sense, rhs, name);
        model.update();
    }

    private final List<BranchRowHandle> branchRows = new ArrayList<>();

    private static class BranchRowHandle {
        final GRBConstr constr;
        final BranchDecision dec;
        BranchRowHandle(GRBConstr c, BranchDecision d) { constr = c; dec = d; }
    }

    /** Adds a permanent branch row. Coefficients cover every current column. */
    public void addBranchRow(BranchDecision dec) throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();
        for (int i = 0; i < realVars.size(); i++) {
            double coef = dec.candidate.coefficient(realVarRoutes.get(i));
            if (Math.abs(coef) > Constant.EPSILON)
                expr.addTerm(coef, realVars.get(i));
        }
        char sense = dec.upperBound ? GRB.LESS_EQUAL : GRB.GREATER_EQUAL;
        GRBConstr c = model.addConstr(expr, sense, dec.rhs, "branch_" + dec.candidate);
        branchRows.add(new BranchRowHandle(c, dec));
        model.update();
    }

    /** Flips λ and artificials to binary, solves, caches results, restores LP mode. */
    public boolean solveAsMip() throws GRBException {
        try {
            setInteger(true);
            model.set(GRB.DoubleParam.MIPGap, 1e-4);
            model.set(GRB.IntParam.Threads,
                    Math.max(1, Runtime.getRuntime().availableProcessors() - 1));
            model.set(GRB.IntParam.Method, -1);
            model.optimize();

            int status = model.get(GRB.IntAttr.Status);
            int sols   = model.get(GRB.IntAttr.SolCount);
            System.out.printf("    [MIP] status=%d sols=%d gap=%.2e obj=%.6f%n",
                    status, sols,
                    sols > 0 ? model.get(GRB.DoubleAttr.MIPGap) : Double.NaN,
                    sols > 0 ? model.get(GRB.DoubleAttr.ObjVal) : Double.NaN);

            if (sols > 0
                    && (status == GRB.Status.OPTIMAL
                    || status == GRB.Status.SUBOPTIMAL
                    || status == GRB.Status.TIME_LIMIT
                    || status == GRB.Status.INTERRUPTED)) {
                objectiveValue   = model.get(GRB.DoubleAttr.ObjVal);
                artificialValues = extractArtificialVariableValues();
                return true;
            }
            return false;
        } finally {
            // CRITICAL: restore LP mode so the next master.solve() is a real LP.
            setInteger(false);
        }
    }

    public boolean addAllAndSolveMip(List<List<Route>> enumeratedByD) throws GRBException {
        if (enumeratedByD == null) return false;
        int before = realVars.size();
        for (List<Route> byD : enumeratedByD) {
            if (byD == null) continue;
            for (Route r : byD) addColumn(r, null);
        }
        System.out.println("    [MIP] added " + (realVars.size() - before)
                        + " new columns, total = " + realVars.size());
        return solveAsMip();
    }
}