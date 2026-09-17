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

        for (int customer : route.customerServed) {
            if (customer == 0 || customer > Constant.TOTAL_CUSTOMER) continue;
            col.addTerm(1, coverConstr[customer - 1]);
        }
        col.addTerm(1, truckConstr);
        col.addTerm(route.getNumDrone(), droneConstr);

        if (cuttingPlanes != null && cuttingPlanes.cuts != null) {
            List<ICut> cuts = cuttingPlanes.cuts;
            for (int i = 0; i < cuts.size(); i++) {
                double coef = cuts.get(i).getCoefficientForRoute(route);
                if (Math.abs(coef) > Constant.EPSILON)
                    col.addTerm(coef, cutsConstr.get(i));
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

    public boolean addAllAndSolveMip(List<List<Route>> enumeratedByD) throws GRBException {
        
        if (enumeratedByD == null) return false;

        // 1. add all columns (addColumn now deduplicates internally)
        int before = realVars.size();
        for (List<Route> byD : enumeratedByD) {
            if (byD == null) continue;
            for (Route r : byD) addColumn(r, null);
        }
        int added = realVars.size() - before;
        System.out.println("    [MIP] added " + added + " new columns, total = " + realVars.size());

        // 2. switch to MIP
        setInteger(true);
        model.set(GRB.DoubleParam.MIPGap, 1e-4);
        model.set(GRB.IntParam.Threads,
                Math.max(1, Runtime.getRuntime().availableProcessors() - 1));

        // 3. solve as MIP
        model.set(GRB.IntParam.Method, -1);   // auto-select MIP method
        model.optimize();

        int status = model.get(GRB.IntAttr.Status);
        int sols   = model.get(GRB.IntAttr.SolCount);
        System.out.printf("    [MIP] status=%d sols=%d gap=%.2e obj=%.6f%n",
        status, sols,
        sols > 0 ? model.get(GRB.DoubleAttr.MIPGap) : Double.NaN,
        sols > 0 ? model.get(GRB.DoubleAttr.ObjVal) : Double.NaN);

        // 4. cache the objective/artificial values so getters work
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
}