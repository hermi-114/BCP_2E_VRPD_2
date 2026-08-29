import java.util.ArrayList;
import java.util.List;
import com.gurobi.gurobi.*;
import java.lang.reflect.Array;


public class MasterProblem {

    private GRBEnv env;
    private GRBModel model;
    private GRBConstr[] coverConstr;
    private List<GRBVar> artificialVars;
    private List<GRBVar> realVars;

    // private GRBLinExpr obj;

    public double[] artificialValues;

    public MasterProblem(int totalCustomer) throws GRBException {

        env = new GRBEnv();
        env.set(GRB.IntParam.LogToConsole, 0);
        env.start();
        model = new GRBModel(env);

        // obj = new GRBLinExpr();

        artificialValues = new double[totalCustomer];

        coverConstr = new GRBConstr[totalCustomer];
        for(int i = 0; i < totalCustomer; i++) {
            coverConstr[i] = model.addConstr(new GRBLinExpr(), GRB.EQUAL, 1.0, "c_" + (i+1));
        }

        artificialVars = new ArrayList<>();
        realVars = new ArrayList<>();

        initializeBigMColumns(totalCustomer);

        model.update();

    }

    private void initializeBigMColumns(int totalCustomer) throws GRBException {
        double M = 999999.0;

        for(int i = 0; i < totalCustomer; i++) {

            GRBColumn col = new GRBColumn();
            col.addTerm(1, coverConstr[i]);

            GRBVar dummy = model.addVar(0, 1, M, GRB.CONTINUOUS, col, "dummy_" + i);

            artificialVars.add(dummy);
        }

        model.update();
    }

    public void addRealColumn(Route route) throws GRBException {
        double cost = route.totalTime;
        
        GRBColumn col = new GRBColumn();
        GRBVar realVar = model.addVar(0, 1, cost, GRB.CONTINUOUS, col, "real_" + realVars.size());

        for(int customer : route.customerServed) {
            col.addTerm(1, coverConstr[customer-1]); // constraint in [0, totalCustomer)
        }
        
        realVars.add(realVar);
        model.update();
    }

    public double[] extractArtificialVariableValues() throws GRBException {
        double[] values = new double[artificialVars.size()];

        for(int i = 0; i < artificialVars.size(); i++) {
            values[i] = artificialVars.get(i).get(GRB.DoubleAttr.X);
        }

        return values;
    }

    public double[] getDualVariables() throws GRBException {
        double[] pi = new double[coverConstr.length];
        for (int i = 0; i < coverConstr.length; i++) {
            pi[i] = coverConstr[i].get(GRB.DoubleAttr.Pi);
        }
        return pi;
    }

    public void solve() throws GRBException {
        model.set(GRB.IntParam.Method, 1);

        // for(var realVar : realVars) obj.addTerm(1, realVar);
        // for(var unrealVar : artificialVars) obj.addTerm(1, unrealVar);

        // model.setObjective(obj, GRB.MINIMIZE);
        // model.update();
        model.optimize();

        if(Config.PRINT_SWITCH_CMD) System.out.println("Solving...");

        int status = model.get(GRB.IntAttr.Status);

        if(status == GRB.Status.OPTIMAL) {
            if(Config.PRINT_SWITCH_CMD) {
                System.out.println("Model is optimal");
                System.out.println(model.get(GRB.DoubleAttr.ObjVal));
            }

            
            artificialValues = extractArtificialVariableValues();
        } else if (status == GRB.Status.INFEASIBLE) {
            if(Config.PRINT_SWITCH_CMD) System.err.println("Model is infeasible. Computing IIS...");
            model.computeIIS();
            model.write("model_infeasible.ilp");

        } else {
            throw new RuntimeException("RMP optimization failed with status code " + status);
        }
    }

    public void dispose() throws GRBException {
        model.dispose();
        env.dispose();
    }
    
}
