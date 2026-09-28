public class Constant {

    public static int TOTAL_CUSTOMER;

    // ============== FLEET PARAMETERS ===================
    public static int MAX_VEHICLE = 30;
    public static int MAX_DRONE_PER_VEHICLE = 4;
    public static int MAX_DRONE = MAX_VEHICLE * MAX_DRONE_PER_VEHICLE;
    public static int MAX_NEIGHBOURS_PER_NEIGHBOURHOOD = 6; // cap size
    public static int DRONE_MAX_STOP = 3; // max customer a drone can serve at a parking spot
    public static double TRUCK_MAX_SHIFT_TIME; // get the time window of depot
    public static int MAX_SCHEDULE_IN_SDIK = 5; // choose the bests  (D: drone used, i: current customer, K: subsets of current node's neighbourhood N_i)
    public static int LABEL_MAX_NG_SIZE = 10; // pricing

    // =============== PHYSICAL CONSTANTS ===================
    public static double TRUCK_SPEED = 80; // km/h
    public static double TRUCK_PAYLOAD = 300; // kg
    public static double DRONE_WEIGHT = 11.0; // kg
    public static double DRONE_AND_EQUIPMENT_WEIGHT; // 20% of truck capacity
    public static double DRONE_PAYLOAD = 30.0; // kg
    public static double DRONE_BATTERY_WEIGHT = 9.25;
    public static double DRONE_SPINNING_BLADE_AREA = 0.586; // m^2
    public static int    DRONE_BLADE_NUMBER = 4;
    public static double DRONE_BATTERY_CAPACITY = 2308.8; // watt * hour
    public static double DRONE_SPEED = 128; // km/h
    public static double DRONE_SETUP_TIME = 0.083; // 5 minutes
    public static double G_FORCE = 9.81;
    public static double AIR_DENSITY = 1.204;

    public static final double EPSILON = 1e-7;
    public static final double ROUTE_REDUCED_COST_THRESHOLD = -1e-2; // rcsp
    public static final double M = 1e7 - 1; // big m artificial column - master problem

    // ============ THREE-STAGE COLUMN GENERATION =================
    public static int MAX_COLUMNS_LIGHT     = 30;  // author
    public static int MAX_COLUMNS_HEURISTIC = 30;  // author
    public static int MAX_COLUMNS_EXACT     = 150; // author: exact labelling
    public static int MAX_ADD_PER_ROUND     = 800; // max routes added to master
    public static int MAX_COL_ROUNDS_ROOT   = 150;  // max col-gen loops at root node
    public static int MAX_COL_ROUNDS_CHILD  = 30;  // max col-gen loops

    // =============== NODE TIME LIMITS (Pulse) =================== branch
    public static long NODE_TIME_LIMIT_ROOT_MS  = 120000; // time limit (milisecond)
    public static long NODE_TIME_LIMIT_CHILD_MS = 10000;  // time linit (milisecond)
    public static long SB_TIME_LIMIT_MS         = 4000;  // for each strong branch

    // =============== CUTS =================== cut gen
    public static int    CUT_ROUNDS_ROOT         = 25;  // cut-gen loop at root node
    public static int    CUT_ROUNDS_CHILD        = 3;   // cut-gen loop
    public static int    MAX_CUTS_PER_ROUND_ROOT = 300; // max cuts (constr) total added at root node
    public static int    MAX_CUTS_PER_ROUND_CHILD= 15;  // max cuts (constr) total added ar ndeo
    public static double CUT_VIOLATION_THRESHOLD = 0.1; // self-definition, if '>' -> add cut

    // =============== STRONG BRANCHING ===================
    public static int    STRONG_BRANCHING_PHASE1_KEEP    = 4;   // cheap heuristic branching
    public static int    STRONG_BRANCHING_PHASE2_KEEP    = 3;   // get best solutions
    public static double STRONG_BRANCHING_HISTORY_DECAY  = 0.9; // decay rate  |  0: no memory; 0.9: keep historical score of branching candidate
    public static double STRONG_BRANCHING_INFEAS_SCORE   = 1e4; // for infeasible lp
    public static int    MAX_ARC_CANDIDATES_PER_NODE     = 30;  // max arc at 1 node
    public static boolean PARALLEL_SB                    = true; // parallel branching switch - multi threads (chatgpt)

    // =============== ROUTE ENUMERATION =================== col gen
    public static int ROUTE_ENUM_THRESHOLD   = 5000; // author  |  if < 5000 -> add to MIP solver
    public static int ROUTE_ENUM_HARD_CAP    = 200_000; // cap infinite loop

    // =============== DUAL SMOOTHING =================== col gen
    public static double SMOOTHING_ALPHA_INIT  = 0.9; // initial at node
    public static double SMOOTHING_ALPHA_MIN   = 0.3; // threshold
    public static double SMOOTHING_ALPHA_DECAY = 0.9;

    public static void initDerivedValues() {
        if (TRUCK_PAYLOAD <= 0) {
            throw new IllegalStateException(
                "TRUCK_PAYLOAD must be set by DataLoader.loadFleet() before initDerivedValues()");
        }
        DRONE_AND_EQUIPMENT_WEIGHT = 0.20 * TRUCK_PAYLOAD;
        int k = (int) Math.floor(TRUCK_PAYLOAD / DRONE_AND_EQUIPMENT_WEIGHT);
        System.out.println("[Constant] TRUCK_PAYLOAD              = " + TRUCK_PAYLOAD);
        System.out.println("[Constant] DRONE_AND_EQUIPMENT_WEIGHT  = " + DRONE_AND_EQUIPMENT_WEIGHT);
        System.out.println("[Constant] k = floor(Q / q_d)          = " + k);
        if (k <= 0) throw new IllegalStateException("k must be > 0");
    }
}