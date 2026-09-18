import com.gurobi.gurobi.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.util.*;

public class Main {

    public static void main(String[] args) {

        boolean checkRoute = true;

        String set = "C102";
        String inputFile = set + ".txt";
        String outputFile = "./output/output.csv";
        // String outputFile = "./output/output_" + Config.SIZE_CUSTOMER_DATASET + "_aut.csv";
        new File("./output").mkdirs();          // <-- make sure the dir exists

        
        try(PrintWriter out = new PrintWriter(outputFile)) {
            out.println();
            out.println(",,,SIZE CUSTOMER DATASET = " + Config.SIZE_CUSTOMER_DATASET);
            out.println();
            out.println(",,set,obj,total_time(s),drone(s),bcp(s)");

            runSingle(inputFile, out);
            // runAll(out);

            if(checkRoute) printRoutePool("./output/route.txt");
            

        } catch (Exception e) {
            e.printStackTrace();
        }

    }
    
    public static void runAll(PrintWriter out) throws Exception {
    
        File folder = new File("./data/Solomon");
    
    
        File[] files = folder.listFiles();
        if (files != null) {
            Arrays.sort(files);
    
            for (File file : files) {    
                if (!file.isFile()) continue;
                if (!file.getName().endsWith(".txt")) continue;
                if (file.getName().equals("capacities.txt")) continue;

                System.out.println("Running " + file.getName());
                try {
                    runSingle(file.getName(), out);
                    // run++;
                } catch (Exception e) {
                    System.err.println("FAILED on " + file.getName());
                    e.printStackTrace();
                    out.println(file.getName() + ",FAILED,-1,-1,-1");
                }
            }
        }
    

    }

    public static void runSingle(String dataset, PrintWriter out) {
        VRPInstance.reset();
        DroneScheduleEnumeration.paretoMap.clear();

        DataLoader.loadCustomer("./data/Solomon/" + dataset);
        DataLoader.loadFleet(dataset);
        Constant.initDerivedValues();

        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance();
        VRPInstance.initNgNeighborhoods(Constant.LABEL_MAX_NG_SIZE);

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long end_drone_enum = System.currentTimeMillis();

        BranchAndPrice bap = new BranchAndPrice();
        try { bap.run(); } catch (GRBException e) { e.printStackTrace(); }

        long end_bap = System.currentTimeMillis();

        double time_drone  = (end_drone_enum - startTime) / 1000.0;
        double time_branch = (end_bap - end_drone_enum) / 1000.0;
        double time_whole  = (end_bap - startTime) / 1000.0;

        System.out.println("=================== Best solution ===================");
        if (bap.getBestSolution() == null) {
            System.out.println("(no integer solution found)");
            out.println(dataset + ",NO_SOL," + time_whole + ","
                      + time_drone + "," + time_branch);
        } else {
            for (Route r : bap.getBestSolution()) System.out.println(r);
            System.out.println("Objective = " + bap.getBestObjective());
            out.println(",," + dataset + "," + bap.getBestObjective() + ","
                      + time_whole + "," + time_drone + "," + time_branch);
        }

        System.out.printf("\nDrone Schedules Enumeration: %.1fs\n", time_drone);
        System.out.printf("Branch and price:           %.1fs\n", time_branch);
        System.out.printf("\nProgramme runs in %.1fs\n", time_whole);
    }

    // ----- these stay as they are -----
    public static void printRoutePool(String fileName) {
        try (PrintWriter out = new PrintWriter(fileName)) {
            for (Route r : VRPInstance.routePool) out.println(r);
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }

    public static void printParetoFront(String fileName) {
        int counter = 0;
        try (PrintWriter out = new PrintWriter(fileName)) {
            for (int customer = 1; customer <= Constant.TOTAL_CUSTOMER; customer++) {
                out.println("\n==== Customer " + customer + " ====");
                for (int drone = 1; drone <= Constant.MAX_DRONE_PER_VEHICLE; drone++) {
                    for (var entry : DroneScheduleEnumeration.paretoMap.get(customer).get(drone).entrySet()) {
                        out.println("\n-" + customer + "- " + getCustomerServedSet(entry.getKey()));
                        for (DroneSchedule s : entry.getValue().nonDominatedSchedules) {
                            out.println(s + s.sequences.toString());
                            counter++;
                        }
                    }
                }
            }
            System.out.println("TOTAL DRONE SCHEDULES: " + counter);
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
    }

    public static List<Integer> getCustomerServedSet(BigInteger customerServed) {
        List<Integer> indices = new ArrayList<>();
        BigInteger temp = customerServed;
        while (!temp.equals(BigInteger.ZERO)) {
            int index = temp.getLowestSetBit();
            indices.add(index);
            temp = temp.clearBit(index);
        }
        return indices;
    }
}