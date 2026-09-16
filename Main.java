import com.gurobi.gurobi.*;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class Main {
    public static void main(String[] args) {

        String fileOutName_drone = "drone.txt";
        String fileOutName_route = "route.txt";

        DataLoader.loadCustomer("./data/Solomon/" + (Config.INPUT_TYPE + Config.INPUT_SET + ".txt"));
        DataLoader.loadFleet();

        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance();
        VRPInstance.initNgNeighborhoods(Constant.LABEL_MAX_NG_SIZE);

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long end_drone_enum = System.currentTimeMillis();

        // -------- Column generation (root only, to seed the route pool) --------
        ColumnGeneration columnGeneration = new ColumnGeneration();
        try {
            columnGeneration.solve();
        } catch (GRBException e) {
            e.printStackTrace();
        }

        long end_column_gen = System.currentTimeMillis();

        // -------- Branch-and-price --------
        BranchAndPrice bap = new BranchAndPrice();
        try {
            bap.run();
        } catch (GRBException e) {
            e.printStackTrace();
        }

        long end_bap = System.currentTimeMillis();

        System.out.println("=================== Best solution ===================");
        if (bap.getBestSolution() == null) {
            System.out.println("(no integer solution found)");
        } else {
            for (Route r : bap.getBestSolution()) System.out.println(r);
            System.out.println("Objective = " + bap.getBestObjective());
        }

        if (Config.PRINT_DRONE) printParetoFront("./output/" + fileOutName_drone);
        if (Config.PRINT_ROUTE) printRoutePool("./output/" + fileOutName_route);

        System.out.printf("\nDrone Schedules Enumeration: %ds\n", (end_drone_enum - startTime) / 1000);
        System.out.printf("Column generation:          %ds\n", (end_column_gen - end_drone_enum) / 1000);
        System.out.printf("Branch and price:           %ds\n", (end_bap - end_column_gen) / 1000);
        System.out.printf("\nProgramme runs in %ds\n", (end_bap - startTime) / 1000);
    }

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