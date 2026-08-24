import java.io.FileNotFoundException;
import java.io.PrintWriter;

public class Main {
    public static void main(String[] args) {

        String fileName = "C101.txt";

        DataLoader.load("./data/Solomon/" + fileName);

        long startTime = System.currentTimeMillis();

        VRPInstance.calculateDistance();

        DroneScheduleEnumeration droneSchedulesEnum = new DroneScheduleEnumeration();
        droneSchedulesEnum.solve();

        long endTime = System.currentTimeMillis();

        
        try (PrintWriter out = new PrintWriter("output.txt")) {

            for (int customer = 1; customer <= Constant.TOTAL_CUSTOMER; customer++) {

                if(DroneScheduleEnumeration.paretoMap.get(customer) == null) break;

                out.println("================== Customer " + customer + "  ===================");
                for (int drone = 1; drone <= Constant.MAX_DRONE_PER_VEHICLE; drone++) {

                    for (var S_d_i_K : DroneScheduleEnumeration.paretoMap
                            .get(customer)
                            .get(drone)
                            .entrySet()) {

                        out.println(S_d_i_K.getKey().toString());

                        for (DroneSchedule schedule : S_d_i_K.getValue().nonDominatedSchedules) {
                            out.println(schedule.toString() + schedule.sequences.toString());
                        }
                    }
                }
            }
        } catch (FileNotFoundException e) {
            System.err.println("File not found " + e.getMessage());
        }
        
        System.out.printf("Progamme runs in %ds", (endTime-startTime)/1000);
    }
}
