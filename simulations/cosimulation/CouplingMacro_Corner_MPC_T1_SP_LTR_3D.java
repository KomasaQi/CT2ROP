// Simcenter STAR-CCM+ macro: CouplingMacro001.java
// Written by Simcenter STAR-CCM+ 16.06.008
package macro;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.Arrays;
import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.report.*;
import star.flow.*;

public class CouplingMacro_Corner_MPC_T1_SP_LTR_3D extends StarMacro {

    public void execute() {
        /*
         * Simulation Settings
         * Define the path of Flag file and the path of Output file
         */
        String basePath = "D:/��Һ������Ϸ���ƽ̨/Һ�޳�Corner�켣����_3D/"; //Please Set this path to install the script

        String matlab_flag = basePath + "matlab_flag.csv";
        String matlab_output = basePath + "matlab_output.csv";
        String java_flag = basePath + "java_flag.csv";
        String java_output = basePath + "java_output.csv";

        /*
         * Initialize the simulation
         */
        writeFile(java_flag, "0");
        StringBuilder starDataStr = new StringBuilder();
        starDataStr.append("start");

        double[] SimParameters = { 0, 0, 0, 0, 0, 0};
        SimParameters = csvRead(matlab_output, 4);
        setFlag(java_flag, "1"); // Read the simulation params, and tell MATLAB that I finished reading~

        double MaxStopPhysicalTime = 0;
        int iteration = (int) SimParameters[0]; // Synchronize the simulation timestep
        double timeStep = SimParameters[1]; // Synchronize the sloving timestep
        int MatlabOutVariableNum = (int) SimParameters[2]; // Output parameter number of Simulink
        int StarOutVariableNum = (int) SimParameters[3]; // Output parameter number of StarCCM+

        double[] StarData = new double[StarOutVariableNum];
        double[] MatlabData = new double[MatlabOutVariableNum];

        double[] gravity = { 0, 0, 0 }; //ax,ay,az
        double[] angularAcc = { 0, 0, 0 }; //ddroll,ddpitch,ddyaw

        Simulation simulation_0 = getActiveSimulation(); // Define the current simulation
        openResidual(simulation_0); // open the resitual plot of current simulation

        /*
         * Iteration process of the simulation
         */
        for (int i = 0; i < iteration; i++) {
            // STATE 1: test unremittingly matlab_flag, wait for Simulink to end one step
            while (!getFlag(matlab_flag)) {
            }
            setFlag(matlab_flag, "0");

            // STATE 2: extract data from the output file of MATLAB
            MatlabData = csvRead(matlab_output, MatlabOutVariableNum);

            // STATE 3: Simulate one step utilizing the output data of MATLAB
            for (int p = 0; p < 3; p++) {
                gravity[p] = MatlabData[p];
            }
            setGravity(simulation_0, gravity, "Physics 1");
            for (int p = 0; p < 3; p++) {
                angularAcc[p] = MatlabData[p+3];
            }
            setAngularAcc(angularAcc,simulation_0);

            setMaximumPhysicalTime(simulation_0, (double) ((i + 1) * timeStep));
            runSim(simulation_0);

            // STATE 4: Stop the current simulation, and output StarCCM+ data
            exportCsv(simulation_0, "dataToExport", java_output);

            // STATE 5: set java_flag, to allow MATLAB to collect output data
            setFlag(java_flag, "1");

        }
        // Stop the entire simulation and set java_flag, to alow Simulink to end the simulation
        delay(2000);
        setFlag(java_flag, "1");

    }

    /**
     * Start the specified simulation
     * 
     * @param simulation_0
     */
    private void runSim(Simulation simulation_0) {
        simulation_0.getSimulationIterator().run();
    }

    private void setGravity(Simulation simulation_0, double[] gravity, String physicsModel) {
        PhysicsContinuum physicsContinuum_0 = ((PhysicsContinuum) simulation_0.getContinuumManager()
                .getContinuum(physicsModel));

        physicsContinuum_0.getReferenceValues().get(Gravity.class).setComponents(gravity[0], gravity[1], gravity[2]);

        Units units_1 = ((Units) simulation_0.getUnitsManager().getObject("m/s^2"));

        physicsContinuum_0.getReferenceValues().get(Gravity.class).setUnits(units_1);
    }

    /**
     * ����ָ�����ƵĻ�ͼ���ݵ�ָ��·����CSV�ļ�
     * 
     * @param simulation_0
     * @param plotName
     * @param destPath
     */
    private void exportCsv(Simulation simulation_0, String plotName, String destPath) {
        MonitorPlot monitorPlot_0 = ((MonitorPlot) simulation_0.getPlotManager().getPlot(plotName));

        monitorPlot_0.export(resolvePath(destPath), ",");

    }

    /**
     * ����������ֹͣʱ��
     * 
     * @param simulation_0
     * @param stopTime
     */
    private void setMaximumPhysicalTime(Simulation simulation_0, double stopTime) {
        PhysicalTimeStoppingCriterion physicalTimeStoppingCriterion_0 = ((PhysicalTimeStoppingCriterion) simulation_0
                .getSolverStoppingCriterionManager().getSolverStoppingCriterion("Maximum Physical Time"));

        physicalTimeStoppingCriterion_0.getMaximumTime().setValue(stopTime);

        Units units_0 = ((Units) simulation_0.getUnitsManager().getObject("s"));

        physicalTimeStoppingCriterion_0.getMaximumTime().setUnits(units_0);
    }

    /**
     * �򿪵�ǰ����Ĳв��ͼ
     * 
     * @param simulation_0
     */
    private void openResidual(Simulation simulation_0) {

        ResidualPlot residualPlot_0 = ((ResidualPlot) simulation_0.getPlotManager().getPlot("Residuals"));

        residualPlot_0.open();
    }

    public static void writeFile(String fileRoot, String cont) {
        // ֻ����Ӣ�ģ�������ר�ŵ��ַ��������������ֽ���
        try (BufferedWriter bos = new BufferedWriter(new FileWriter(fileRoot))) {
            bos.write(cont);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * ����־λ�ļ������ݣ������ر�־λ
     * 
     * @param srcpath Ŀ���־λ�ļ���·��
     * @return flag ����Ŀ���ļ�����д�ı�־λ
     */
    public static boolean getFlag(String srcpath) {
        boolean flag = false;
        int data = 0;
        try (FileInputStream fis = new FileInputStream(srcpath)) {
            data = fis.read();
        } catch (Exception e) {
            e.printStackTrace();// TODO: handle exception
        }
        if (data == 49) {
            flag = true;
        }
        return flag;
    }

    /**
     * ��λ��־λ�ļ�������
     * 
     * @param fileRoot Ŀ���־λ�ļ���·��
     * @param flag     ��Ҫ�õı�־λ��"0"��"1"
     */
    public static void setFlag(String fileRoot, String flag) {
        try (BufferedWriter bos = new BufferedWriter(new FileWriter(fileRoot))) {
            bos.write(flag);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    /**
     * ��ʱ�������ȴ�����ĺ���ʱ��
     * 
     * @param milliSec ��Ҫ�ȴ��ĺ�����
     */
    public static void delay(int milliSec) {
        try {
            Thread.currentThread().sleep(milliSec);// ����
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * ��ȡCSV�������һ�У�����ָ������
     * 
     * @param srcpath            ��ȡĿ���ļ������һ�У�����ָ��������data����
     * @param FileOutVariableNum Matlab����Ĳ����ĸ���
     * @return
     */
    public static double[] csvRead(String srcpath, int FileOutVariableNum) {
        String str = readLastLineV1(srcpath);
        String[] str2;
        str2 = str.split(",");
        double[] data = new double[FileOutVariableNum];
        for (int i = 0; i < str2.length; i++) {
            data[i] = Double.parseDouble(str2[i]);
        }
        return data;
    }

    /**
     * ��ȡ�ļ������һ��
     * 
     * @param srcpath Ŀ���ļ���ַ
     * @return �����ַ�������ȡ�����ļ������һ��
     */
    public static String readLastLineV1(String srcpath) {
        // �洢���
        StringBuilder builder = new StringBuilder();
        try (RandomAccessFile randomAccessFile = new RandomAccessFile(new File(srcpath), "r")) {
            // ָ��λ�ÿ�ʼΪ0��������󳤶�Ϊ length-1
            long fileLastPointer = randomAccessFile.length() - 1;
            // �Ӻ���ǰ��ȡ�ļ�
            for (long filePointer = fileLastPointer; filePointer != -1; filePointer--) {
                // �ƶ�ָ��ָ��
                randomAccessFile.seek(filePointer);
                int readByte = randomAccessFile.readByte();
                if (0xA == readByte) {
                    // LF='\n'=0x0A ����
                    if (filePointer == fileLastPointer) {
                        // ��������Ļ��У����˵�
                        continue;
                    }
                    break;
                }
                if (0xD == readByte) {
                    // CR ='\r'=0x0D �س�
                    if (filePointer == fileLastPointer - 1) {
                        // ����ǵ����Ļس�Ҳ���˵�
                        continue;
                    }
                    break;
                }
                builder.append((char) readByte);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return builder.reverse().toString();
    }


    /**
     * Set angular acceleraion around 3 orthogonal axis
     * @param angluarAcc 3 dimensional vector,{ddroll,ddpitch,ddyaw} w/ unit /s^2
     * @param simulation_0
     */
    private void setAngularAcc(double[] angluarAcc, Simulation simulation_0){
        setParameter("ddroll",angluarAcc[0],simulation_0);
        setParameter("ddpitch",angluarAcc[1],simulation_0);
        setParameter("ddyaw",angluarAcc[2],simulation_0);
      }
    
    private void setParameter(String param_name, double value, Simulation simulation_0) {
    
        ScalarGlobalParameter scalarGlobalParameter_0 = 
          ((ScalarGlobalParameter) simulation_0.get(GlobalParameterManager.class).getObject(param_name));
    
        scalarGlobalParameter_0.getQuantity().setValue(value);
    
        Units units_0 = 
          ((Units) simulation_0.getUnitsManager().getObject("/s^2"));
    
        scalarGlobalParameter_0.getQuantity().setUnits(units_0);
      }


}


