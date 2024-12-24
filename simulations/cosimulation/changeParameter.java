// Simcenter STAR-CCM+ macro: changeParameter.java
// Written by Simcenter STAR-CCM+ 16.06.008
package macro;

import java.util.*;

import star.common.*;
import star.base.neo.*;

public class changeParameter extends StarMacro {

  public void execute() {
    Simulation simulation_0 = getActiveSimulation();
    double[] angularAcc = {1.55,1.56,1.57};
    setAngularAcc(angularAcc,simulation_0);
  }

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
