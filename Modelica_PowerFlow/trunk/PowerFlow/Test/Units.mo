within PowerFlow.Test;
package Units

  model PowerPlantTest1 "Test primary control"
    import PowerFlow;
    annotation (
      experiment(StopTime=30),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                           graphics));
    PowerFlow.Units.PowerPlant powerPlant 
                                     annotation (Placement(transformation(
            extent={{-20,0},{2,22}}, rotation=0)));
    Modelica.Blocks.Sources.Constant schedule(k=450) 
      annotation (Placement(transformation(extent={{-80,-40},{-60,-20}},
            rotation=0)));
    PowerFlow.Sources.PrescribedPowerLoad prescribedLoad(phi=
          0.34906585039887)               annotation (Placement(
          transformation(extent={{20,0},{40,20}}, rotation=0)));
    Modelica.Blocks.Sources.Step step(
      startTime=10,
      height=-40,
      offset=490) 
                 annotation (Placement(transformation(extent={{40,40},{60,60}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant secondary(k=40) 
                 annotation (                       Placement(transformation(
            extent={{-80,0},{-60,20}},    rotation=0)));
    Modelica.Blocks.Sources.Constant primary(k=40) 
                 annotation (                       Placement(transformation(
            extent={{-80,40},{-60,60}},   rotation=0)));
  equation
    connect(powerPlant.terminal, prescribedLoad.terminal) annotation (Line(
        points={{2,10},{20,10}},
        color={0,0,0},
        pattern=LinePattern.None,
        smooth=Smooth.None));
    connect(step.y, prescribedLoad.P) annotation (Line(
        points={{61,50},{70,50},{70,10},{41,10}},
        color={0,0,127},
        pattern=LinePattern.None,
        smooth=Smooth.None));
    connect(secondary.y, powerPlant.plantDispatch[2]) 
                                                   annotation (Line(
        points={{-59,10},{-40,10},{-40,4},{-20,4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(schedule.y, powerPlant.plantDispatch[1]) 
                                                  annotation (Line(
        points={{-59,-30},{-30,-30},{-30,3.33333},{-20,3.33333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(primary.y, powerPlant.plantDispatch[3]) annotation (Line(
        points={{-59,50},{-30,50},{-30,4.66667},{-20,4.66667}},
        color={0,0,127},
        smooth=Smooth.None));
  end PowerPlantTest1;

  model PowerPlantTest2 "Test secondary control"
    import PowerFlow;
    annotation (
      experiment(StopTime=600),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                           graphics));
    PowerFlow.Units.PowerPlant powerPlant 
                                     annotation (Placement(transformation(
            extent={{-20,0},{2,22}}, rotation=0)));
    Modelica.Blocks.Sources.Constant schedule(k=490) 
      annotation (Placement(transformation(extent={{-80,-40},{-60,-20}},
            rotation=0)));
    PowerFlow.Sources.PrescribedPowerLoad prescribedLoad(phi=
          0.34906585039887)               annotation (Placement(
          transformation(extent={{20,0},{40,20}}, rotation=0)));
    Modelica.Blocks.Sources.Ramp ramp(
      offset=490,
      height=48,
      duration=180,
      startTime=100) 
                 annotation (Placement(transformation(extent={{40,40},{60,60}},
            rotation=0)));
    Modelica.Blocks.Sources.Step secondary(height=48, startTime=100) 
                 annotation (                       Placement(transformation(
            extent={{-80,0},{-60,20}},    rotation=0)));
    Modelica.Blocks.Sources.Constant primary(k=40) 
                 annotation (                       Placement(transformation(
            extent={{-80,40},{-60,60}},   rotation=0)));
  equation
    connect(powerPlant.terminal, prescribedLoad.terminal) annotation (Line(
        points={{2,10},{20,10}},
        color={0,0,0},
        pattern=LinePattern.None,
        smooth=Smooth.None));
    connect(ramp.y, prescribedLoad.P) annotation (Line(
        points={{61,50},{70,50},{70,10},{41,10}},
        color={0,0,127},
        pattern=LinePattern.None,
        smooth=Smooth.None));
    connect(schedule.y, powerPlant.plantDispatch[1]) 
                                                  annotation (Line(
        points={{-59,-30},{-30,-30},{-30,3.33333},{-20,3.33333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(secondary.y, powerPlant.plantDispatch[2]) 
                                                 annotation (Line(
        points={{-59,10},{-40,10},{-40,4},{-20,4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(primary.y, powerPlant.plantDispatch[3]) annotation (Line(
        points={{-59,50},{-30,50},{-30,4.66667},{-20,4.66667}},
        color={0,0,127},
        smooth=Smooth.None));
  end PowerPlantTest2;

  model PowerPlantTest3 "Test connection to a large net"
    import PowerFlow;
    annotation (
      experiment(StopTime=600),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                           graphics));
    PowerFlow.Units.PowerPlant powerPlant 
                                     annotation (Placement(transformation(
            extent={{-20,0},{2,22}}, rotation=0)));
    Modelica.Blocks.Sources.Constant schedule(k=490) 
      annotation (Placement(transformation(extent={{-80,-40},{-60,-20}},
            rotation=0)));
    PowerFlow.Sources.FixedVoltageSource prescribedLoad 
                                          annotation (Placement(
          transformation(extent={{80,0},{60,20}}, rotation=0)));
    Modelica.Blocks.Sources.Ramp secondary(
      startTime=100,
      height=300,
      duration=0) 
                 annotation (                       Placement(transformation(
            extent={{-80,0},{-60,20}},    rotation=0)));
    PowerFlow.Components.Impedance line(R=1, L=1/50) 
      annotation (Placement(transformation(extent={{20,0},{40,20}})));
    Modelica.Blocks.Sources.Constant primary(k=40) 
                 annotation (                       Placement(transformation(
            extent={{-80,40},{-60,60}},   rotation=0)));
  equation
    connect(schedule.y, powerPlant.plantDispatch[1]) 
                                                  annotation (Line(
        points={{-59,-30},{-30,-30},{-30,3.33333},{-20,3.33333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(secondary.y, powerPlant.plantDispatch[2]) 
                                                 annotation (Line(
        points={{-59,10},{-40,10},{-40,4},{-20,4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(powerPlant.terminal, line.terminal_p)      annotation (Line(
        points={{2,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(line.terminal_n, prescribedLoad.terminal)      annotation (Line(
        points={{40,10},{60,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(primary.y, powerPlant.plantDispatch[3]) annotation (Line(
        points={{-59,50},{-30,50},{-30,4.66667},{-20,4.66667}},
        color={0,0,127},
        smooth=Smooth.None));
  end PowerPlantTest3;

  model HydroPlantTest1 "Test primary control"
    import PowerFlow;
    annotation (
      experiment(StopTime=900),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                           graphics));
    PowerFlow.Units.HydroPlant hydroPlant(primaryControlMax=310) 
                                     annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
    Modelica.Blocks.Sources.Constant schedule(k=50) 
      annotation (Placement(transformation(extent={{-80,-40},{-60,-20}},
            rotation=0)));
    PowerFlow.Sources.PrescribedPowerLoad prescribedLoad 
                                          annotation (Placement(
          transformation(extent={{20,0},{40,20}}, rotation=0)));
    Modelica.Blocks.Sources.Trapezoid trapezoid(
      startTime=100,
      offset=50,
      amplitude=300,
      width=300,
      falling=300,
      nperiod=1,
      rising=60,
      period=900) 
                 annotation (Placement(transformation(extent={{40,40},{60,
              60}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant primary(k=400) 
      annotation (Placement(transformation(extent={{-80,40},{-60,60}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant secondary(k=0) 
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
  equation
    connect(trapezoid.y, prescribedLoad.P) 
                                      annotation (Line(
        points={{61,50},{80,50},{80,10},{41,10}},
        color={0,0,127},
        pattern=LinePattern.None,
        smooth=Smooth.None));
    connect(schedule.y, hydroPlant.hydroDispatch[1]) 
                                                  annotation (Line(
        points={{-59,-30},{-40,-30},{-40,9.33333},{-20,9.33333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroPlant.terminal, prescribedLoad.terminal) annotation (Line(
        points={{0,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(primary.y, hydroPlant.hydroDispatch[3]) 
                                                   annotation (Line(
        points={{-59,50},{-40,50},{-40,10.6667},{-20,10.6667}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(secondary.y, hydroPlant.hydroDispatch[2]) annotation (Line(
        points={{-59,10},{-20,10}},
        color={0,0,127},
        smooth=Smooth.None));
  end HydroPlantTest1;

  model HydroPlantTest2 "Test secondary control"
    import PowerFlow;
    annotation (
      experiment(StopTime=86400),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                           graphics));
    PowerFlow.Units.HydroPlant hydroPlant 
                                     annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
    Modelica.Blocks.Sources.Constant schedule(k=50) 
      annotation (Placement(transformation(extent={{-80,-40},{-60,-20}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant primary(k=400) 
      annotation (Placement(transformation(extent={{-80,40},{-60,60}},
            rotation=0)));
    Modelica.Blocks.Sources.Trapezoid secondary(
      amplitude=100,
      offset=-50,
      rising=86400/4,
      width=86400/4,
      falling=86400/4,
      period=86400,
      startTime=86400/8) 
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Sources.FixedVoltageSource largeNet 
                                          annotation (Placement(
          transformation(extent={{80,0},{60,20}}, rotation=0)));
    PowerFlow.Components.Impedance line(R=1, L=1/50) 
      annotation (Placement(transformation(extent={{20,0},{40,20}})));
  equation
    connect(schedule.y, hydroPlant.hydroDispatch[1]) 
                                                  annotation (Line(
        points={{-59,-30},{-40,-30},{-40,9.33333},{-20,9.33333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(primary.y, hydroPlant.hydroDispatch[3]) 
                                                   annotation (Line(
        points={{-59,50},{-40,50},{-40,10.6667},{-20,10.6667}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(secondary.y, hydroPlant.hydroDispatch[2]) annotation (Line(
        points={{-59,10},{-20,10}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(line.terminal_n, largeNet.terminal)            annotation (Line(
        points={{40,10},{60,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(hydroPlant.terminal, line.terminal_p) annotation (Line(
        points={{0,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
  end HydroPlantTest2;

  model WindFarmLoadTest "WindFarm connected to a load"
    import PowerFlow;
    annotation (
      experiment(StopTime=86400),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
              graphics));
    PowerFlow.Units.WindFarm windFarm 
                                 annotation (Placement(transformation(extent=
              {{-60,0},{-40,20}}, rotation=0)));
    PowerFlow.Components.Impedance load(R=30, L=10/50) 
                                   annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
    PowerFlow.Components.Ground ground 
                              annotation (Placement(transformation(extent={{20,0},{
              40,20}},        rotation=0)));
  equation
    connect(load.terminal_n,ground. terminal) 
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    connect(windFarm.terminal, load.terminal_p) annotation (Line(
        points={{-40,10},{-20,10}},
        color={0,0,0},
        smooth=Smooth.None));
  end WindFarmLoadTest;

  model WindFarmNetTest "WindFarm connected to a large net"
    import PowerFlow;
    annotation (
      experiment(StopTime=86400),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
              graphics));
    PowerFlow.Units.WindFarm windFarm 
                                 annotation (Placement(transformation(extent=
              {{-60,0},{-40,20}}, rotation=0)));
    PowerFlow.Sources.FixedVoltageSource largeNet 
      annotation (Placement(transformation(extent={{40,0},{20,20}})));
  equation
    connect(windFarm.terminal, largeNet.terminal) annotation (Line(
        points={{-40,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
  end WindFarmNetTest;

  model WindFarmHVDCTest "WindFarm connected to a large net via HVDC"
    import PowerFlow;
    annotation (
      experiment(StopTime=86400),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
              graphics));
    PowerFlow.Units.WindFarm windFarm(redeclare package PhaseSystem = 
          PowerFlow.PhaseSystems.DirectCurrent) 
                                 annotation (Placement(transformation(extent=
              {{-60,0},{-40,20}}, rotation=0)));
    PowerFlow.Sources.FixedVoltageSource largeNet 
      annotation (Placement(transformation(extent={{40,0},{20,20}})));
    PowerFlow.Components.Inverter inverter 
      annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  equation
    connect(windFarm.terminal, inverter.terminal_dc) annotation (Line(
        points={{-40,10},{-20,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(inverter.terminal, largeNet.terminal) annotation (Line(
        points={{0,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
  end WindFarmHVDCTest;

  model CityTest
    import PowerFlow;
    annotation (
      experiment(StopTime=86400),
      experimentSetupOutput,
      Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
              graphics));
    PowerFlow.Sources.FixedVoltageSource largeNet 
                                    annotation (Placement(transformation(
            extent={{-60,0},{-40,20}},
                                     rotation=0)));
    PowerFlow.Units.City city 
      annotation (Placement(transformation(extent={{20,0},{40,20}})));
  equation
    connect(largeNet.terminal, city.terminal) annotation (Line(
        points={{-40,10},{20,10}},
        color={0,0,0},
        smooth=Smooth.None));
  end CityTest;

  model LoadDispatcherTest
    import PowerFlow;
    PowerFlow.Units.LoadDispatcher loadDispatcher 
      annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
    Modelica.Blocks.Sources.Trapezoid frequency(
      amplitude=2,
      rising=15,
      falling=15,
      period=60,
      offset=49,
      width=15) 
      annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
              {{-100,-100},{100,100}}), graphics));
  equation
    connect(frequency.y, loadDispatcher.frequency) annotation (Line(
        points={{-59,30},{-30,30},{-30,17}},
        color={0,0,127},
        smooth=Smooth.None));
  end LoadDispatcherTest;
end Units;
