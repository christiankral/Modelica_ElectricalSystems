within PowerFlow;
package Units

  model PowerPlant "Thermal Power Plant with primary and secondary control"
    extends Interfaces.PartialSource(final potentialReference=false);

    parameter Boolean Modakond = false "Install Modakond for condensate stop"
      annotation(Dialog(group="Features"));
    parameter Real primaryControlMax(unit="MW") = 40
      "Maximum power for primary frequency control"
      annotation(Dialog(group="Features"));

    Real P_control(unit="MW") = generator.S[1]/1e6 - plantDispatch[1];

    Components.EMF generator(redeclare package PhaseSystem = PhaseSystem,
        definiteReference=definiteReference)
                                   annotation (
        Placement(transformation(extent={{60,-20},{80,0}},  rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia rotor(
                     J=10e6, w(fixed=false, start=50))
      annotation (                        Placement(transformation(extent={{30,-20},
              {50,0}},  rotation=0)));
    Modelica.Mechanics.Rotational.Sources.Torque turbine
      annotation (                       Placement(transformation(extent={{0,-20},
              {20,0}},  rotation=0)));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor frequency
      annotation (                         Placement(transformation(extent={{46,4},{
              34,16}},  rotation=0)));
    Modelica.Blocks.Sources.Constant reference(k=50)
      annotation (                         Placement(transformation(extent={{46,24},
              {34,36}},  rotation=0)));
    Modelica.Blocks.Continuous.LimPID primaryControl(
                                             k=0.05*800/0.2,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      yMax=primaryControlMax) "UCTE: df of 200 mHz corresponds to 5% of load"
      annotation (                         Placement(transformation(extent={{20,20},
              {0,40}},  rotation=0)));
    Modelica.Blocks.Interfaces.RealInput[3] plantDispatch(each unit="MW")
      annotation (                            Placement(transformation(extent={{-130,
              -70},{-110,-50}}, rotation=0)));
    Modelica.Blocks.Math.Add loadControl(k2=1e6/50, k1=1e6/50)
      annotation (                          Placement(transformation(extent={{-30,-20},
              {-10,0}},  rotation=0)));
    Modelica.Blocks.Continuous.FirstOrder evaporator(T=60, y_start=490)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-90,10})));
    Modelica.Blocks.Continuous.FirstOrder superheater1(T=60, y_start=490)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-90,50})));
    Modelica.Blocks.Continuous.FirstOrder superheater2(T=60, y_start=490)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-90,90})));
    Modelica.Blocks.Math.Add fuel annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-90,-30})));
    Modelica.Blocks.Continuous.TransferFunction transferFunction(b={180,1}, a=
         {60,1}) "achieve a power ramp of 2% per minute, minus 0.8% tolerance"
      annotation (Placement(transformation(extent={{-76,-86},{-64,-74}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-96,-86},{-84,-74}})));
    Modelica.Blocks.Continuous.LimIntegrator hotwellLevel(
      k=1,
      outMax=10,
      outMin=0,
      y(unit="m")) "level of hotwell"
      annotation (Placement(transformation(extent={{50,-70},{70,-50}})));
    Modelica.Blocks.Math.Max on
      annotation (Placement(transformation(extent={{4,-46},{16,-34}})));
    Modelica.Blocks.Math.Sign condStop
      annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
    Modelica.Blocks.Math.Min off
      annotation (Placement(transformation(extent={{4,-86},{16,-74}})));
    Modelica.Blocks.Math.Add accumulation(k2=1e-2, k1=if Modakond then 5e-4 else
                0)
      annotation (Placement(transformation(extent={{24,-66},{36,-54}})));
    Modelica.Blocks.Sources.Constant zero(k=0)
      annotation (Placement(transformation(extent={{16,-66},{4,-54}})));
    Modelica.Blocks.Continuous.FirstOrder throttleDynamics(T=120, k=-300/800)
      annotation (Placement(transformation(extent={{-12,68},{0,80}})));
    Modelica.Blocks.Math.Add pressure(k1=300/800)
      annotation (Placement(transformation(extent={{60,90},{80,110}})));
    Modelica.Blocks.Math.Min throttling
      annotation (Placement(transformation(extent={{-32,68},{-20,80}})));
    Modelica.Blocks.Sources.Constant throttleMin(k=if Modakond then 0 else
          Modelica.Constants.inf)
      annotation (Placement(transformation(extent={{-20,88},{-32,100}})));
    Modelica.Blocks.Sources.Constant throttleReserve(k=if Modakond then 0 else
                5)
      annotation (                         Placement(transformation(extent={{-12,88},
              {0,100}},  rotation=0)));
    Modelica.Blocks.Math.Add pressureLoss
      annotation (Placement(transformation(extent={{20,70},{40,90}})));
    Modelica.Blocks.Continuous.Integrator throttleCosts(k=1/365)
      annotation (Placement(transformation(extent={{60,60},{80,80}})));
  equation
    connect(rotor.flange_b, frequency.flange)   annotation (Line(
        points={{50,-10},{54,-10},{54,10},{46,10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(primaryControl.y, loadControl.u1)
                                       annotation (Line(
        points={{-1,30},{-40,30},{-40,-4},{-32,-4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(evaporator.y, superheater1.u)
                                  annotation (Line(
        points={{-90,21},{-90,38}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(superheater1.y, superheater2.u)
                                  annotation (Line(
        points={{-90,61},{-90,78}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(superheater2.y, loadControl.u2)
                                 annotation (Line(
        points={{-90,101},{-90,110},{-60,110},{-60,-16},{-32,-16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(loadControl.y, turbine.tau)
                                  annotation (Line(
        points={{-9,-10},{-2,-10}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(turbine.flange, rotor.flange_a)   annotation (Line(
        points={{20,-10},{30,-10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotor.flange_b, generator.flange)   annotation (Line(
        points={{50,-10},{60,-10}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(fuel.y, evaporator.u)
                               annotation (Line(
        points={{-90,-19},{-90,-2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(transferFunction.y, fuel.u2) annotation (Line(
        points={{-63.4,-80},{-60,-80},{-60,-60},{-84,-60},{-84,-42}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(add.y, transferFunction.u) annotation (Line(
        points={{-83.4,-80},{-77.2,-80}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(loadControl.u1, add.u2) annotation (Line(
        points={{-32,-4},{-40,-4},{-40,-90},{-104,-90},{-104,-83.6},{-97.2,
            -83.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(accumulation.y, hotwellLevel.u)
                                   annotation (Line(
        points={{36.6,-60},{48,-60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(loadControl.u1, condStop.u) annotation (Line(
        points={{-32,-4},{-40,-4},{-40,-60},{-32,-60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(condStop.y, on.u1) annotation (Line(
        points={{-9,-60},{-4,-60},{-4,-36.4},{2.8,-36.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(condStop.y, off.u2) annotation (Line(
        points={{-9,-60},{-4,-60},{-4,-83.6},{2.8,-83.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(zero.y, on.u2) annotation (Line(
        points={{3.4,-60},{0,-60},{0,-43.6},{2.8,-43.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(zero.y, off.u1) annotation (Line(
        points={{3.4,-60},{0,-60},{0,-76.4},{2.8,-76.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(on.y, accumulation.u1) annotation (Line(
        points={{16.6,-40},{20,-40},{20,-56.4},{22.8,-56.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(off.y, accumulation.u2) annotation (Line(
        points={{16.6,-80},{20,-80},{20,-63.6},{22.8,-63.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(primaryControl.y, throttling.u2) annotation (Line(
        points={{-1,30},{-40,30},{-40,70.4},{-33.2,70.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(throttleMin.y, throttling.u1) annotation (Line(
        points={{-32.6,94},{-40,94},{-40,77.6},{-33.2,77.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(throttling.y, throttleDynamics.u) annotation (Line(
        points={{-19.4,74},{-13.2,74}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(superheater2.y, pressure.u1) annotation (Line(
        points={{-90,101},{-90,110},{50,110},{50,106},{58,106}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(throttleDynamics.y, pressureLoss.u2) annotation (Line(
        points={{0.6,74},{18,74}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(throttleReserve.y, pressureLoss.u1) annotation (Line(
        points={{0.6,94},{10,94},{10,86},{18,86}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pressureLoss.y, pressure.u2) annotation (Line(
        points={{41,80},{50,80},{50,94},{58,94}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pressureLoss.y, throttleCosts.u) annotation (Line(
        points={{41,80},{50,80},{50,70},{58,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(generator.terminal, terminal) annotation (Line(
        points={{80,-10},{90,-10},{90,5.55112e-16},{100,5.55112e-16}},
        color={0,0,0},
        smooth=Smooth.None));

    connect(plantDispatch[2], add.u1) annotation (Line(
        points={{-120,-60},{-106,-60},{-106,-76.4},{-97.2,-76.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(plantDispatch[1], fuel.u1) annotation (Line(
        points={{-120,-66.6667},{-96,-66.6667},{-96,-42}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(reference.y, primaryControl.u_s) annotation (Line(
        points={{33.4,30},{22,30}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(frequency.w, primaryControl.u_m) annotation (Line(
        points={{33.4,10},{10,10},{10,18}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Icon(graphics={
          Polygon(
            points={{-30,100},{40,100},{60,-100},{-50,-100},{-30,100}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillPattern=FillPattern.VerticalCylinder,
            fillColor={189,189,126}),
          Rectangle(
            extent={{-44,-100},{80,-28}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,127,127},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-100,-100},{100,-140}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%"),
          Rectangle(
            extent={{-100,-100},{-30,50}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={0,127,127},
            fillPattern=FillPattern.Solid)},
        coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{100,
              120}})),       Diagram(coordinateSystem(preserveAspectRatio=true,
                    extent={{-120,-100},{100,120}}), graphics));
  end PowerPlant;

  model HydroPlant
    extends Interfaces.PartialSource(final potentialReference=false);

    parameter Real primaryControlMax(unit="MW") = 200
      "Maximum power for primary frequency control"
      annotation(Dialog(group="Features"));

    Real P_control(unit="MW") = generator.S[1]/1e6 - hydroDispatch[1];

    Modelica.Blocks.Interfaces.RealInput[3] hydroDispatch(each unit="MW")
      annotation (                            Placement(transformation(extent={{-110,
              -10},{-90,10}},   rotation=0)));
    Modelica.Blocks.Continuous.Integrator reservoirLevel(y_start=10, k=-1/5e4,
      y(unit="m"))
      annotation (Placement(transformation(extent={{-30,40},{-10,20}})));
    Modelica.Mechanics.Rotational.Sources.Torque reservoirTurbine
      annotation (                       Placement(transformation(extent={{-10,-10},
              {10,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia rotor(J=1e6, w(fixed=
            false, start=50))
      annotation (                        Placement(transformation(extent={{20,-10},
              {40,10}}, rotation=0)));
    Components.EMF generator(redeclare package PhaseSystem = PhaseSystem,
        definiteReference=definiteReference)
                                   annotation (
        Placement(transformation(extent={{60,-10},{80,10}}, rotation=0)));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor frequency
      annotation (                         Placement(transformation(extent={{36,34},
              {24,46}}, rotation=0)));
    Modelica.Blocks.Sources.Constant reference(k=50)
      annotation (                         Placement(transformation(extent={{36,64},
              {24,76}},  rotation=0)));
    Modelica.Blocks.Continuous.LimPID primaryControl(
      k=50/0.2,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      yMax=primaryControlMax) "200 mHz corresponds to 50 MW"
      annotation (                         Placement(transformation(extent={{10,60},
              {-10,80}},rotation=0)));
    Modelica.Blocks.Math.Gain powerControl(k=1e6/50)
      annotation (                         Placement(transformation(extent={{-36,-6},
              {-24,6}}, rotation=0)));
    Modelica.Blocks.Continuous.TransferFunction controlDynamics(a={1,1}, b={1})
      annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-76,-6},{-64,6}})));
    Modelica.Blocks.Math.Gain riverControl(k=1e6/50)
      annotation (                         Placement(transformation(extent={{-36,-46},
              {-24,-34}},
                        rotation=0)));
    Modelica.Mechanics.Rotational.Sources.Torque riverTurbine
      annotation (                       Placement(transformation(extent={{-10,-50},
              {10,-30}},rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia rotorRiver(w(fixed=false,
          start=50), J=0.5e6)
      annotation (                        Placement(transformation(extent={{20,-50},
              {40,-30}},rotation=0)));
  equation

    connect(rotor.flange_b,frequency. flange)   annotation (Line(
        points={{40,6.10623e-16},{44,6.10623e-16},{44,40},{36,40}},
        color={0,0,0},
        smooth=Smooth.None));

    connect(reservoirTurbine.flange, rotor.flange_a)
                                              annotation (Line(
        points={{10,6.10623e-16},{12.5,6.10623e-16},{12.5,1.22125e-15},{15,
            1.22125e-15},{15,0},{20,0}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotor.flange_b, generator.flange)   annotation (Line(
        points={{40,6.10623e-16},{45,6.10623e-16},{45,1.22125e-15},{50,
            1.22125e-15},{50,0},{60,0}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(generator.terminal, terminal) annotation (Line(
        points={{80,6.10623e-16},{85,6.10623e-16},{85,1.16573e-15},{90,
            1.16573e-15},{90,0},{100,0}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(powerControl.y, reservoirTurbine.tau)
                                         annotation (Line(
        points={{-23.4,-1.88738e-16},{-20.55,-1.88738e-16},{-20.55,4.77396e-16},
            {-17.7,4.77396e-16},{-17.7,0},{-12,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(controlDynamics.y, reservoirLevel.u) annotation (Line(
        points={{-43.4,-1.88738e-16},{-40,-1.88738e-16},{-40,30},{-32,30}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(controlDynamics.y, powerControl.u) annotation (Line(
        points={{-43.4,-1.88738e-16},{-41.85,-1.88738e-16},{-41.85,-3.44169e-16},
            {-40.3,-3.44169e-16},{-40.3,0},{-37.2,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(add.y, controlDynamics.u) annotation (Line(
        points={{-63.4,-1.88738e-16},{-61.85,-1.88738e-16},{-61.85,-3.44169e-16},
            {-60.3,-3.44169e-16},{-60.3,0},{-57.2,0}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroDispatch[2], add.u2) annotation (Line(
        points={{-100,0},{-84,0},{-84,-3.6},{-77.2,-3.6}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(hydroDispatch[1], riverControl.u) annotation (Line(
        points={{-100,-6.66667},{-80,-6.66667},{-80,-40},{-37.2,-40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(riverControl.y, riverTurbine.tau) annotation (Line(
        points={{-23.4,-40},{-12,-40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(frequency.w, primaryControl.u_m)
                                      annotation (Line(
        points={{23.4,40},{-4.44089e-16,40},{-4.44089e-16,58}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(reference.y, primaryControl.u_s)
                                      annotation (Line(
        points={{23.4,70},{12,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(primaryControl.y, add.u1) annotation (Line(
        points={{-11,70},{-80,70},{-80,3.6},{-77.2,3.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(riverTurbine.flange, rotorRiver.flange_a) annotation (Line(
        points={{10,-40},{20,-40}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(rotorRiver.flange_b, generator.flange) annotation (Line(
        points={{40,-40},{50,-40},{50,6.10623e-16},{60,6.10623e-16}},
        color={0,0,0},
        smooth=Smooth.None));
    annotation (Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}), graphics={
          Text(
            extent={{-100,-100},{100,-140}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%"),
          Polygon(
            points={{-18,-18},{16,-56},{36,-34},{76,-36},{18,24},{-104,20},{-18,
                -18}},
            lineColor={0,0,255},
            smooth=Smooth.Bezier,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{14,-44},{34,-86},{54,-66},{82,-68},{66,-28},{14,-44}},
            lineColor={0,0,255},
            smooth=Smooth.Bezier,
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{52,74},{54,50},{96,50},{98,74},{96,98},{54,98},{52,74}},
            lineColor={0,0,255},
            smooth=Smooth.Bezier,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{56,-30},{74,50}},
            color={0,128,255},
            smooth=Smooth.Bezier,
            thickness=1)}),  Diagram(coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
                                     graphics));
  end HydroPlant;

  model WindFarm
    extends Interfaces.PartialSource(final potentialReference=false);
    parameter Boolean cut_out = false
      "stop producing energy as wind exceeds cut-out speed of 20-25 m/s"
      annotation(Dialog(group="Features"));

    Modelica.Blocks.Sources.CombiTimeTable wind(
      tableName="tab",
      fileName="Data/LoadData.txt",
      tableOnFile=true,
      table=fill(
              0.0,
              0,
              10)) annotation (Placement(transformation(extent={{-80,-10},{
              -60,10}}, rotation=0)));
    PowerFlow.Sources.PrescribedPowerSource mills(
      redeclare package PhaseSystem = PhaseSystem,
      definiteReference = definiteReference)
        annotation (Placement(transformation(extent=
             {{10,-10},{30,10}}, rotation=0)));
    Modelica.Blocks.Sources.Trapezoid disturbance(
      rising=120,
      period=86400,
      offset=1,
      width=1800,
      falling=1800,
      startTime=20040,
      amplitude=if cut_out then -1 else 0)
      annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
    Modelica.Blocks.Math.Product product
      annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  equation
    connect(mills.terminal, terminal)
                                     annotation (Line(
        points={{30,6.10623e-16},{47.5,6.10623e-16},{47.5,1.16573e-15},{65,
            1.16573e-15},{65,5.55112e-16},{100,5.55112e-16}},
        color={0,0,0},
        smooth=Smooth.None));

    connect(wind.y[4], product.u2) annotation (Line(
        points={{-59,5.55112e-16},{-40,5.55112e-16},{-40,-6},{-22,-6}},
        color={0,0,127},
        smooth=Smooth.None));

    connect(disturbance.y, product.u1) annotation (Line(
        points={{-39,30},{-30,30},{-30,6},{-22,6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(product.y, mills.P) annotation (Line(
        points={{1,6.10623e-16},{3,6.10623e-16},{3,1.27676e-15},{5,1.27676e-15},
            {5,6.66134e-16},{9,6.66134e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
          Rectangle(
            extent={{-48,24},{-32,-100}},
            lineColor={85,85,255},
            fillColor={85,85,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-44,24},{-98,22},{-46,38},{-28,88},{-32,36},{0,-6},{-44,
                24}},
            lineColor={85,85,255},
            smooth=Smooth.None,
            fillColor={85,85,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{32,24},{48,-100}},
            lineColor={85,85,255},
            fillColor={85,85,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{36,24},{-12,20},{32,38},{54,86},{48,36},{82,-2},{36,24}},
            lineColor={85,85,255},
            smooth=Smooth.None,
            fillColor={85,85,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-100,-100},{100,-140}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%")}),
                             Diagram(coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
                                     graphics));
  end WindFarm;

  model LoadDispatcher

    Modelica.Blocks.Sources.CombiTimeTable data(
      tableName="tab",
      fileName="Data/LoadData.txt",
      tableOnFile=true,
      table=fill(
          0.0,
          0,
          10)) annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    Modelica.Blocks.Interfaces.RealOutput[3] plantDispatch(each unit="MW")
      annotation (Placement(transformation(extent={{60,30},{80,50}}, rotation=
             0)));
    Modelica.Blocks.Interfaces.RealOutput loadForcast(unit="MW")
      annotation (Placement(transformation(extent={{-80,30},{-60,50}},
            rotation=0)));
    Modelica.Blocks.Interfaces.RealOutput windForcast(unit="MW")
      annotation (Placement(transformation(extent={{-80,-30},{-60,-10}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant hydroBase(k=25)
      annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
    Modelica.Blocks.Sources.Constant primaryControl(k=40)
      annotation (Placement(transformation(extent={{20,70},{40,90}})));
    Modelica.Blocks.Interfaces.RealOutput[3] hydroDispatch(each unit="MW")
      annotation (Placement(transformation(extent={{60,-30},{80,-10}},
                                                                     rotation=
             0)));
    Modelica.Blocks.Sources.Constant controlHydro(k=200)
      annotation (Placement(transformation(extent={{80,0},{60,20}})));
    Modelica.Blocks.Interfaces.RealInput frequency(unit="Hz")
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,70})));
    Modelica.Blocks.Continuous.LimPID secondaryControl(   k=60/0.4,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      Ti=600,
      yMax=60) "400 mHz corresponds to 60 MW"
      annotation (Placement(transformation(extent={{20,50},{40,30}})));
    Modelica.Blocks.Sources.Constant reference(k=50)
      annotation (                         Placement(transformation(extent={{-6,34},
              {6,46}},   rotation=0)));
    Modelica.Blocks.Math.Add3 plantSchedule(k2=-1, k3=-1,
      y(unit="MW"))
      annotation (Placement(transformation(extent={{10,-10},{30,10}})));
    Modelica.Blocks.Sources.Trapezoid hydroDaily(
      rising=86400/4,
      width=86400/4,
      falling=86400/4,
      period=86400,
      startTime=86400/8,
      amplitude=50,
      offset=-25,
      y(unit="MW"))
      annotation (Placement(transformation(extent={{-80,-60},{-60,-40}},
            rotation=0)));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-16,-36},{-4,-24}})));
    Modelica.Blocks.Math.Gain distributionLoss(k=1)      annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-16,16})));
  equation
    connect(primaryControl.y, plantDispatch[3]) annotation (Line(
        points={{41,80},{50,80},{50,46.6667},{70,46.6667}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroBase.y, hydroDispatch[1])  annotation (Line(
        points={{-59,-80},{56,-80},{56,-26.6667},{70,-26.6667}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(controlHydro.y, hydroDispatch[3]) annotation (Line(
        points={{59,10},{56,10},{56,-13.3333},{70,-13.3333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(data.y[3], windForcast) annotation (Line(
        points={{-59,10},{-40,10},{-40,-20},{-70,-20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(secondaryControl.y, plantDispatch[2]) annotation (Line(
        points={{41,40},{70,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(windForcast, plantSchedule.u2) annotation (Line(
        points={{-70,-20},{-20,-20},{-20,6.66134e-16},{8,6.66134e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(plantSchedule.y, plantDispatch[1]) annotation (Line(
        points={{31,6.10623e-16},{50,6.10623e-16},{50,33.3333},{70,33.3333}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroDaily.y, hydroDispatch[2]) annotation (Line(
        points={{-59,-50},{48,-50},{48,-20},{70,-20}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroDaily.y, add.u1) annotation (Line(
        points={{-59,-50},{-32,-50},{-32,-26.4},{-17.2,-26.4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hydroBase.y, add.u2) annotation (Line(
        points={{-59,-80},{-26,-80},{-26,-33.6},{-17.2,-33.6}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(add.y, plantSchedule.u3) annotation (Line(
        points={{-3.4,-30},{0,-30},{0,-8},{8,-8}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(frequency, secondaryControl.u_m) annotation (Line(
        points={{5.55112e-16,70},{5.55112e-16,56},{30,56},{30,52}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(reference.y, secondaryControl.u_s) annotation (Line(
        points={{6.6,40},{18,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(data.y[1], loadForcast) annotation (Line(
        points={{-59,10},{-40,10},{-40,40},{-70,40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(loadForcast, distributionLoss.u) annotation (Line(
        points={{-70,40},{-40,40},{-40,16},{-28,16}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(distributionLoss.y, plantSchedule.u1) annotation (Line(
        points={{-5,16},{0,16},{0,8},{8,8}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics={
          Polygon(
            points={{-80,-80},{-60,-40},{60,-40},{80,-80},{-80,-80}},
            lineColor={96,96,96},
            smooth=Smooth.None,
            fillColor={96,96,96},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-60,60},{60,-30}},
            lineColor={96,96,96},
            fillColor={96,96,96},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,50},{48,42}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,30},{18,22}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,10},{40,2}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-48,-12},{36,-20}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-100,-100},{100,-140}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%")}),
                              Diagram(coordinateSystem(preserveAspectRatio=true,
                    extent={{-100,-100},{100,100}}), graphics));
  end LoadDispatcher;

  model City
    extends Interfaces.PartialLoad;
    Modelica.Blocks.Sources.CombiTimeTable data(
      tableName="tab",
      fileName="Data/LoadData.txt",
      tableOnFile=true,
      table=fill(
              0.0,
              0,
              10)) annotation (Placement(transformation(extent={{-60,40},{-40,
              60}}, rotation=0)));
    PowerFlow.Sources.PrescribedPowerLoad load(phi=0.34906585039887)
                                annotation (Placement(transformation(extent={
              {-60,-10},{-40,10}}, rotation=0)));
  equation
    connect(terminal, load.terminal) annotation (Line(
        points={{-100,5.55112e-16},{-90,5.55112e-16},{-90,1.16573e-15},{-80,
            1.16573e-15},{-80,6.10623e-16},{-60,6.10623e-16}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(data.y[2], load.P) annotation (Line(
        points={{-39,50},{-20,50},{-20,6.66134e-16},{-39,6.66134e-16}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
              -100},{100,100}}),
                        graphics),
                         Icon(coordinateSystem(preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
          Polygon(
            points={{-100,-100},{-100,100},{-66,100},{-66,0},{-40,0},{-40,40},
                {0,40},{0,0},{40,0},{40,60},{80,60},{80,-20},{100,-20},{100,-100},
                {-100,-100}},
            lineColor={0,0,0},
            smooth=Smooth.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-92,70},{-74,52}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-92,44},{-74,26}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-28,30},{-10,12}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{52,50},{70,32}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{52,18},{70,0}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-92,-12},{-74,-30}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-62,-28},{-44,-46}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-26,-28},{-8,-46}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{12,-28},{30,-46}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{52,-28},{70,-46}},
            lineColor={0,0,255},
            pattern=LinePattern.None,
            fillColor={255,255,85},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-100,-100},{100,-140}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="%name%")}));
  end City;

end Units;
