within PowerFlow.Examples;
model NetworkControlled
  "see Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.6: Leistungsfluss in Ringnetzen"
  extends Modelica.Icons.Example;

  PowerFlow.Components.Impedance impedance1(R=2, L=0) 
    annotation (Placement(transformation(
        origin={-50,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  PowerFlow.Components.Impedance impedance2(L=0, R=4) 
    annotation (Placement(transformation(
        origin={-50,-50},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  PowerFlow.Components.Impedance impedance3(R=2, L=0) 
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}},
          rotation=0)));
  PowerFlow.Components.Impedance impedance4(L=0, R=1) 
    annotation (Placement(transformation(
        origin={50,-50},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  PowerFlow.Components.Impedance impedance5(L=0, R=3) 
    annotation (Placement(transformation(
        origin={50,-10},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  annotation (Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-180,-100},{100,140}},
        initialScale=0.1), graphics),
    experiment(StopTime=120));
  Sources.FixedCurrent fixedCurrent3(I=50) annotation (Placement(
        transformation(extent={{70,-90},{90,-70}}, rotation=0)));
  Sources.FixedCurrent fixedCurrent1(I=55) annotation (Placement(
        transformation(extent={{-70,-40},{-90,-20}}, rotation=0)));
  Sources.FixedCurrent fixedCurrent2(I=45) 
    annotation (Placement(transformation(extent={{-70,-90},{-90,-70}},
          rotation=0)));
  Sources.FixedCurrent fixedCurrent4(I=60) annotation (Placement(
        transformation(extent={{70,-40},{90,-20}}, rotation=0)));
  PowerFlow.Components.VoltageConverter transformer1(ratio=10/10.4156) 
    annotation (Placement(transformation(
        origin={-50,30},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  PowerFlow.Components.VoltageConverter transformer2(ratio=10/10) 
    annotation (Placement(transformation(
        origin={50,30},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  PowerFlow.Components.EMF eMF   annotation (Placement(transformation(
          extent={{-30,70},{-10,90}}, rotation=0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
                                                J=1e3, w(start=50)) 
    annotation (Placement(transformation(extent={{-60,70},{-40,90}},
          rotation=0)));
  PowerFlow.Components.EMF eMF1 
                              annotation (Placement(transformation(extent={
            {70,70},{90,90}}, rotation=0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(
                                                 J=1e3, w(start=50)) 
    annotation (Placement(transformation(extent={{40,70},{60,90}}, rotation=
           0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=false) 
    annotation (Placement(transformation(extent={{-90,70},{-70,90}},
          rotation=0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor frequency 
    annotation (Placement(transformation(extent={{-70,110},{-90,130}},
          rotation=0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1(useSupport=false) 
    annotation (Placement(transformation(extent={{10,70},{30,90}}, rotation=
           0)));
  Modelica.Blocks.Sources.Trapezoid disturbance(
    width=30,
    amplitude=2e4,
    offset=2e4,
    rising=0,
    falling=0,
    period=60) annotation (Placement(transformation(extent={{30,110},{10,
            130}}, rotation=0)));
  Modelica.Blocks.Sources.Constant const(k=50) 
    annotation (Placement(transformation(extent={{-160,70},{-140,90}},
          rotation=0)));
  Modelica.Blocks.Continuous.LimPID frequencyPowerControl(
    k=1e6/50,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=120,
    Td=0,
    yMax=1e5) 
    annotation (Placement(transformation(extent={{-120,90},{-100,70}},
          rotation=0)));
equation
  connect(impedance1.terminal_n, impedance2.terminal_p) 
    annotation (Line(points={{-50,-20},{-50,-40}}, color={0,0,0}));
  connect(impedance2.terminal_n, impedance3.terminal_p) annotation (Line(
        points={{-50,-60},{-50,-80},{-10,-80}}, color={0,0,0}));
  connect(impedance3.terminal_n, impedance4.terminal_n) annotation (Line(
        points={{10,-80},{50,-80},{50,-60}}, color={0,0,0}));
  connect(impedance4.terminal_p, impedance5.terminal_n) 
    annotation (Line(points={{50,-40},{50,-20}}, color={0,0,0}));
  connect(impedance3.terminal_n, fixedCurrent3.terminal) 
    annotation (Line(points={{10,-80},{70,-80}}, color={0,0,0}));
  connect(fixedCurrent1.terminal, impedance1.terminal_n) annotation (Line(
        points={{-70,-30},{-50,-30},{-50,-20}}, color={0,0,0}));
  connect(fixedCurrent2.terminal, impedance3.terminal_p) 
    annotation (Line(points={{-70,-80},{-10,-80}}, color={0,0,0}));
  connect(fixedCurrent4.terminal, impedance5.terminal_n) annotation (Line(
        points={{70,-30},{50,-30},{50,-20}}, color={0,0,0}));
  connect(transformer1.terminal_n, impedance1.terminal_p) 
    annotation (Line(points={{-50,20},{-50,0}}, color={0,0,0}));
  connect(transformer2.terminal_n, impedance5.terminal_p) 
    annotation (Line(points={{50,20},{50,0}}, color={0,0,0}));
  connect(inertia.flange_b, eMF.flange) annotation (Line(points={{-40,80},{
          -30,80}}, color={0,0,0}));
  connect(eMF.terminal, transformer1.terminal_p) annotation (Line(points={{
          -10,80},{-10,60},{-50,60},{-50,40}}, color={0,0,0}));
  connect(inertia1.flange_b, eMF1.flange) annotation (Line(points={{60,80},
          {70,80}}, color={0,0,0}));
  connect(eMF1.terminal, transformer2.terminal_p) annotation (Line(points={
          {90,80},{90,60},{50,60},{50,40}}, color={0,0,0}));
  connect(torque.flange,   inertia.flange_a) 
    annotation (Line(points={{-70,80},{-60,80}}, color={0,0,0}));
  connect(torque1.flange,   inertia1.flange_a) 
    annotation (Line(points={{30,80},{40,80}}, color={0,0,0}));
  connect(disturbance.y, torque1.tau) 
                                    annotation (Line(points={{9,120},{0,120},
          {0,80},{8,80}}, color={0,0,127}));
  connect(frequency.flange, inertia.flange_b)     annotation (Line(points={{-70,120},
          {-40,120},{-40,80}},            color={0,0,0}));
  connect(frequencyPowerControl.y, torque.tau) 
                             annotation (Line(points={{-99,80},{-92,80}},
        color={0,0,127}));
  connect(const.y, frequencyPowerControl.u_s) annotation (Line(
      points={{-139,80},{-122,80}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(frequency.w, frequencyPowerControl.u_m) annotation (Line(
      points={{-91,120},{-110,120},{-110,92}},
      color={0,0,127},
      smooth=Smooth.None));
end NetworkControlled;
