within PowerFlow.Test;
package Components
  model ImpedanceTest
    import PowerFlow;

    Sources.FixedVoltageSource source
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Components.Impedance load(R=30, L=10/50)
                                   annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
    PowerFlow.Components.Ground ground
                              annotation (Placement(transformation(extent={{
              40,0},{60,20}}, rotation=0)));
  equation
    connect(source.terminal, load.terminal_p)
      annotation (Line(points={{-60,10},{-20,10}}, color={0,0,0}));
    connect(load.terminal_n, ground.terminal)
      annotation (Line(points={{0,10},{40,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end ImpedanceTest;

  model AdmittanceTest
    import PowerFlow;

    Sources.FixedVoltageSource source
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Components.Admittance load(G=30/(30*30 + 10*10), C=10/(30*30
           + 10*10)/50)            annotation (Placement(transformation(
            extent={{-20,0},{0,20}}, rotation=0)));
    PowerFlow.Components.Ground ground
                              annotation (Placement(transformation(extent={{
              40,0},{60,20}}, rotation=0)));
  equation
    connect(source.terminal, load.terminal_p)
      annotation (Line(points={{-60,10},{-20,10}}, color={0,0,0}));
    connect(load.terminal_n, ground.terminal)
      annotation (Line(points={{0,10},{40,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end AdmittanceTest;

  model InductiveLoadTest
    import PowerFlow;

    Sources.FixedVoltageSource source
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Components.Impedance line(R=1.2, L=1.6/50)
                                   annotation (Placement(transformation(
            extent={{-40,0},{-20,20}}, rotation=0)));
    PowerFlow.Components.Impedance load(R=30, L=10/50)
                                   annotation (Placement(transformation(
            extent={{0,0},{20,20}}, rotation=0)));
    PowerFlow.Components.Ground ground
                             annotation (Placement(transformation(extent={{40,
              0},{60,20}}, rotation=0)));
  equation
    connect(source.terminal, line.terminal_p)
      annotation (Line(points={{-60,10},{-40,10}}, color={0,0,0}));
    connect(load.terminal_n, ground.terminal)
      annotation (Line(points={{20,10},{40,10}}, color={0,0,0}));
    connect(line.terminal_n, load.terminal_p)
      annotation (Line(points={{-20,10},{0,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end InductiveLoadTest;

  model FixedCurrentTest
    import PowerFlow;

    PowerFlow.Sources.FixedVoltageSource source
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Components.Impedance line(R=1.2, L=1.6/50)
                                   annotation (Placement(transformation(
            extent={{-40,0},{-20,20}}, rotation=0)));
    Sources.FixedCurrent load(I=173.448, phi=-0.356)                                         annotation (Placement(
          transformation(extent={{0,0},{20,20}}, rotation=0)));
  equation
    connect(line.terminal_n, load.terminal)
      annotation (Line(points={{-20,10},{0,10}}, color={0,0,0}));
    connect(source.terminal, line.terminal_p)
      annotation (Line(points={{-60,10},{-40,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end FixedCurrentTest;

  model FixedLoadTest
    import PowerFlow;

    Sources.FixedVoltageSource source
      annotation (Placement(transformation(extent={{-80,0},{-60,20}},
            rotation=0)));
    PowerFlow.Components.Impedance line(R=1.2, L=1.6/50)
                               annotation (Placement(transformation(extent={{
              -40,0},{-20,20}}, rotation=0)));
    Sources.FixedLoad load(                       P=2.7076e6, phi=arctan(1000
          /3000))                                                  annotation (Placement(
          transformation(extent={{0,0},{20,20}}, rotation=0)));
  equation
    connect(source.terminal, line.terminal_p)
      annotation (Line(points={{-60,10},{-40,10}}, color={0,0,0}));
    connect(line.terminal_n, load.terminal)
      annotation (Line(points={{-20,10},{0,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end FixedLoadTest;

  model MeshTestEMF
    "see Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.6"
    import PowerFlow;

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
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{-100,110},{-120,130}},
            rotation=0)));
    Modelica.Mechanics.Rotational.Sources.Torque torque1(useSupport=false)
      annotation (Placement(transformation(extent={{10,70},{30,90}}, rotation=
             0)));
    Modelica.Blocks.Sources.Trapezoid trapezoid(
      width=30,
      amplitude=2e4,
      offset=2e4,
      rising=0,
      falling=0,
      period=60) annotation (Placement(transformation(extent={{30,110},{10,
              130}}, rotation=0)));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-150,90},{-130,70}},
            rotation=0)));
    Modelica.Blocks.Sources.Constant const(k=50)
      annotation (Placement(transformation(extent={{-180,70},{-160,90}},
            rotation=0)));
    Modelica.Blocks.Continuous.PID PID(k=1e6/50)
      annotation (Placement(transformation(extent={{-120,70},{-100,90}},
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
    connect(trapezoid.y, torque1.tau) annotation (Line(points={{9,120},{0,120},
            {0,80},{8,80}}, color={0,0,127}));
    connect(speedSensor.flange,   inertia.flange_b) annotation (Line(points={
            {-100,120},{-40,120},{-40,80}}, color={0,0,0}));
    connect(const.y, feedback.u1) annotation (Line(points={{-159,80},{-148,80}},
          color={0,0,127}));
    connect(speedSensor.w, feedback.u2) annotation (Line(points={{-121,120},{
            -140,120},{-140,88}}, color={0,0,127}));
    connect(feedback.y, PID.u) annotation (Line(points={{-131,80},{-122,80}},
          color={0,0,127}));
    connect(PID.y, torque.tau) annotation (Line(points={{-99,80},{-92,80}},
          color={0,0,127}));
    annotation (Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-180,-100},{100,140}},
          initialScale=0.1), graphics),
      experiment(StopTime=120));
  end MeshTestEMF;

  model EMFTest
    import PowerFlow;

    PowerFlow.Components.EMF eMF
                       annotation (Placement(transformation(extent={{-20,0},{
              0,20}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(
                                                  J=1e3, w(start=50))
      annotation (Placement(transformation(extent={{-50,0},{-30,20}},
            rotation=0)));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
                                                                tau_constant=
          3e6/50, useSupport=false)
                  annotation (Placement(transformation(extent={{-90,0},{-70,
              20}}, rotation=0)));
    Sources.FixedLoad fixedLoad(P=3e6,
      phi=0.3)
      annotation (Placement(transformation(extent={{20,0},{40,20}}, rotation=
              0)));
  equation
    connect(inertia.flange_b, eMF.flange) annotation (Line(points={{-30,10},{
            -20,10}}, color={0,0,0}));
    connect(constantTorque.flange, inertia.flange_a) annotation (Line(points=
            {{-70,10},{-50,10}}, color={0,0,0}));
    connect(eMF.terminal, fixedLoad.terminal) annotation (Line(points={{0,10},
            {20,10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end EMFTest;

  model EMFTest2
    import PowerFlow;

    PowerFlow.Components.EMF eMF
                       annotation (Placement(transformation(extent={{-20,0},{
              0,20}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(
                                                  J=1e3, w(start=50))
      annotation (Placement(transformation(extent={{-50,0},{-30,20}},
            rotation=0)));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
                                                                tau_constant=
          3e6/50/2, useSupport=false)
                  annotation (Placement(transformation(extent={{-90,0},{-70,
              20}}, rotation=0)));
    Sources.FixedLoad fixedLoad(P=3e6,
      phi=0.3)
      annotation (Placement(transformation(extent={{60,-20},{80,0}}, rotation=
             0)));
    PowerFlow.Components.EMF eMF1(synchronous=false)
                       annotation (Placement(transformation(extent={{-20,-40},
              {0,-20}}, rotation=0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(
                                                  J=1e3, w(start=50))
      annotation (Placement(transformation(extent={{-50,-40},{-30,-20}},
            rotation=0)));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(
                                                                 tau_constant=
         3e6/50, useSupport=false)
                  annotation (Placement(transformation(extent={{-90,-40},{-70,
              -20}}, rotation=0)));
    PowerFlow.Components.Impedance impedance
                                   annotation (Placement(transformation(
            extent={{20,0},{40,20}}, rotation=0)));
    PowerFlow.Components.Impedance impedance1
                                    annotation (Placement(transformation(
            extent={{20,-40},{40,-20}}, rotation=0)));
  equation
    connect(inertia.flange_b, eMF.flange) annotation (Line(points={{-30,10},{
            -20,10}}, color={0,0,0}));
    connect(constantTorque.flange, inertia.flange_a) annotation (Line(points=
            {{-70,10},{-50,10}}, color={0,0,0}));
    connect(inertia1.flange_b, eMF1.flange)
                                          annotation (Line(points={{-30,-30},
            {-20,-30}}, color={0,0,0}));
    connect(constantTorque1.flange, inertia1.flange_a)
                                                     annotation (Line(points=
            {{-70,-30},{-50,-30}}, color={0,0,0}));
    connect(eMF.terminal, impedance.terminal_p)
      annotation (Line(points={{0,10},{20,10}}, color={0,0,0}));
    connect(impedance.terminal_n, fixedLoad.terminal) annotation (Line(points=
           {{40,10},{50,10},{50,-10},{60,-10}}, color={0,0,0}));
    connect(eMF1.terminal, impedance1.terminal_p)
      annotation (Line(points={{0,-30},{20,-30}}, color={0,0,0}));
    connect(impedance1.terminal_n, fixedLoad.terminal) annotation (Line(
          points={{40,-30},{50,-30},{50,-10},{60,-10}}, color={0,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                        graphics),
                         experiment(StopTime=1));
  end EMFTest2;
end Components;
