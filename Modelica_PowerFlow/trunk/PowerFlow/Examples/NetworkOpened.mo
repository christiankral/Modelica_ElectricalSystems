within PowerFlow.Examples;
model NetworkOpened
  "see Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.6: Leistungsfluss in Ringnetzen"
  extends Modelica.Icons.Example;

  Sources.FixedVoltageSource fixedVoltageSource1(V=10e3)
    annotation (Placement(transformation(
        origin={-50,70},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Sources.FixedVoltageSource fixedVoltageSource2(V=10e3)
                  annotation (Placement(transformation(
        origin={50,70},
        extent={{-10,-10},{10,10}},
        rotation=270)));
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
  connect(fixedVoltageSource1.terminal, transformer1.terminal_p)
    annotation (Line(points={{-50,60},{-50,40}}, color={0,0,0}));
  connect(transformer1.terminal_n, impedance1.terminal_p)
    annotation (Line(points={{-50,20},{-50,0}}, color={0,0,0}));
  connect(fixedVoltageSource2.terminal, transformer2.terminal_p)
    annotation (Line(points={{50,60},{50,40}}, color={0,0,0}));
  connect(transformer2.terminal_n, impedance5.terminal_p)
    annotation (Line(points={{50,20},{50,0}}, color={0,0,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),
                      graphics),
                       experiment(StopTime=1));
end NetworkOpened;
