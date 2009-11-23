within ;
package PowerFlow "Library for electrical power flow calculations"

  annotation (version="0.3", uses(Modelica(version="3.0")));
package Rationale "Documentation of main concepts"

  annotation (DocumentationClass = true, Documentation(info="<HTML>
<h1><font color=\"#008000\" size=4>Overview</font></h1>
<p>
PowerFlow provides basic models and a general connector for the electrical side of power plant systems. 
Currently it allows to treat symmetrically loaded three phase systems and direct current lines in one framework.
PowerFlow is used to investigate the test case \"Stabilization of wind power\" in the Eurosyslib work package 5.3. 
</p>
<p>
The PowerFlow connector is motivated by the SPOT library and by the concept of replaceable Modelica.Media packages for fluid models.
The aim is to support different single and polyphase systems and different mathematical formulations in one framework. 
This may cover applications like: 
</p>
<ul>
<li> Three phase AC transmission lines, using natural or modal coordinates,
<li> Single phase AC applications,
<li> Variable freqency systems, e.g. to control the power of a motor with a frequency converter, and
<li> Direct current lines, like HVDC
</ul>
<p>
A general terminal for electrical power systems can be defined as:
<pre>
connector Terminal \"General power terminal\" 
  replaceable package PhaseSystem = PhaseSystems.PartialPhaseSystem \"Phase system\"; 
  PhaseSystem.Voltage v[PhaseSystem.n] \"voltage vector\";
  flow PhaseSystem.Current i[PhaseSystem.n] \"current vector\";
  PhaseSystem.ReferenceAngle theta[PhaseSystem.m] \"vector of phase angles\";
end Terminal;
</pre>
The replaceable PhaseSystem defines the number <tt><b>n</b></tt> of independent voltage and current components and 
their representation in the connector. Moreover it defines types for the physical quantities so that terminals of 
different phase systems cannot be directly connected.
</p>
<p>
The vector of reference angles <tt><b>theta[m]</b></tt> allows the definition of a rotating reference system for the description of 
AC systems with modal components. It is known from the SPOT library that this simplifies the treatment of sinosodial quantities 
in the time domain. The power Terminal is overdetermined with the reference angles though and the operators 
Connections.root, Connections.potentialRoot, Connections.isRoot and Connections.branch are used for their implementation. 
A Modelica tool needs to analyze connection graphs and eliminate redundant equations.
</p>
<h1><font color=\"#008000\" size=4>Existing electrical libraries</font></h1>
<p>
<b>Modelica.Electrical</b> describes voltages and currents using natural coordinates in the time domain, i.e. m=0.
It defines a distinct MultiPhase sublibrariy for three phase systems (n=3). 
This has the drawback that sinusoidal quantities of AC systems with frequencies of e.g. 50 Hz 
complicate the numerical analysis as electrical quantities vary with a period of 20 ms while the system dynamics of interest
may be in the order of seconds to minutes.
</p>
<p>
<b>ObjectStab</b> uses complex current and voltage phasors for the analysis of symmetrically loaded three phase systems with constant frequency, i.e. n=2, m=0.
The freqency is a global property. This complicates the analysis of power/frequency control and of drives with frequency 
converters.
</p>
<p>
<b>Complex</b> combines the concepts of Modelica.Electrical and ObjectStab. 
It uses complex voltage and current phasors with constant frequency (m=0).
Similar to Modelica.Electrical it defines different sublibraries for one phase systems (n=1) and three phase systems (n=3).
</p>
<p>
<b>SPOT</b> allows the detailed and efficient analysis of three phase systems. 
It provides the most comprehensive collection of models for power electronics currently available in Modelica.
SPOT provides three sublibraries for three phase systems with modal dq0 coordinates (n=3, m=2), natural abc coordinates (n=3, m=2) 
as well as for DC systems (n=1, m=0).
</p>
<h1><font color=\"#008000\" size=4>PhaseSystems and generic component models</font></h1>
<p>
It is the hope that the valuable concepts of different existing libraries can be combined into one 
and that at least basic interfaces and components can be shared.
The current version of the PowerFlow library concentrates on the not yet available symetrically loaded three phase systems with modal coordinates.
This is provided by the <tt>ThreePhaseSymmetric</tt> phase system (n=2, m=1). Moreover the <tt>DirectCurrent</tt> phase system (n=1, m=0) 
is provided to investigate the sharing of connectors and component models.
</p>
A PhaseSystem is a package that provides types, functions and constants. 
A generic steady-state impedance model, which is independent of the phase system in use, can be formulated as:
<pre>
model GenericImpedance
  replaceable package PhaseSystem = PackagePhaseSystem \"Phase system\"; 
 
  function j = PhaseSystem.j;
 
  Terminal terminal_p(redeclare package PhaseSystem = PhaseSystem);
  Terminal terminal_n(redeclare package PhaseSystem = PhaseSystem);
 
  PhaseSystem.Voltage v[:] = terminal_p.v - terminal_n.v;
  PhaseSystem.Current i[:] = terminal_p.i;
  PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));
 
  parameter Modelica.SIunits.Resistance R = 1 \"active component\";
  parameter Modelica.SIunits.Inductance L = 1/50 \"reactive component\";
equation 
  v = R*i + w*L*j(i);
  zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
  terminal_p.theta = terminal_n.theta;
  Connections.branch(terminal_p.theta, terminal_n.theta);
end GenericImpedance;
</pre>
</p>
<h1><font color=\"#008000\" size=4>Examples</font></h1>
<p>
The examples NetworkLoop and NetworkOpened are taken from the textbook Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.5: Leistungsfluss in Ringnetzen. 
The example NetworkControlled additionally investigates frequency/power control in conjunction with the Modelica.Rotational library and a basic EMF (Electro-Motoric Force).
</p>
<p>
The PowerWorld example models a control area for power distribution in island mode.
It is used to investigate the test case \"Stabilization of wind power\" in the Eurosyslib work package 5.3.
See <a href=\"Modelica://PowerFlow.Examples.PowerWorld\">Examples.PowerWorld</a>.
</p>
<h1><font color=\"#008000\" size=4>Tests</font></h1>
<p>
The component tests are taken from the textbook Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14: Leistungsfluss im Drehstromnetz. 
</p>
<hr>
<p>
Copyright &copy; 2007-2009, ABB AG.
</p>
<p>
<i>This Modelica package is <b>Open Source</b> software; it can be redistributed and/or modified
under the terms of the <b>Modelica license, version 2.0, 
see the license conditions and the accompanying <b>disclaimer</b> 
<a href=\"Modelica://Modelica.UsersGuide.ModelicaLicense\">here</a>.</i>
</p>
<p><i>
This work was in parts supported by the ITEA2 EUROSYSLIB project 
(http://www.itea2.org/public/project_leaflets/EUROSYSLIB_profile_oct-07.pdf) 
by funding of BMBF under contract number ITEA 2 - 06020.
</i></p>
<hr>
</HTML>",
  revisions="<html>
<ul>
<li><i>26 Feb 2009</i>
    by <a href=\"mailto:Ruediger.Franke@de.abb.com\">Ruediger Franke</a>:<br>
     Version 0.3
  <ul>
  <li>Generalize power Terminal with n voltages/currents and m reference angles</li>
  <li>Add PowerWorld example</li>
  </ul>
<li><i>21 Oct 2008</i>
    by <a href=\"mailto:Ruediger.Franke@de.abb.com\">Ruediger Franke</a>:<br>
     Version 0.2
  <ul>
  <li>Replace balanced, non-minimal connector with overdetermined minimal connector</li>
  <li>Remove instance variables BaseProperties from PhaseSystems and introduce member functions instead</li>
  </ul>
<li><i>15 Aug 2008</i>
    by <a href=\"mailto:Ruediger.Franke@de.abb.com\">Ruediger Franke</a>:<br>
     Version 0.1
</li>
</ul>
</html>"));
end Rationale;

  replaceable package PackagePhaseSystem = 
      PowerFlow.PhaseSystems.ThreePhaseSymmetric "Default phase system" 
    annotation (choicesAllMatching=true);

  package Examples

    model NetworkLoop
      "see Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.6: Leistungsfluss in Ringnetzen"
      extends Modelica.Icons.Example;

      Sources.FixedVoltageSource fixedVoltageSource1(V=10e3) 
        annotation (Placement(transformation(
            origin={0,70},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      PowerFlow.Components.Impedance impedance1(R=2, L=0) 
        annotation (Placement(transformation(
            origin={-50,-10},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      PowerFlow.Components.Impedance impedance2(R=4, L=0) 
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
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           experiment(StopTime=1));
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
      connect(impedance4.terminal_p, impedance5.terminal_n) 
        annotation (Line(points={{50,-40},{50,-20}}, color={0,0,0}));
      connect(fixedCurrent1.terminal, impedance1.terminal_n) annotation (Line(
            points={{-70,-30},{-50,-30},{-50,-20}}, color={0,0,0}));
      connect(fixedCurrent2.terminal, impedance3.terminal_p) 
        annotation (Line(points={{-70,-80},{-10,-80}}, color={0,0,0}));
      connect(fixedCurrent4.terminal, impedance5.terminal_n) annotation (Line(
            points={{70,-30},{50,-30},{50,-20}}, color={0,0,0}));
      connect(fixedVoltageSource1.terminal, transformer1.terminal_p) annotation (Line(
            points={{-1.83697e-015,60},{-1.83697e-015,50},{-50,50},{-50,40}},
            color={0,0,0}));
      connect(transformer1.terminal_n, impedance1.terminal_p) 
        annotation (Line(points={{-50,20},{-50,0}}, color={0,0,0}));
      connect(transformer2.terminal_n, impedance5.terminal_p) 
        annotation (Line(points={{50,20},{50,0}}, color={0,0,0}));
      connect(transformer2.terminal_p, fixedVoltageSource1.terminal) annotation (Line(
            points={{50,40},{50,50},{-1.83697e-015,50},{-1.83697e-015,60}},
                                                                          color=
             {0,0,0}));
      connect(impedance3.terminal_n, fixedCurrent3.terminal) annotation (Line(
          points={{10,-80},{70,-80}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(impedance4.terminal_n, impedance3.terminal_n) annotation (Line(
          points={{50,-60},{50,-80},{10,-80}},
          color={0,0,0},
          smooth=Smooth.None));
    end NetworkLoop;

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
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           experiment(StopTime=1));
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
    end NetworkOpened;

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

    model PowerWorld "Interoperation of wind power and thermal power"
      extends Modelica.Icons.Example;

      PowerFlow.Units.WindFarm windFarm(redeclare package PhaseSystem = 
            PowerFlow.PhaseSystems.DirectCurrent) 
                                   annotation (Placement(transformation(extent={{-50,60},
                {-30,80}},           rotation=0)));
      PowerFlow.Units.City city 
                           annotation (                      Placement(
            transformation(extent={{60,-50},{80,-30}},
                                                    rotation=0)));
      PowerFlow.Units.LoadDispatcher dispatcher 
        annotation (                           Placement(transformation(extent={{-90,-60},
                {-70,-40}}, rotation=0)));
      PowerFlow.Units.PowerPlant powerPlant(primaryControlMax=40) 
                                       annotation (
          Placement(transformation(extent={{-62,-10},{-40,12}},
                                                              rotation=0)));
      annotation (
        experiment(StopTime=86400),
        Commands(file(ensureSimulated=true)="plot summary.mos" "plot summary",
                 file(ensureSimulated=true)="plot powerPlant.mos"
            "plot powerPlant",
                 file(ensureSimulated=true)="plot hydroPlant.mos"
            "plot hydroPlant"),
        Documentation(info="<html>
<p>
This example models a control area for power distribution in island mode, i.e. without connection to a larger net.
It contains the following consumers and producers:
<ul>
<li>a city with a load of about 1000 MW, i.e. 1 Mio inhabitants,</li>
<li>a thermal power plant with
   <ul>
   <li>800 MW nominal power</li>
   <li>60 MW for secondary frequency control</li>
   <li>40 MW for primary frequency control</li>
   </ul></li>
<li>a wind park with a max power of 300 MW,</li>
<li>a hydro plant providing:
   <ul> 
   <li>50 MW base load using a river turbine</li>
   <li>+-25 MW pumping power to support the day/night load cycle</li>
   <li>up to 200 MW peak power on demand.</li>
   </ul></li>
</ul>
</p>
<p>
The following switches/features are provided:
<ul>
<li><b>powerPlant.Modakond</b>: enhance the frequency/power control of the power plant to reduce throttle losses by utilizing condensate stop (see powerPlant.hotwellLevel.y, powerPlant.throttleReserve.y, powerPlant.pressureLoss.y, powerPlant.throttleCosts.y)</li>
<li><b>windForm.cut_off</b>: suddenly take off the wind farm as the wind speed exceeds the cut-off speed, e.g. in case of a storm</li>
</ul>
</p>
    </html>"),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics));
      Components.VoltageConverter trafoPlant(ratio=10/380) 
        annotation (Placement(transformation(extent={{-36,-6},{-24,6}})));
      Components.VoltageConverter distribution(ratio=380/50) 
        annotation (Placement(transformation(extent={{44,-46},{56,-34}})));
      Components.Inverter HVDC 
        annotation (Placement(transformation(extent={{-16,34},{-4,46}})));
      PowerFlow.Units.HydroPlant hydroPlant(primaryControlMax=200) 
        annotation (Placement(transformation(extent={{80,20},{60,40}})));
      Components.VoltageConverter trafoHydro(ratio=380/10) 
        annotation (Placement(transformation(extent={{44,24},{56,36}})));
      Components.Impedance linePlant(               R=1, L=1/50) 
        annotation (Placement(transformation(extent={{-16,-46},{-4,-34}})));
      Components.Impedance lineWind(R=1, L=1/50) 
                                         annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=-90,
            origin={10,10})));
      Components.Impedance lineHydro(               R=1, L=1/50) 
                                          annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=-90,
            origin={40,-10})));
      Modelica.Blocks.Sources.RealExpression frequency(y=der(
            distribution.PhaseSystem.angle(distribution.terminal_p.theta)))
        "Frequency in the distribution net" annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={-80,-20})));
    equation
      connect(powerPlant.terminal, trafoPlant.terminal_p) 
                                                     annotation (Line(
          points={{-40,0},{-36,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(distribution.terminal_n, city.terminal) annotation (Line(
          points={{56,-40},{60,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(windFarm.terminal, HVDC.terminal_dc) annotation (Line(
          points={{-30,70},{-20,70},{-20,40},{-16,40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(dispatcher.plantDispatch, powerPlant.plantDispatch) annotation (Line(
          points={{-73,-46},{-70,-46},{-70,-6},{-62,-6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(dispatcher.hydroDispatch, hydroPlant.hydroDispatch) annotation (Line(
          points={{-73,-52},{-70,-52},{-70,-80},{90,-80},{90,30},{80,30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(trafoPlant.terminal_n, linePlant.terminal_p) annotation (Line(
          points={{-24,0},{-20,0},{-20,-40},{-16,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(linePlant.terminal_n, distribution.terminal_p) annotation (Line(
          points={{-4,-40},{44,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(HVDC.terminal, lineWind.terminal_p) annotation (Line(
          points={{-4,40},{10,40},{10,16}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(lineWind.terminal_n, distribution.terminal_p) annotation (Line(
          points={{10,4},{10,-38},{44,-38},{44,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(lineHydro.terminal_n, distribution.terminal_p) annotation (Line(
          points={{40,-16},{40,-36},{44,-36},{44,-40}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(frequency.y, dispatcher.frequency) annotation (Line(
          points={{-80,-31},{-80,-43}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(hydroPlant.terminal, trafoHydro.terminal_n) annotation (Line(
          points={{60,30},{56,30}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(trafoHydro.terminal_p, lineHydro.terminal_p) annotation (Line(
          points={{44,30},{40,30},{40,-4}},
          color={0,0,0},
          smooth=Smooth.None));
    end PowerWorld;

  end Examples;

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
      Modelica.Blocks.Math.Add fuel annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-90,-30})));
      Modelica.Blocks.Continuous.TransferFunction transferFunction(b={180,1}, a=
           {60,1})
        "achieve a power ramp of 2% per minute, minus 0.8% tolerance" 
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
          points={{80,-10},{90,-10},{90,0},{100,0}},
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
      connect(rotor.flange_b,frequency. flange)   annotation (Line(
          points={{40,0},{44,0},{44,40},{36,40}},
          color={0,0,0},
          smooth=Smooth.None));

      connect(reservoirTurbine.flange, rotor.flange_a) 
                                                annotation (Line(
          points={{10,0},{20,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(rotor.flange_b, generator.flange)   annotation (Line(
          points={{40,0},{60,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(generator.terminal, terminal) annotation (Line(
          points={{80,0},{100,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(powerControl.y, reservoirTurbine.tau) 
                                           annotation (Line(
          points={{-23.4,0},{-12,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(controlDynamics.y, reservoirLevel.u) annotation (Line(
          points={{-43.4,0},{-40,0},{-40,30},{-32,30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(controlDynamics.y, powerControl.u) annotation (Line(
          points={{-43.4,0},{-37.2,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, controlDynamics.u) annotation (Line(
          points={{-63.4,0},{-57.2,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(hydroDispatch[2], add.u2) annotation (Line(
          points={{-100,4.44089e-016},{-84,4.44089e-016},{-84,-3.6},{-77.2,-3.6}},
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
          points={{23.4,40},{0,40},{0,58}},
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
          points={{40,-40},{50,-40},{50,0},{60,0}},
          color={0,0,0},
          smooth=Smooth.None));
    end HydroPlant;

    model WindFarm
      extends Interfaces.PartialSource(final potentialReference=false);
      parameter Boolean cut_out = false
        "stop producing energy as wind exceeds cut-out speed of 20-25 m/s" 
        annotation(Dialog(group="Features"));

      Modelica.Blocks.Sources.CombiTimeTable wind(
        tableName="tab",
        fileName="LoadData.txt",
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
          points={{30,0},{100,0}},
          color={0,0,0},
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
      connect(wind.y[4], product.u2) annotation (Line(
          points={{-59,0},{-40,0},{-40,-6},{-22,-6}},
          color={0,0,127},
          smooth=Smooth.None));

      connect(disturbance.y, product.u1) annotation (Line(
          points={{-39,30},{-30,30},{-30,6},{-22,6}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(product.y, mills.P) annotation (Line(
          points={{1,0},{9,0}},
          color={0,0,127},
          smooth=Smooth.None));
    end WindFarm;

    model LoadDispatcher
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
      Modelica.Blocks.Sources.CombiTimeTable data(
        tableName="tab",
        fileName="LoadData.txt",
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
          points={{-70,-20},{-20,-20},{-20,0},{8,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(plantSchedule.y, plantDispatch[1]) annotation (Line(
          points={{31,0},{50,0},{50,33.3333},{70,33.3333}},
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
          points={{0,70},{0,56},{30,56},{30,52}},
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
    end LoadDispatcher;

    model City
      extends Interfaces.PartialLoad;
      Modelica.Blocks.Sources.CombiTimeTable data(
        tableName="tab",
        fileName="LoadData.txt",
        tableOnFile=true,
        table=fill(
                0.0,
                0,
                10)) annotation (Placement(transformation(extent={{-60,40},{-40,
                60}}, rotation=0)));
      PowerFlow.Sources.PrescribedPowerLoad load(phi=0.34906585039887) 
                                  annotation (Placement(transformation(extent={
                {-60,-10},{-40,10}}, rotation=0)));
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
    equation
      connect(terminal, load.terminal) annotation (Line(
          points={{-100,0},{-60,0}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(data.y[2], load.P) annotation (Line(
          points={{-39,50},{-20,50},{-20,0},{-39,0}},
          color={0,0,127},
          smooth=Smooth.None));
    end City;

  end Units;

  package Components
    model Impedance
      extends PowerFlow.Interfaces.PartialTwoTerminal;
      parameter Modelica.SIunits.Resistance R = 1 "active component";
      parameter Modelica.SIunits.Inductance L = 1/50 "reactive component";
      PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));
    equation
      v = R*i + w*L*j(i);
      zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
      terminal_p.theta = terminal_n.theta;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-70,30},{70,-30}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-100,0},{-70,0}}, color={0,0,0}),
            Line(points={{70,0},{100,0}}, color={0,0,0}),
            Text(
              extent={{-144,-60},{144,-100}},
              lineColor={0,0,0},
              textString="R=%R%, L=%L%"),
            Text(
              extent={{-144,40},{144,100}},
              lineColor={0,0,0},
              textString="%name"),
            Rectangle(
              extent={{0,10},{66,-10}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end Impedance;

    model Admittance
      extends Interfaces.PartialTwoTerminal;
      parameter Modelica.SIunits.Conductance G = 1 "active component";
      parameter Modelica.SIunits.Capacitance C = 1/50 "reactive component";
      PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));
    equation
      i = G*v + w*C*j(v);
      zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
      terminal_p.theta = terminal_n.theta;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-70,30},{70,-30}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-100,0},{-70,0}}, color={0,0,0}),
            Line(points={{70,0},{100,0}}, color={0,0,0}),
            Text(
              extent={{-144,-60},{144,-100}},
              lineColor={0,0,0},
              textString="G=%G%, C=%C%"),
            Text(
              extent={{-144,40},{144,100}},
              lineColor={0,0,0},
              textString="%name"),
            Rectangle(
              extent={{14,30},{24,-30}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{36,30},{46,-30}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end Admittance;

    model VoltageConverter
      extends PowerFlow.Interfaces.PartialTwoTerminal;
      parameter Real ratio = 1 "conversion ratio terminal_p.v/terminal_n.v";
    equation
      terminal_p.v = ratio*terminal_n.v;
      zeros(PhaseSystem.n) = ratio*terminal_p.i + terminal_n.i;
      terminal_p.theta = terminal_n.theta;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,0},{-70,0}}, color={0,0,0}),
            Line(points={{70,0},{100,0}}, color={0,0,0}),
            Text(
              extent={{-144,-60},{144,-100}},
              lineColor={0,0,0},
              textString="%ratio%"),
            Text(
              extent={{-144,60},{144,120}},
              lineColor={0,0,0},
              textString="%name"),
            Ellipse(
              extent={{-80,50},{20,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-20,50},{80,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(extent={{-80,50},{20,-50}}, lineColor={0,0,0})}));
    end VoltageConverter;

    model Ground
      extends Interfaces.PartialLoad;
    equation
      terminal.v = zeros(PhaseSystem.n);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Line(points={{0,0},{0,-60}}, color={0,0,0}),
            Line(points={{-80,-60},{80,-60}}, color={0,0,0}),
            Line(points={{-50,-80},{50,-80}}, color={0,0,0}),
            Line(points={{-20,-100},{20,-100}}, color={0,0,0}),
            Line(points={{-100,0},{0,0}}, color={0,0,0})}),
                                 Diagram(graphics));
    end Ground;

    model EMF "Electro-Motoric Force"
      extends Interfaces.PartialSource(final potentialReference = synchronous);
      parameter Boolean synchronous = PhaseSystem.m > 0 "synchronous machine";
      parameter PhaseSystem.Frequency w_ref = 50 "reference value of frequency"
        annotation (Dialog(group="Reference Parameters"));
      parameter PhaseSystem.Voltage V_ref = 10e3 "reference value of voltage" 
        annotation (Dialog(group="Reference Parameters"));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange 
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}},
              rotation=0)));
      PhaseSystem.Frequency w = der(flange.phi);
      PhaseSystem.Voltage V;
    equation
      0 = PhaseSystem.systemPower(terminal.v*terminal.i) + w*flange.tau;
      terminal.v = PhaseSystem.phaseVoltages(V, 0);
      if synchronous then
        flange.phi = PhaseSystem.angle(terminal.theta);
        if isRoot(terminal.theta) then
          V = V_ref;
        end if;
      else
        V = V_ref/w_ref*w;
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
                -100},{100,100}}), graphics={
            Line(points={{-100,0},{-50,0}}, color={0,0,0}),
            Line(points={{50,0},{100,0}}, color={0,0,0}),
            Text(
              extent={{-144,-60},{144,-100}},
              lineColor={0,0,0},
              textString="V=%V_ref V"),
            Text(
              extent={{-144,40},{144,100}},
              lineColor={0,0,0},
              textString="%name"),
            Ellipse(
              extent={{-50,50},{50,-50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-40,30},{40,-30}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="A",
              visible=not synchronous),
            Rectangle(
              extent={{-28,30},{30,-30}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=synchronous),
            Text(
              extent={{-40,30},{40,-30}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="S",
              visible=synchronous)}));
    end EMF;

    model Inverter "Convert direct current to alternating current"
      extends Interfaces.PartialSource(final potentialReference = true);
      package PhaseSystem_dc = PowerFlow.PhaseSystems.DirectCurrent;
      PowerFlow.Interfaces.Terminal_p terminal_dc(
        redeclare package PhaseSystem = PhaseSystem_dc) 
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
      parameter PhaseSystem_dc.Voltage V_dc = 150e3 "voltage of dc system";
      parameter PhaseSystem.Frequency w_ref = 50
        "frequency of sinusoidal voltage" 
        annotation (Dialog(group="Reference Parameters"));
      PhaseSystem.Current I "value of current";
    equation
      terminal_dc.v = PhaseSystem_dc.phaseVoltages(V_dc);
      terminal.i = PhaseSystem.phaseCurrents(I, 0);
      0 = PhaseSystem_dc.systemPower(terminal_dc.v*terminal_dc.i) + PhaseSystem.systemPower(terminal.v*terminal.i);
      if isRoot(terminal.theta) and PhaseSystem.m > 0 then
        PhaseSystem.angle(terminal.theta) = w_ref*time;
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=true,
              extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{0,-68},{80,12}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="~"),
            Line(
              points={{-90,-90},{90,90}},
              color={0,0,0},
              smooth=Smooth.None),
            Text(
              extent={{-68,-10},{12,70}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="="),
            Text(
              extent={{0,-84},{80,-4}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="~"),
            Text(
              extent={{0,-100},{80,-20}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="~"),
            Text(
              extent={{-142,98},{146,158}},
              lineColor={0,0,0},
              textString="%name")}));
    end Inverter;
  end Components;

  package Sources
    model FixedVoltageSource
      extends Interfaces.PartialSource;
      parameter PhaseSystem.Voltage V = 10e3 "value of constant voltage";
      parameter PhaseSystem.Frequency w_ref = 50
        "frequency of sinusoidal voltage" 
        annotation (Dialog(group="Reference Parameters"));
    equation
      terminal.v = PhaseSystem.phaseVoltages(V, 0);
      if isRoot(terminal.theta) and PhaseSystem.m > 0 then
        PhaseSystem.angle(terminal.theta) =  w_ref*time;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.CrossDiag)}),
                                 Diagram(coordinateSystem(preserveAspectRatio=false,
                       extent={{-100,-100},{100,100}}),
                                         graphics));
    end FixedVoltageSource;

    model FixedLoad
      extends Interfaces.PartialLoad;
      parameter Modelica.SIunits.Power P = 0
        "rms value of constant active power";
      parameter Modelica.SIunits.Angle phi = 0 "phase angle";
    equation
      PhaseSystem.phasePowers_vi(terminal.v, terminal.i) = PhaseSystem.phasePowers(P, phi);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-38,-68},{38,-26}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="VA")}));
    end FixedLoad;

    model FixedCurrent
      extends Interfaces.PartialLoad;
      parameter Modelica.SIunits.Current I = 0 "rms value of constant current";
      parameter Modelica.SIunits.Angle phi = 0 "phase angle" 
      annotation (Dialog(group="Reference Parameters", enable = definiteReference));
    equation
      terminal.i = PhaseSystem.phaseCurrents(I, phi);
      annotation (Diagram(graphics),
                          Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-78,22},{80,64}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="%I% A"),
            Text(
              extent={{-38,-74},{38,-32}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="%phi%")}));
    end FixedCurrent;

    model PrescribedPowerSource "Prescribed power source"
      extends Interfaces.PartialSource(
        final potentialReference=true);
      parameter PhaseSystem.Frequency w_ref = 50
        "frequency of sinusoidal voltage" 
        annotation (Dialog(group="Reference Parameters"));
      Modelica.Blocks.Interfaces.RealInput P(unit="MW") annotation (Placement(
            transformation(extent={{-130,-20},{-90,20}}, rotation=0)));
      PhaseSystem.Current I "value of current";
    equation
      terminal.i = PhaseSystem.phaseCurrents(I, 0);
      0 = PhaseSystem.systemPower(terminal.v*terminal.i) + 1e6*P;
      if isRoot(terminal.theta) and PhaseSystem.m > 0 then
        PhaseSystem.angle(terminal.theta) =  w_ref*time;
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-90,-132},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="%name%")}));
    end PrescribedPowerSource;

    model PrescribedPowerLoad "Prescribed power load"
      extends Interfaces.PartialLoad;
      parameter Modelica.SIunits.Angle phi = 0 "phase angle";
      Modelica.Blocks.Interfaces.RealInput P(unit="MW") annotation (Placement(
            transformation(extent={{130,-20},{90,20}}, rotation=0)));
    equation
      PhaseSystem.phasePowers_vi(terminal.v, terminal.i) = PhaseSystem.phasePowers(1e6*P, phi);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics),
                           Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={Rectangle(
              extent={{-90,90},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-90,-132},{90,-90}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="%name%")}));
    end PrescribedPowerLoad;
  end Sources;

  package Interfaces
    connector Terminal "General power terminal"
      replaceable package PhaseSystem = PhaseSystems.PartialPhaseSystem
        "Phase system" 
        annotation (choicesAllMatching=true);
      PhaseSystem.Voltage v[PhaseSystem.n] "voltage vector";
      flow PhaseSystem.Current i[PhaseSystem.n] "current vector";
      PhaseSystem.ReferenceAngle theta[PhaseSystem.m] "vector of phase angles";
    end Terminal;

    connector Terminal_p "Positive terminal"
      extends PowerFlow.Interfaces.Terminal(v(each start=1e3, each nominal=1e3), i(each start
            =                                                                                 1e3, each
            nominal =                                                                                           1e3));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Polygon(
              points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,-90},{150,-150}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              textString="%name")}),     Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Polygon(
              points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end Terminal_p;

    connector Terminal_n "Negative terminal"
      extends PowerFlow.Interfaces.Terminal(v(each start=1e3, each nominal=1e3), i(each start
            =                                                                                 -1e3, each
            nominal =                                                                                            1e3));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Polygon(
              points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,-90},{150,-150}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              textString="%name"),
            Polygon(
              points={{70,70},{-70,70},{-70,-70},{70,-70},{70,70}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),         Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Polygon(
              points={{100,100},{-100,100},{-100,-100},{100,-100},{100,100}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid), Polygon(
              points={{70,70},{-70,70},{-70,-70},{70,-70},{70,70}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end Terminal_n;

    partial model PartialTwoTerminal
      replaceable package PhaseSystem = PackagePhaseSystem constrainedby
        PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
        annotation (choicesAllMatching=true);
      function j = PhaseSystem.j;
      PowerFlow.Interfaces.Terminal_p terminal_p(
                                        redeclare package PhaseSystem = 
            PhaseSystem)                                           annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
      PowerFlow.Interfaces.Terminal_n terminal_n(
                                        redeclare package PhaseSystem = 
            PhaseSystem)                                           annotation (Placement(
            transformation(extent={{90,-10},{110,10}}, rotation=0)));
      PhaseSystem.Voltage v[:] = terminal_p.v - terminal_n.v;
      PhaseSystem.Current i[:] = terminal_p.i;
      PhaseSystem.Power S[PhaseSystem.n] = PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(v, i));
      PhaseSystem.Voltage V = PhaseSystem.systemVoltage(v);
      PhaseSystem.Current I = PhaseSystem.systemCurrent(i);
      PhaseSystem.PhaseAngle phi = PhaseSystem.phase(v) - PhaseSystem.phase(i);
    equation
      Connections.branch(terminal_p.theta, terminal_n.theta);
    end PartialTwoTerminal;

    partial model PartialSource
      replaceable package PhaseSystem = PackagePhaseSystem constrainedby
        PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
        annotation (choicesAllMatching=true);
      function j = PhaseSystem.j;
      PowerFlow.Interfaces.Terminal_n terminal(
                                      redeclare package PhaseSystem = 
            PhaseSystem) 
        annotation (Placement(transformation(extent={{90,-10},{110,10}},
              rotation=0)));
      PhaseSystem.Power S[PhaseSystem.n] = -PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(terminal.v, terminal.i));
      PhaseSystem.PhaseAngle phi = PhaseSystem.phase(terminal.v) - PhaseSystem.phase(-terminal.i);
      parameter Boolean potentialReference = true "serve as potential root" 
         annotation (Evaluate=true, Dialog(group="Reference Parameters"));
      parameter Boolean definiteReference = false "serve as definite root" 
         annotation (Evaluate=true, Dialog(group="Reference Parameters"));
    equation
      if potentialReference then
        if definiteReference then
          Connections.root(terminal.theta);
        else
          Connections.potentialRoot(terminal.theta);
        end if;
      end if;
    end PartialSource;

    partial model PartialLoad
      replaceable package PhaseSystem = PackagePhaseSystem constrainedby
        PowerFlow.PhaseSystems.PartialPhaseSystem "Phase system" 
        annotation (choicesAllMatching=true);
      function j = PhaseSystem.j;
      PowerFlow.Interfaces.Terminal_p terminal(
                                      redeclare package PhaseSystem = 
            PhaseSystem) 
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}},
              rotation=0)));
      PhaseSystem.Voltage v[:] = terminal.v;
      PhaseSystem.Current i[:] = terminal.i;
      PhaseSystem.Power S[PhaseSystem.n] = PhaseSystem.systemPower(PhaseSystem.phasePowers_vi(v, i));
    end PartialLoad;
  end Interfaces;

  package Types
    type ReferenceAngle "Reference angle"
      extends Modelica.SIunits.Angle;

      function equalityConstraint
        input ReferenceAngle theta1[:];
        input ReferenceAngle theta2[:];
        output Real[0] residue "No constraints";
      algorithm
        for i in 1:size(theta1, 1) loop
          assert(abs(theta1[i] - theta2[i]) < Modelica.Constants.eps, "angles theta1 and theta2 not equal over connection!");
        end for;
      end equalityConstraint;
    end ReferenceAngle;

    type PhaseDisplacement "Phase displacement between voltage and current"
      extends Modelica.SIunits.LossAngle;

      function equalityConstraint
        input PhaseDisplacement phi1;
        input PhaseDisplacement phi2;
        output Real[0] residue "No constraints";
      algorithm
        assert(abs(phi1 - phi2) < Modelica.Constants.eps, "phase displacements phi1 and phi2 not equal over connection!");
      end equalityConstraint;
    end PhaseDisplacement;

    package ComplexNumbers
      record Complex "complex number"
        Real re(start = 0) "real part";
        Real im(start = 0) "imaginary part";
      end Complex;

      function plus
        input Complex x1;
        input Complex x2;
        output Complex y(re = x1.re + x2.re, im = x1.im + x2.im);
      algorithm
      end plus;

      function minus
        input Complex x1;
        input Complex x2;
        output Complex y(re = x1.re - x2.re, im = x1.im - x2.im);
      algorithm
      end minus;

      function times
        input Complex x1;
        input Complex x2;
        output Complex y(re = x1.re*x2.re - x1.im*x2.im, im = x1.im*x2.re + x1.re*x2.im);
      algorithm
      end times;

      function divide
        input Complex x1;
        input Complex x2;
        output Complex y(re = (x1.re*x2.re + x1.im*x2.im)/(x2.re^2 + x2.im^2), im = (x1.im*x2.re - x1.re*x2.im)/(x2.re^2 + x2.im^2));
      algorithm
      end divide;

    end ComplexNumbers;

    record ComplexPower = ComplexNumbers.Complex (re(unit="W"), im(unit="var"));
    record ComplexResistance = ComplexNumbers.Complex (re(unit="Ohm"), im(unit=
              "Ohm"));
    record ComplexVoltage = ComplexNumbers.Complex (re(unit="V"), im(unit="V"));
    record ComplexCurrent = ComplexNumbers.Complex (re(unit="A"), im(unit="A"));
  end Types;

  package PhaseSystems "Phase system used in a power flow connection"

    partial package PartialPhaseSystem "Base package of all phase systems"
      constant String phaseSystemName = "UnspecifiedPhaseSystem";
      constant Integer n "Number of independent voltage and current components";
      constant Integer m "Number of reference angles";

      type Voltage = Real(unit = "V", quantity = "Voltage." + phaseSystemName);
      type Current = Real(unit = "A", quantity = "Current." + phaseSystemName);
      type Frequency = Real(unit = "Hz", quantity = "Frequency." + phaseSystemName);
      type Angle = Real(unit = "rad", displayUnit="deg", quantity = "Angle." + phaseSystemName);
      type PhaseAngle = Real(unit = "rad", displayUnit="deg", quantity = "Angle." + phaseSystemName);
      type LossAngle = Real(unit = "rad", quantity = "Angle." + phaseSystemName);
      type Power = Real(unit = "W", displayUnit = "MW", quantity = "Power." + phaseSystemName);
      type ActivePower = Power;
      type ReactivePower = Real(unit = "var", quantity = "Power." + phaseSystemName);
      type ApparentPower = Real(unit = "VA", quantity = "Power." + phaseSystemName);

      type ReferenceAngle "Reference angle"
        extends Modelica.SIunits.Angle;

        function equalityConstraint
          input ReferenceAngle theta1[:];
          input ReferenceAngle theta2[:];
          output Real[0] residue "No constraints";
        algorithm
          for i in 1:size(theta1, 1) loop
            assert(abs(theta1[i] - theta2[i]) < Modelica.Constants.eps, "angles theta1 and theta2 not equal over connection!");
          end for;
        end equalityConstraint;
      end ReferenceAngle;

      replaceable partial function j "Return vector rotated by 90 degrees"
        extends Modelica.Icons.Function;
        input Real x[n];
        output Real y[n];
      end j;

      replaceable partial function angle
        "Return angle of rotating reference system"
        input Angle theta[m];
        output Angle angle;
      end angle;

      replaceable partial function phase "Return phase"
        extends Modelica.Icons.Function;
        input Real x[n];
        output PhaseAngle phase;
      end phase;

      replaceable partial function phaseVoltages
        "Return phase to neutral voltages"
        extends Modelica.Icons.Function;
        input Voltage V "system voltage";
        input Angle phi = 0 "phase angle";
        output Voltage v[n] "phase to neutral voltages";
      end phaseVoltages;

      replaceable partial function phaseCurrents "Return phase currents"
        extends Modelica.Icons.Function;
        input Current I "system current";
        input Angle phi = 0 "phase angle";
        output Current i[n] "phase currents";
      end phaseCurrents;

      replaceable partial function phasePowers "Return phase powers"
        extends Modelica.Icons.Function;
        input ActivePower P "active system power";
        input Angle phi = 0 "phase angle";
        output Power p[n] "phase powers";
      end phasePowers;

      replaceable partial function phasePowers_vi "Return phase powers"
        extends Modelica.Icons.Function;
        input Voltage v[n] "phase voltages";
        input Current i[n] "phase currents";
        output Power p[n] "phase powers";
      end phasePowers_vi;

      replaceable partial function systemVoltage
        "Return system voltage as function of phase voltages"
        extends Modelica.Icons.Function;
        input Voltage v[n];
        output Voltage V;
      end systemVoltage;

      replaceable partial function systemCurrent
        "Return system current as function of phase currents"
        extends Modelica.Icons.Function;
        input Current i[n];
        output Current I;
      end systemCurrent;

      replaceable partial function systemPower
        "Return total power as function of phase powers"
        extends Modelica.Icons.Function;
        input Power p "phase power";
        output Power P "system power";
      end systemPower;

    end PartialPhaseSystem;

    package DirectCurrent "DC system"
      extends PartialPhaseSystem(phaseSystemName="DirectCurrent", n=1, m=0);

      redeclare function j "Return vector rotated by 90 degrees"
        extends Modelica.Icons.Function;
        input Real x[n];
        output Real y[n];
      algorithm
        y := x;
      end j;

      redeclare function angle
        "Return absolute angle of rotating reference system"
        input Angle theta[m];
        output Angle angle;
      algorithm
        angle := 0;
      end angle;

      redeclare function phase "Return phase"
        extends Modelica.Icons.Function;
        input Real x[n];
        output PhaseAngle phase;
      algorithm
        phase := 0;
      end phase;

      redeclare function phaseVoltages "Return phase to neutral voltages"
        extends Modelica.Icons.Function;
        input Voltage V "system voltage";
        input Angle phi = 0 "phase angle";
        output Voltage v[n] "phase to neutral voltages";
      algorithm
        v := {V};
      end phaseVoltages;

      redeclare function phaseCurrents "Return phase currents"
        extends Modelica.Icons.Function;
        input Current I "system current";
        input Angle phi = 0 "phase angle";
        output Current i[n] "phase currents";
      algorithm
        i := {I};
      end phaseCurrents;

      redeclare function phasePowers "Return phase powers"
        extends Modelica.Icons.Function;
        input ActivePower P "active system power";
        input Angle phi = 0 "phase angle";
        output Power p[n] "phase powers";
      algorithm
        p := {P};
      end phasePowers;

      redeclare function phasePowers_vi "Return phase powers"
        extends Modelica.Icons.Function;
        input Voltage v[n] "phase voltages";
        input Current i[n] "phase currents";
        output Power p[n] "phase powers";
      algorithm
        p := {v*i};
      end phasePowers_vi;

      redeclare function systemVoltage
        "Return system voltage as function of phase voltages"
        extends Modelica.Icons.Function;
        input Voltage v[n];
        output Voltage V;
      algorithm
        V := v[1];
      end systemVoltage;

      redeclare function systemCurrent
        "Return system current as function of phase currents"
        extends Modelica.Icons.Function;
        input Current i[n];
        output Current I;
      algorithm
        I := i[1];
      end systemCurrent;

      redeclare function systemPower
        "Return total power as function of phase power"
        extends Modelica.Icons.Function;
        input Power p "phase power";
        output Power P "system power";
      algorithm
        P := p;
      end systemPower;

    end DirectCurrent;

    package ThreePhaseSymmetric "AC system, symmetrically loaded three phases"
      extends PartialPhaseSystem(phaseSystemName="ThreePhaseSymmetric", n=2, m=1);

      redeclare function j "Return vector rotated by 90 degrees"
        extends Modelica.Icons.Function;
        input Real x[n];
        output Real y[n];
      algorithm
        y := {-x[2], x[1]};
      end j;

      redeclare function angle
        "Return absolute angle of rotating reference system"
        input Angle theta[m];
        output Angle angle;
      algorithm
        angle := theta[1];
      end angle;

      redeclare function phase "Return phase"
        extends Modelica.Icons.Function;
        input Real x[n];
        output PhaseAngle phase;
      algorithm
        phase :=arctan2(x[2], x[1]);
      end phase;

      redeclare function phaseVoltages "Return phase to neutral voltages"
        extends Modelica.Icons.Function;
        input Voltage V "system voltage";
        input Angle phi = 0 "phase angle";
        output Voltage v[n] "phase to neutral voltages";
      algorithm
        v := {V*cos(phi), V*sin(phi)}/sqrt(3);
      end phaseVoltages;

      redeclare function phaseCurrents "Return phase currents"
        extends Modelica.Icons.Function;
        input Current I "system current";
        input Angle phi = 0 "phase angle";
        output Current i[n] "phase currents";
      algorithm
        i := {I*cos(phi), I*sin(phi)};
      end phaseCurrents;

      redeclare function phasePowers "Return phase powers"
        extends Modelica.Icons.Function;
        input ActivePower P "active system power";
        input Angle phi = 0 "phase angle";
        output Power p[n] "phase powers";
      algorithm
        p := {P, P*tan(phi)}/3;
      end phasePowers;

      redeclare function phasePowers_vi "Return phase powers"
        extends Modelica.Icons.Function;
        input Voltage v[n] "phase voltages";
        input Current i[n] "phase currents";
        output Power p[n] "phase powers";
      algorithm
        p := {v*i, -j(v)*i};
      end phasePowers_vi;

      redeclare function systemVoltage
        "Return system voltage as function of phase voltages"
        extends Modelica.Icons.Function;
        input Voltage v[n];
        output Voltage V;
      algorithm
        V := sqrt(3*v*v);
      end systemVoltage;

      redeclare function systemCurrent
        "Return system current as function of phase currents"
        extends Modelica.Icons.Function;
        input Current i[n];
        output Current I;
      algorithm
        I := sqrt(i*i);
      end systemCurrent;

      redeclare function systemPower
        "Return total power as function of phase power"
        extends Modelica.Icons.Function;
        input Power p "phase power";
        output Power P "system power";
      algorithm
        P := 3*p;
      end systemPower;

    end ThreePhaseSymmetric;

  end PhaseSystems;

  package Test
    package Components
      model ImpedanceTest
        import PowerFlow;

        Sources.FixedVoltageSource source 
          annotation (Placement(transformation(extent={{-80,0},{-60,20}},
                rotation=0)));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
      end ImpedanceTest;

      model AdmittanceTest
        import PowerFlow;

        Sources.FixedVoltageSource source 
          annotation (Placement(transformation(extent={{-80,0},{-60,20}},
                rotation=0)));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
      end AdmittanceTest;

      model InductiveLoadTest
        import PowerFlow;

        Sources.FixedVoltageSource source 
          annotation (Placement(transformation(extent={{-80,0},{-60,20}},
                rotation=0)));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
      end InductiveLoadTest;

      model FixedCurrentTest
        import PowerFlow;

        PowerFlow.Sources.FixedVoltageSource source 
          annotation (Placement(transformation(extent={{-80,0},{-60,20}},
                rotation=0)));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
      end FixedCurrentTest;

      model FixedLoadTest
        import PowerFlow;

        Sources.FixedVoltageSource source 
          annotation (Placement(transformation(extent={{-80,0},{-60,20}},
                rotation=0)));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
        annotation (Diagram(coordinateSystem(
              preserveAspectRatio=false,
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
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics),
                             experiment(StopTime=1));
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
      end EMFTest2;
    end Components;

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
  end Test;
end PowerFlow;
