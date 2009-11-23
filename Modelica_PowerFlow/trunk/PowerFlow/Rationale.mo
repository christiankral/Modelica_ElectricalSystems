within PowerFlow;
package Rationale "Documentation of main concepts"

  annotation (DocumentationClass = true, Documentation(info="<html>
<p><h4><font color=\"#008000\">Overview</font></h4></p>
<p>PowerFlow provides basic models and a general connector for the electrical side of power plant systems. Currently it allows to treat symmetrically loaded three phase systems and direct current lines in one framework. PowerFlow is used to investigate the test case &QUOT;Stabilization of wind power&QUOT; in the Eurosyslib work package 5.3. </p>
<p>The PowerFlow connector is motivated by the SPOT library and by the concept of replaceable Modelica.Media packages for fluid models. The aim is to support different single and polyphase systems and different mathematical formulations in one framework. This may cover applications like: </p>
<p><ul>
<li>Three phase AC transmission lines, using natural or modal coordinates, </li>
<li>Single phase AC applications, </li>
<li>Variable freqency systems, e.g. to control the power of a motor with a frequency converter, and </li>
<li>Direct current lines, like HVDC </li>
</ul></p>
<p>A general terminal for electrical power systems can be defined as: </p>
<pre>connector Terminal &QUOT;General power terminal&QUOT;
  replaceable package PhaseSystem = PhaseSystems.PartialPhaseSystem &QUOT;Phase system&QUOT;;
  PhaseSystem.Voltage v[PhaseSystem.n] &QUOT;voltage vector&QUOT;;
  flow PhaseSystem.Current i[PhaseSystem.n] &QUOT;current vector&QUOT;;
  PhaseSystem.ReferenceAngle theta[PhaseSystem.m] &QUOT;vector of phase angles&QUOT;;
end Terminal;</pre>
<p>The replaceable PhaseSystem defines the number <code><b>n</b></code> of independent voltage and current components and their representation in the connector. Moreover it defines types for the physical quantities so that terminals of different phase systems cannot be directly connected. </p>
<p>The vector of reference angles <code><b>theta[m]</b></code> allows the definition of a rotating reference system for the description of AC systems with modal components. It is known from the SPOT library that this simplifies the treatment of sinosodial quantities in the time domain. The power Terminal is overdetermined with the reference angles though and the operators Connections.root, Connections.potentialRoot, Connections.isRoot and Connections.branch are used for their implementation. A Modelica tool needs to analyze connection graphs and eliminate redundant equations. </p>
<p><h4><font color=\"#008000\">Existing electrical libraries</font></h4></p>
<p><b>Modelica.Electrical</b> describes voltages and currents using natural coordinates in the time domain, i.e. m=0. It defines a distinct MultiPhase sublibrary for multi phase systems (n&GT;=1, default: 3). This has the drawback that sinusoidal quantities of AC systems with frequencies of e.g., 50 Hz complicate the numerical analysis as electrical quantities vary with a period of 20 ms while the system dynamics of interest may be in the order of seconds to minutes. </p>
<p><b>ObjectStab</b> uses complex current and voltage phasors for the analysis of symmetrically loaded three phase systems with constant frequency, i.e. n=2, m=0. The freqency is a global property. This complicates the analysis of power/frequency control and of drives with frequency converters. </p>
<p><b>Complex</b> combines the concepts of Modelica.Electrical and ObjectStab. It uses complex voltage and current phasors with constant frequency (m=0). Similar to Modelica.Electrical it defines different sublibraries for one phase systems (n=1) and three phase systems (n=3). </p>
<p><b>AC</b> uses complex voltage and current phasors for SinglePhase systems (n=1, m=1).</p>
<p><b>SPOT</b> allows the detailed and efficient analysis of three phase systems. It provides the most comprehensive collection of models for power electronics currently available in Modelica. SPOT provides three sublibraries for three phase systems with modal dq0 coordinates (n=3, m=2), natural abc coordinates (n=3, m=2) as well as for DC systems (n=1, m=0). </p>
<p><h4><font color=\"#008000\">PhaseSystems and generic component models</font></h4></p>
<p>It is the hope that the valuable concepts of different existing libraries can be combined into one and that at least basic interfaces and components can be shared. The current version of the PowerFlow library concentrates on the not yet available symetrically loaded three phase systems with modal coordinates. This is provided by the <code>ThreePhaseSymmetric</code> phase system (n=2, m=1). Moreover the <code>DirectCurrent</code> phase system (n=1, m=0) is provided to investigate the sharing of connectors and component models. </p>
<p>A PhaseSystem is a package that provides types, functions and constants. A generic steady-state impedance model, which is independent of the phase system in use, can be formulated as: </p>
<pre>model GenericImpedance
  replaceable package PhaseSystem = PackagePhaseSystem &QUOT;Phase system&QUOT;;

  function j = PhaseSystem.j;

  Terminal terminal_p(redeclare package PhaseSystem = PhaseSystem);
  Terminal terminal_n(redeclare package PhaseSystem = PhaseSystem);

  PhaseSystem.Voltage v[:] = terminal_p.v - terminal_n.v;
  PhaseSystem.Current i[:] = terminal_p.i;
  PhaseSystem.Frequency w = der(PhaseSystem.angle(terminal_p.theta));

  parameter Modelica.SIunits.Resistance R = 1 &QUOT;active component&QUOT;;
  parameter Modelica.SIunits.Inductance L = 1/50 &QUOT;reactive component&QUOT;;

equation
  v = R*i + w*L*j(i);
  zeros(PhaseSystem.n) = terminal_p.i + terminal_n.i;
  terminal_p.theta = terminal_n.theta;
  Connections.branch(terminal_p.theta, terminal_n.theta);

end GenericImpedance; </pre>
<p><h4><font color=\"#008000\">Examples</font></h4></p>
<p>The examples NetworkLoop and NetworkOpened are taken from the textbook Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14.2.5: Leistungsfluss in Ringnetzen. The example NetworkControlled additionally investigates frequency/power control in conjunction with the Modelica.Rotational library and a basic EMF (Electro-Motoric Force). </p>
<p>The PowerWorld example models a control area for power distribution in island mode. It is used to investigate the test case &QUOT;Stabilization of wind power&QUOT; in the Eurosyslib work package 5.3. See <a href=\"Modelica://PowerFlow.Examples.PowerWorld\">Examples.PowerWorld</a>. </p>
<p><h4><font color=\"#008000\">Tests</font></h4></p>
<p>The component tests are taken from the textbook Oeding, Oswald: Elektrische Kraftwerke und Netze, section 14: Leistungsfluss im Drehstromnetz. </p>
<p><br/>Copyright &copy; 2007-2009, <a href=\"Modelica://Modelica.UsersGuide.Contact\">Modelica Association</a>. </p>
<p>This Modelica package is Open Source software; it can be redistributed and/or modified under the terms of the <a href=\"Modelica://Modelica.UsersGuide.ModelicaLicense\">Modelica license, version 2.0</a>. </p>
<p>Partial financial support of ABB for this work within the ITEA2 EUROSYSLIB project is highly appreciated (BMBF F&ouml;rderkennzeichen  01IS07022F). </p>
</html>",
  revisions="<html>
<p><ul>
<li><i>26 Feb 2009</i> by <a href=\"mailto:Ruediger.Franke@de.abb.com\">R&uuml;diger Franke</a>:<br/>Version 0.3 </li>
<li><ul>
<li>Generalize power Terminal with n voltages/currents and m reference angles </li>
<li>Add PowerWorld example </li>
</ul></li>
<li><i>21 Oct 2008</i> by <a href=\"mailto:Ruediger.Franke@de.abb.com\">R&uuml;diger Franke</a>:<br/>Version 0.2 </li>
<li><ul>
<li>Replace balanced, non-minimal connector with overdetermined minimal connector </li>
<li>Remove instance variables BaseProperties from PhaseSystems and introduce member functions instead </li>
</ul></li>
<li><i>15 Aug 2008</i> by <a href=\"mailto:Ruediger.Franke@de.abb.com\">R&uuml;diger Franke</a>:<br/>Version 0.1 </li>
</ul></p>
</html>"));
end Rationale;
