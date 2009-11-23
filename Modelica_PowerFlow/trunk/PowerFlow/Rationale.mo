within PowerFlow;
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
