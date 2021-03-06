within ;
package Modelica_Electrical_PowerConverters "Rectifiers, Inverters and DC/DC converters"
extends Modelica.Icons.Package;


package UsersGuide "User's Guide"
  extends Modelica.Icons.Information;
  class ACDCConcept "AC/DC converter concept"
    extends Modelica.Icons.Information;
    annotation (DocumentationClass=true, Documentation(info="<html>

<p>AC/DC converters are also called rectifiers</p>

<h4>Component classification</h4>

<p>Convential AC/DC converters contain diodes and thyristors. If thyristors are used, the output voltage of the rectifier can be controlled. If only diodes are used, the output voltages is solely dependent on the input voltage and the characteristic of applied diodes.</p>
<ul>
  <li>Diode rectifiers</li>
  <li>Thyristor rectifiers</li>
  <li>Half controlled rectifiers; half of the semiconductors are diodes and thyristors, respectively</li>
</ul>

<h4>Topology classificaton</h4>

<p>The PowerConverters library provides bridge and center tap rectifiers for single and multi phase supply, see 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC\">AC/DC converters</a>.</p>

<h4>Control</h4>

<p>For each of the provided rectifiers a 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control\">control model</a> is availanble. 
These control models have electrical connectors to be connected with the AC suppy. 
The firing angle of thyristor rectifiers can either be set by a parameter or a signal input. 
</p>

<h4>Examples</h4>

<p>A variety of examples is provided at
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC\">Examples.ACDC</a>. 
These examples include different kinds of DC loads. Even the control characeteristics 
of the rectifiers can be obtained experimentally; the names of these models 
contain <code>_Characteristic</code>.
</p>
</html>"));
  end ACDCConcept;

  class DCACConcept "DC/AC converter concept"
    extends Modelica.Icons.Information;
    annotation (DocumentationClass=true, Documentation(info="<html>

<p>There are a single and multi phase DC/AC converter model provided by the PowerConverters library.</p>

<h4>Control</h4>

<p>There are currently no space phasor pulse width modulation (PWM) models provided. However, for operating the single 
and multi phase inverter the PWM 
<a href=\"modelica://Modelica_Electrical_PowerConverters.DCDC.Control.SignalPWM\">controller</a> 
can be used. 
</p>

<h4>Examples</h4>

<p>Some examples are provided at
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.DCAC\">Examples.DCAC</a>.
</p>
</html>"));
  end DCACConcept;

  class DCDCConcept "DC/DC converter concept"
    extends Modelica.Icons.Information;
    annotation (DocumentationClass=true, Documentation(info="<html>

<p>The following DC/DC converter topologies are currently included in the PowerConverters libraray.</p>

<p>
<ul>
<li>Chopper step down converter</li>
<li>H bridge converter; four quadrant operation</li>
</ul>
</p>

<h4>Control</h4>

<p>A pulse width modulation (PWM) 
<a href=\"modelica://Modelica_Electrical_PowerConverters.DCDC.Control\">controller</a> 
is provided. 
</p>

<h4>Examples</h4>

<p>Some examples are provided at
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.DCDC\">Examples.DCDC</a>.
</p>
</html>"));
  end DCDCConcept;

  class Contact "Contact"
    extends Modelica.Icons.Contact;
    annotation (Documentation(info="<html>
<h4>Contact</h4>

<p>
  Dr. Christian Kral<br>
  <a href=\"http://christiankral.net/\">Electric Machines, Drives and Systems</a><br>
  A-1060 Vienna, Austria<br>
  email: <a href=\"mailto:dr.christian.kral@gmail.com\">dr.christian.kral@gmail.com</a>
</p>

<p>
Anton Haumer<br>
<a href=\"http://www.haumer.at\">Technical Consulting &amp; Electrical Engineering</a><br>
3423 St. Andrae-Woerdern, Austria<br>
email: <a HREF=\"mailto:a.haumer@haumer.at\">a.haumer@haumer.at</a><br>
</p>

</html>"));
  end Contact;

  class ReleaseNotes "Release Notes"
    extends Modelica.Icons.ReleaseNotes;
    annotation (Documentation(info="<html>

<h5>Version 1.2.0, 2014-04-06</h5>
<ul>
<li>Moved enabling signals from control to inverter models due to consistency reasons</li>
<li>Added partial models for enabling firing signals</li>
</ul>

<h5>Version 1.1.0, 2014-03-24</h5>
<ul>
<li>Removed StepUp converter due to consistency reasons</li>
</ul>

<h5>Version 1.0.0, 2014-03-24</h5>
<ul>
<li>First tagged version</li>
</ul>

</html>"));
  end ReleaseNotes;

  class References "References"
    extends Modelica.Icons.References;
    annotation (Documentation(info="<html>
<h4>References</h4>

<p>
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
    <tr>
      <td valign=\"top\">[Skvarenina01]</td>
      <td valign=\"top\">Timothy L. Skvarenina,
        <a href=\"http://www.crcpress.com/product/isbn/9780849373367\">
        <i>The Power Electronics Handbook</i></a>,
        CRC Press 2001, ISBN 9780849373367</td>
    </tr>

    <tr>
      <td valign=\"top\">[Luo05]</td>
      <td valign=\"top\">Fang Lin Luo, Hong Ye and Muhammad H. Rashid,
        <a href=\"http://store.elsevier.com/product.jsp?isbn=9780120887576&_requestid=1725\"><i>Digital Power Electronics and Applications</i></a>,
        Elsevier Academic Press, 2005, ISBN 978-0120887576</td>
    </tr>

    <tr>
      <td valign=\"top\">[Williams2006]</td>
      <td valign=\"top\"><a href=\"http://www.freescience.info/go.php?pagename=books&id=1732\">
<i>Principles and Elements of Power Electronics: Devices, Drivers, Applications, and Passive Components</i></a>, available at <a href=\"http://www.freescience.info/go.php?pagename=books&id=1732\">FreeScience</a>, ISBN 978-0-9553384-0-3</td>
    </tr>
</table>
</p>
</html>"));
  end References;
  annotation (DocumentationClass=true, Documentation(info="*<html>
<p>
This library provides power converters for DC and AC single and multi phase electrical systems. The PowerConverters library contains three types of converters.
</p>

<ul>
  <li>AC/DC converters (rectifiers)</li>
  <li>DC/AC converters (inverters)</li>
  <li>DC/DC converters</li>
</ul>

<p>
General types of AC/AC converters are currently not provided in this library.
</p>

<h4>Converter characteristics</h4>

<ul>
  <li>All converter models rely on existing diode, thyristor and switch models provided in the
      <a href=\"modelica://Modelica.Electrical.Analog.Ideal\">Analog.Ideal</a> and the 
      <a href=\"modelica://Modelica.Electrical.MultiPhase.Ideal\">MultiPhase.Ideal</a> 
      package of the Modelica Standard Library.</li>
  <li>Switching losses and recovery effects are not considered</li>
  <li>Only conduction losses are taken into account</li>
  <li>The parameters of the semiconductors include<li>
  <ul>
    <li>The on state resistance <code>Ron</code><li> 
    <li>The off state conductance <code>Goff</code><li> 
    <li>The knee voltage <code>Vknee</code><li> 
  </ul>
  <li>Each converter is equipped with an optional heat port which can be enabled by the parameter
      <code>useHeatPort</code>; the heat ports of all semiconductors are connected,
      so all temepratures of all semiconductors are equal and the heat flow of the converter heat port 
      is determined by the sum of all semiconductor heat flows</li> 
  <li>Each converter containg boolean firing inputs provides variables <code>offStart...</code>     
      to specify the inital conditions of the off state of each semiconductor</li>
  <li>The boolean firing signals are enabled either by means of the a parameter <code>constantEnable</code> or by a conditional signal input, enabled by <code>useConstantEnable = false</code><li>
  <li>The number of phases of multi phase converters is not restricted to three</li>
</ul>

<h4>Literature</h4>

<p>
General background knowledge on power converters and power electronics can be found in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.References\">[Skvarenina01]</a> and 
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.References\">[Luo05]</a>. 
A freely available book is available in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.References\">[Williams2006]</a>.
</p>
</html>"));
end UsersGuide;


package Examples "Examples"
  extends Modelica.Icons.ExamplesPackage;
  package ACDC "AC to DC converter examples"
    extends Modelica.Icons.ExamplesPackage;
    package Thyristor1Pulse "Single pulse rectifier"
      extends Modelica.Icons.ExamplesPackage;
      model Thyristor1Pulse_R "One pulse rectifier with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.Thyristor1Pulse(
            pulse2(
            useConstantFiringAngle=true,
            f=f,
            constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(idealthyristor.n, resistor.p) annotation (Line(
            points={{4.44089e-16,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end Thyristor1Pulse_R;

      model Thyristor1Pulse_R_Characteristic
        "Control characteristic of one pulse rectifier with resitive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.Thyristor1Pulse(
            pulse2(useConstantFiringAngle=false, f=f));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-40,-70})));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, idealthyristor.n) annotation (Line(
            points={{30,40},{0,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2.firingAngle) annotation (Line(
            points={{-40,-59},{-40,-12}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulse2.firingAngle</code> to see control characteristic of this type of rectifier with resistive load.</p>
</html>"));
      end Thyristor1Pulse_R_Characteristic;
    end Thyristor1Pulse;

    package ThyristorBridge2Pulse "Two pulse Graetz bridge"
      extends Modelica.Icons.ExamplesPackage;
      model ThyristorBridge2Pulse_R
        "Graetz thyristor bridge rectifier with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2Pulse_R;

      model ThyristorBridge2Pulse_RL
        "Graetz thyristor bridge rectifier with R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2Pulse_RL;

      model ThyristorBridge2Pulse_RLV
        "Graetz thyristor bridge rectifier with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2Pulse_RLV;

      model ThyristorBridge2Pulse_RLV_Characteristic
        "Characteristic of Graetz thyristor bridge rectifier with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
            pulse2(useConstantFiringAngle=false));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vdi0=2/pi*sin(pi/2)*sqrt(2)*Vrms
          "Ideal max. DC voltage";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,70})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2.firingAngle) annotation (Line(
            points={{-30,59},{-30,-14}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulse2.firingAngle</code> to see control characteristic of this type of rectifier with R-L load including active voltage.</p>
</html>"));
      end ThyristorBridge2Pulse_RLV_Characteristic;

      model ThyristorBridge2Pulse_DC_Drive
        "Graetz thyristor bridge feeding a DC drive"
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms=dcpmData.VaNominal/(2/pi*sin(pi
            /2)*sqrt(2)) "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        parameter Modelica.SIunits.ApparentPower SMains=250E3
          "Mains short circuit apparent power";
        parameter Real lamdaMains=0.1 "Mains short circuit power factor";
        final parameter Modelica.SIunits.Impedance ZMains=Vrms^2/SMains
          "Mains short circuit impedance";
        final parameter Modelica.SIunits.Resistance RMains=ZMains*lamdaMains
          "Mains resistance" annotation (Evaluate=true);
        final parameter Modelica.SIunits.Inductance LMains=ZMains*sqrt(1 -
            lamdaMains^2)/(2*pi*f) "Mains inductance" annotation (Evaluate=true);
        parameter Modelica.SIunits.Inductance Ld=10*dcpmData.La
          "Smoothing inductance" annotation (Evaluate=true);
        final parameter Modelica.SIunits.Torque tauNominal=dcpmData.ViNominal*
            dcpmData.IaNominal/dcpmData.wNominal "Nominal torque";
        output Modelica.SIunits.AngularVelocity w(displayUnit="rpm") = dcpm.wMechanical;
        output Modelica.SIunits.Torque tau=dcpm.tauShaft;
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              visible=true, transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*
              Vrms, freqHz=f) annotation (Placement(visible=true,
              transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorBridge2Pulse
          rectifier(useHeatPort=false)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2Pulse
          pulse2(useConstantFiringAngle=false, useFilter=true) annotation (
            Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,-30})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=Ld) annotation (
            Placement(visible=true, transformation(
              origin={30,-10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet
          dcpm(
          VaNominal=dcpmData.VaNominal,
          IaNominal=dcpmData.IaNominal,
          wNominal=dcpmData.wNominal,
          TaNominal=dcpmData.TaNominal,
          Ra=dcpmData.Ra,
          TaRef=dcpmData.TaRef,
          La=dcpmData.La,
          Jr=dcpmData.Jr,
          useSupport=false,
          Js=dcpmData.Js,
          frictionParameters=dcpmData.frictionParameters,
          coreParameters=dcpmData.coreParameters,
          strayLoadParameters=dcpmData.strayLoadParameters,
          brushParameters=dcpmData.brushParameters,
          phiMechanical(fixed=true),
          wMechanical(fixed=true, start=dcpmData.wNominal),
          TaOperational=293.15,
          alpha20a=dcpmData.alpha20a,
          ia(start=0, fixed=true)) annotation (Placement(transformation(extent=
                  {{10,-90},{30,-70}}, rotation=0)));
        parameter
          Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData
          dcpmData "Data record of PM excited DC machine"
          annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque
          annotation (Placement(transformation(extent={{60,-90},{40,-70}})));
        Modelica.Blocks.Sources.Ramp ramp(
          duration=10,
          startTime=5,
          height=tauNominal,
          offset=-tauNominal)
          annotation (Placement(transformation(extent={{90,-90},{70,-70}})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-30,-60})));
        Modelica.Electrical.Analog.Basic.Resistor rMains(R=RMains) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-80,30})));
        Modelica.Electrical.Analog.Basic.Inductor lMains(L=LMains) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-80,60})));
      initial equation
        lMains.i = 0;
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage.n) annotation (Line(
            points={{-80,-40},{-80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage.n, rectifier.ac_n) annotation (Line(
            points={{-80,-10},{-60,-10},{-60,-6},{-40,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n) annotation (Line(
            points={{-19.8,-6},{-10,-6},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-20,6},{-10,6},{-10,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,-19},{-36,-12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
            points={{-24,-19},{-24,-12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
            points={{-40,-24},{-54,-24},{-54,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
            points={{-40,-6},{-50,-6},{-50,-36},{-40,-36}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, dcpm.pin_ap) annotation (Line(
            points={{30,-20},{30,-70},{26,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(torque.flange, dcpm.flange) annotation (Line(
            points={{40,-80},{30,-80}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(ramp.y, torque.tau) annotation (Line(
            points={{69,-80},{62,-80}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rectifier.dc_p, inductor.p) annotation (Line(
            points={{-20,6},{-10,6},{-10,40},{30,40},{30,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, dcpm.pin_an) annotation (Line(
            points={{10,-40},{10,-70},{14,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const.y, pulse2.firingAngle) annotation (Line(
            points={{-30,-49},{-30,-42}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rMains.n, lMains.p) annotation (Line(
            points={{-80,40},{-80,50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rMains.p, sinevoltage.p) annotation (Line(
            points={{-80,20},{-80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_p, lMains.n) annotation (Line(
            points={{-40,6},{-60,6},{-60,70},{-80,70}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=15,
            Interval=0.0002,
            Tolerance=1e-006),
          Documentation(info="<html>
<p>
In this example a PM excited DC machine is started with nominal torque at nominal speed. After 5 seconds, load torque is reduced to zero over a period of additional 10 seconds. At 15 seconds, the machine is operating at no load.
</p>

<p>
Plot torque <code>tau</code>, current <code>currentSensor.i</code> and average current <code>meanCurrent.y</code>. Also plot speed <code>w</code>, voltage <code>voltageSensor.v</code> and the average voltage <code>meanVoltage.y</code>.</p>  
</html>"));
      end ThyristorBridge2Pulse_DC_Drive;
    end ThyristorBridge2Pulse;

    package ThyristorCenterTap2Pulse "Examples of Power Electronics with M2C"
      extends Modelica.Icons.ExamplesPackage;
      model ThyristorCenterTap2Pulse_R
        "Two pulse thyristor rectifier with center tap and resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2Pulse_R;

      model ThyristorCenterTap2Pulse_RL
        "Two pulse thyristor rectifier with center tap and R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2Pulse_RL;

      model ThyristorCenterTap2Pulse_RLV
        "Two pulse thyristor rectifier with center tap and R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
            pulse2(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(start=0,
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2Pulse_RLV;

      model ThyristorCenterTap2Pulse_RLV_Characteristic
        "Characteristic of two pulse thyristor rectifier with center tap and R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
            pulse2(useConstantFiringAngle=false));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-30,-60})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2.firingAngle) annotation (Line(
            points={{-30,-49},{-30,-12}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulse2.firingAngle</code> to see control characteristic of this type of rectifier with R-L load including active voltage.</p>
</html>"));
      end ThyristorCenterTap2Pulse_RLV_Characteristic;
      annotation (Icon(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.1,
            grid={2,2})), Diagram(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.1,
            grid={2,2})));
    end ThyristorCenterTap2Pulse;

    package ThyristorCenterTapmPulse
      "m pulse thyristor rectifier with center tap"
      extends Modelica.Icons.ExamplesPackage;
      model ThyristorCenterTapmPulse_R
        "2*m pulse thyristor rectifier with center tap and resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
            pulsem(constantFiringAngle=constantFiringAngle), m=6);
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTapmPulse_R;

      model ThyristorCenterTapmPulse_RL
        "2*m pulse thyristor rectifier with center tap and R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
            pulsem(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTapmPulse_RL;

      model ThyristorCenterTapmPulse_RLV
        "2*m pulse thyristor rectifier with center tap and R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
            pulsem(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-50 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{30,-40},{30,-48},{30,-48},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTapmPulse_RLV;

      model ThyristorCenterTapmPulse_RLV_Characteristic
        "Characteristic of 2*m pulse thyristor rectifier with center tap and R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
            pulsem(useConstantFiringAngle=false));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-50 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-30,-70})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, constantVoltage.n) annotation (Line(
            points={{10,-50},{30,-50},{30,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulsem.firingAngle) annotation (Line(
            points={{-30,-59},{-30,-12}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulsem.firingAngle</code> to see control characteristic of this type of rectifier with R-L load including active voltage.</p>
</html>"));
      end ThyristorCenterTapmPulse_RLV_Characteristic;
    end ThyristorCenterTapmPulse;

    package ThyristorBridge2mPulse "2*m pulse thyristor bridge"
      extends Modelica.Icons.ExamplesPackage;
      model ThyristorBridge2mPulse_R
        "2*m pulse thyristor rectifier bridge with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2mPulse_R;

      model ThyristorBridge2mPulse_RL
        "2*m pulse thyristor rectifier bridge with R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle), rectifier(
              offStart_p=fill(true, m), offStart_n=fill(true, m)));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(start=0,
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2mPulse_RL;

      model ThyristorBridge2mPulse_RLV
        "2*m pulse thyristor rectifier bridge with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(start=0,
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorBridge2mPulse_RLV;

      model ThyristorBridge2mPulse_RLV_Characteristic
        "Characteristic of 2*m pulse thyristor rectifier bridge with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
            pulse2m(useConstantFiringAngle=false));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vdi0=
            Modelica.Electrical.MultiPhase.Functions.factorY2DC(m)*Vrms
          "Ideal max. DC voltage";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-30,-50})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, constantVoltage.n) annotation (Line(
            points={{10,-40},{30,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2m.firingAngle) annotation (Line(
            points={{-30,-39},{-30,-12}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulse2m.firingAngle</code> to see control characteristic of this type of rectifier with R-L load including active voltage.</p>
</html>"));
      end ThyristorBridge2mPulse_RLV_Characteristic;

      model ThyristorBridge2mPulse_DC_Drive
        "2m pulse thyristor bridge feeding a DC drive"
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms=dcpmData.VaNominal/
            Modelica.Electrical.MultiPhase.Functions.factorY2DC(m)
          "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        parameter Modelica.SIunits.ApparentPower SMains=250E3
          "Mains short circuit apparent power";
        parameter Real lamdaMains=0.1 "Mains short circuit power factor";
        final parameter Modelica.SIunits.Impedance ZMains=Vrms^2/SMains*m
          "Mains short circuit impedance";
        final parameter Modelica.SIunits.Resistance RMains=ZMains*lamdaMains
          "Mains resistance" annotation (Evaluate=true);
        final parameter Modelica.SIunits.Inductance LMains=ZMains*sqrt(1 -
            lamdaMains^2)/(2*pi*f) "Mains inductance" annotation (Evaluate=true);
        parameter Modelica.SIunits.Inductance Ld=3*dcpmData.La
          "Smoothing inductance" annotation (Evaluate=true);
        final parameter Modelica.SIunits.Torque tauNominal=dcpmData.ViNominal*
            dcpmData.IaNominal/dcpmData.wNominal "Nominal torque";
        output Modelica.SIunits.AngularVelocity w(displayUnit="rpm") = dcpm.wMechanical;
        output Modelica.SIunits.Torque tau=dcpm.tauShaft;
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sinevoltage(
          m=m,
          each final V=fill(sqrt(2)*Vrms, m),
          each freqHz=fill(f, m)) annotation (Placement(visible=true,
              transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorBridge2mPulse
          rectifier(useHeatPort=false, m=m)
          annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*m*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2mPulse
          pulse2(
          useConstantFiringAngle=false,
          useFilter=true,
          m=m) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-38,-40})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=Ld) annotation (
            Placement(visible=true, transformation(
              origin={30,-10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet
          dcpm(
          VaNominal=dcpmData.VaNominal,
          IaNominal=dcpmData.IaNominal,
          wNominal=dcpmData.wNominal,
          TaNominal=dcpmData.TaNominal,
          Ra=dcpmData.Ra,
          TaRef=dcpmData.TaRef,
          La=dcpmData.La,
          Jr=dcpmData.Jr,
          useSupport=false,
          Js=dcpmData.Js,
          frictionParameters=dcpmData.frictionParameters,
          coreParameters=dcpmData.coreParameters,
          strayLoadParameters=dcpmData.strayLoadParameters,
          brushParameters=dcpmData.brushParameters,
          phiMechanical(fixed=true),
          wMechanical(fixed=true, start=dcpmData.wNominal),
          TaOperational=293.15,
          alpha20a=dcpmData.alpha20a,
          ia(start=0, fixed=true)) annotation (Placement(transformation(extent=
                  {{10,-90},{30,-70}}, rotation=0)));
        parameter
          Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData
          dcpmData "Data record of PM excited DC machine"
          annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque
          annotation (Placement(transformation(extent={{60,-90},{40,-70}})));
        Modelica.Blocks.Sources.Ramp ramp(
          duration=10,
          startTime=5,
          height=tauNominal,
          offset=-tauNominal)
          annotation (Placement(transformation(extent={{90,-90},{70,-70}})));
        Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-38,-70})));
        Modelica.Electrical.MultiPhase.Basic.Resistor rMains(m=m, R=fill(RMains,
              m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-80,30})));
        Modelica.Electrical.MultiPhase.Basic.Inductor lMains(m=m, L=fill(LMains,
              m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-80,60})));
        Modelica.Electrical.MultiPhase.Basic.MultiStarResistance earthing(m=m)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-30})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-30,-60},{-10,-40}})));
      initial equation
        lMains.i[1:m - 1] = zeros(m - 1);
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n) annotation (Line(
            points={{-28,-6},{-10,-6},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-28,6},{-10,6},{-10,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_p, rectifier.fire_p) annotation (Line(
            points={{-44,-29},{-44,-11.8}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
            points={{-32,-29},{-32,-12}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(inductor.n, dcpm.pin_ap) annotation (Line(
            points={{30,-20},{30,-70},{26,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(torque.flange, dcpm.flange) annotation (Line(
            points={{40,-80},{30,-80}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(ramp.y, torque.tau) annotation (Line(
            points={{69,-80},{62,-80}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rectifier.dc_p, inductor.p) annotation (Line(
            points={{-28,6},{-10,6},{-10,40},{30,40},{30,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, dcpm.pin_an) annotation (Line(
            points={{10,-40},{10,-70},{14,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(const.y, pulse2.firingAngle) annotation (Line(
            points={{-38,-59},{-38,-52}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rMains.plug_p, sinevoltage.plug_p) annotation (Line(
            points={{-80,20},{-80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(lMains.plug_p, rMains.plug_n) annotation (Line(
            points={{-80,50},{-80,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(lMains.plug_n, rectifier.ac) annotation (Line(
            points={{-80,70},{-60,70},{-60,6.66134e-16},{-48,6.66134e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac, pulse2.ac) annotation (Line(
            points={{-48,6.66134e-16},{-60,6.66134e-16},{-60,-40},{-48,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.plug_n, earthing.plug) annotation (Line(
            points={{-80,-10},{-80,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, ground.p) annotation (Line(
            points={{-28,-6},{-20,-6},{-20,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          experiment(
            StopTime=15,
            Interval=0.0002,
            Tolerance=1e-006));
      end ThyristorBridge2mPulse_DC_Drive;
    end ThyristorBridge2mPulse;

    package ThyristorCenterTap2mPulse
      "2*m pulse thyristor rectifier with center tap"
      extends Modelica.Icons.ExamplesPackage;
      model ThyristorCenterTap2mPulse_R
        "m pulse thyristor rectifier bridge with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2mPulse_R;

      model ThyristorCenterTap2mPulse_RL
        "m pulse thyristor rectifier bridge with R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2mPulse_RL;

      model ThyristorCenterTap2mPulse_RLV
        "m pulse thyristor rectifier bridge with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
            pulse2m(constantFiringAngle=constantFiringAngle));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle constantFiringAngle=30*pi/180
          "Firing angle";
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(start=0,
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{30,-40},{30,-48},{30,-48},{30,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ThyristorCenterTap2mPulse_RLV;

      model ThyristorCenterTap2mPulse_RLV_Characteristic
        "Characteristic of m pulse thyristor rectifier bridge with R-L load and voltage"
        extends
          Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
            pulse2m(useConstantFiringAngle=false));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R=20 "Load resistance";
        parameter Modelica.SIunits.Inductance L=1 "Load resistance"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(visible=true, transformation(
              origin={30,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(start=0,
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={30,0},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              VDC) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={-30,-70})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, constantVoltage.n) annotation (Line(
            points={{10,-50},{30,-50},{30,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ramp.y, pulse2m.firingAngle) annotation (Line(
            points={{-30,-59},{-30,-12}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.p, rectifier.dc_p) annotation (Line(
            points={{30,40},{-10,40},{-10,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=10,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot average voltage <code>meanVoltage.v</code> versus firingAngle <code>pulse2m.firingAngle</code> to see control characteristic of this type of rectifier with R-L load including active voltage.</p>
</html>"));
      end ThyristorCenterTap2mPulse_RLV_Characteristic;
    end ThyristorCenterTap2mPulse;

    package ExampleTemplates "Templates of examples"
      partial model Thyristor1Pulse "Template of single pulse rectifier"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*
              Vrms, freqHz=f) annotation (Placement(visible=true,
              transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              visible=true, transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanVoltage(f=f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Blocks.Math.Mean meanCurrent(f=f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2Pulse
          pulse2(f=f, useFilter=false) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-40,0})));
        Modelica.Electrical.Analog.Ideal.IdealThyristor idealthyristor(off(
              fixed=true)) annotation (Placement(visible=true, transformation(
              origin={-10,40},
              extent={{-10,10},{10,-10}},
              rotation=0)));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage.n) annotation (Line(
            points={{-80,-40},{-80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage.p, idealthyristor.p) annotation (Line(
            points={{-80,10},{-80,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, currentSensor.n) annotation (Line(
            points={{-80,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_p, idealthyristor.fire) annotation (Line(
            points={{-46,11},{-46,20},{-3,20},{-3,29}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(idealthyristor.n, voltagesensor.p) annotation (Line(
            points={{0,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.p, pulse2.ac_p) annotation (Line(
            points={{-80,10},{-80,10},{-80,20},{-60,20},{-60,6},{-50,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.n, pulse2.ac_n) annotation (Line(
            points={{-80,-10},{-80,-10},{-80,-20},{-60,-20},{-60,-6},{-50,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor.</p>
</html>"));
      end Thyristor1Pulse;
      extends Modelica.Icons.Package;
      partial model ThyristorBridge2Pulse
        "Template of two pulse Graetz thyristor bridge"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              visible=true, transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*
              Vrms, freqHz=f) annotation (Placement(visible=true,
              transformation(
              origin={-80,20},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorBridge2Pulse
          rectifier(useHeatPort=false, offStart_p1=true)
          annotation (Placement(transformation(extent={{-40,24},{-20,44}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2Pulse
          pulse2(f=f, useFilter=false) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,-2})));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage.n) annotation (Line(
            points={{-80,-40},{-80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage.p, rectifier.ac_p) annotation (Line(
            points={{-80,30},{-80,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.n, rectifier.ac_n) annotation (Line(
            points={{-80,10},{-80,10},{-80,-8},{-50,-8},{-50,28},{-40,28}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n) annotation (Line(
            points={{-19.8,28},{-10,28},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,9},{-36,22}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
            points={{-24,9},{-24,22}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
            points={{-40,4},{-60,4},{-60,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
            points={{-40,28},{-50,28},{-50,-8},{-40,-8}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          Documentation(info="<html>
<p>Two pulse thyristor bridge example template.</p>
</html>"));
      end ThyristorBridge2Pulse;

      model ThyristorCenterTap2Pulse
        "Template of two pulse thyristor rectifier with center tap"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              visible=true, transformation(
              origin={-90,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage_n(V=sqrt(2)*
              Vrms, freqHz=f) annotation (Placement(visible=true,
              transformation(
              origin={-80,-13.9999},
              extent={{-9.999890000000001,-10},{10,10}},
              rotation=-90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage_p(V=sqrt(2)*
              Vrms, freqHz=f) annotation (Placement(visible=true,
              transformation(
              origin={-80,14},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTap2Pulse
          rectifier
          annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2Pulse
          pulse2(f=f, useFilter=false) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,0})));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{-4.44089e-16,-60},{-4.44089e-16,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sinevoltage_p.n, sinevoltage_n.p) annotation (Line(
            points={{-80,4},{-80,-4.00001}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, sinevoltage_p.n) annotation (Line(
            points={{-90,-40},{-90,0},{-80,0},{-80,4}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage_p.p, rectifier.ac_p) annotation (Line(
            points={{-80,24},{-80,46},{-40,46}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage_n.n, rectifier.ac_n) annotation (Line(
            points={{-80,-23.9999},{-80,-32},{-50,-32},{-50,34},{-40,34}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, currentSensor.n) annotation (Line(
            points={{-90,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.p, rectifier.dc_p) annotation (Line(
            points={{50,20},{50,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,11},{-36,28}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
            points={{-24,11},{-24,28}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
            points={{-40,34},{-50,34},{-50,-6},{-40,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
            points={{-40,6},{-60,6},{-60,46},{-40,46}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=true,
              initialScale=0.1,
              grid={2,2})),
          Diagram(coordinateSystem(
              extent={{-100,-100},{100,100}},
              preserveAspectRatio=false,
              initialScale=0.1,
              grid={2,2}), graphics),
          Documentation(info="<html>
<p>Two pulse thyristor center tap example template.</p>
</html>"));
      end ThyristorCenterTap2Pulse;

      partial model ThyristorBridge2mPulse
        "Template of 2*m pulse thyristor rectifier"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-30})));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorBridge2mPulse
          rectifier(final m=m)
          annotation (Placement(transformation(extent={{-40,24},{-20,44}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*m*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2mPulse
          pulse2m(
          m=m,
          f=f,
          useFilter=false) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,0})));
        Modelica.Electrical.MultiPhase.Basic.MultiStarResistance
          multiStarResistance(final m=m) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-60})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-100},{-70,-80}})));
      equation
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-60},{0,-60},{0,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, rectifier.ac) annotation (Line(
            points={{-80,-20},{-80,34},{-40,34}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_n, currentSensor.n) annotation (Line(
            points={{-20,28},{-10,28},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,11},{-36,22.2}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fire_n, rectifier.fire_n) annotation (Line(
            points={{-24,11},{-24,22}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.ac, sineVoltage.plug_p) annotation (Line(
            points={{-40,6.66134e-16},{-80,6.66134e-16},{-80,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage.plug_n, multiStarResistance.plug) annotation (Line(
            points={{-80,-40},{-80,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(multiStarResistance.pin, ground.p) annotation (Line(
            points={{-80,-70},{-80,-80}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Documentation(info="<html>
<p><code>2*m</code> pulse thyristor bridge example template, where <code>m</code> is the number of phases.</p>
</html>"));
      end ThyristorBridge2mPulse;

      partial model ThyristorCenterTapmPulse
        "Template of 2*m pulse rectifier with center tap"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-80,-100},{-60,-80}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,-30})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_p(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,10})));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTapmPulse
          rectifier(final m=m)
          annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=m*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-50})));
        Modelica.Blocks.Math.Mean meanCurrent(f=m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-70})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageBridge2mPulse
          pulsem(
          m=m,
          f=f,
          useFilter=false) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,0})));
      equation
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-70,-40},{-70,-80}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-70},{0,-70},{0,-60},{-6.66134e-16,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-50},{50,-50},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulsem.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,11},{-36,28}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulsem.ac, sineVoltage_p.plug_p) annotation (Line(
            points={{-40,6.66134e-16},{-46,6.66134e-16},{-46,0},{-50,0},{-50,40},
                {-70,40},{-70,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.plug_p, sineVoltage_p.plug_n) annotation (Line(
            points={{-70,-20},{-70,1.33227e-15}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, currentSensor.n) annotation (Line(
            points={{-70,-40},{-70,-50},{-10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rectifier.ac, sineVoltage_p.plug_p) annotation (Line(
            points={{-40,40},{-70,40},{-70,20}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Documentation(info="<html>
<p><code>m</code> pulse thyristor center tap example template, where <code>m</code> is the number of phases.</p>
</html>"));
      end ThyristorCenterTapmPulse;

      partial model ThyristorCenterTap2mPulse
        "Template of 2*m pulse rectifier with center tap"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms=110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        // parameter Modelica.SIunits.Angle constantFiringAngle = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-100},{-70,-80}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-100,-10})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_p(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,10})));
        Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTap2mPulse
          rectifier(final m=m)
          annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation (Placement(visible=true, transformation(
              origin={50,10},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=2*m*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-50})));
        Modelica.Blocks.Math.Mean meanCurrent(f=2*m*f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-70})));
        Modelica_Electrical_PowerConverters.ACDC.Control.VoltageCenterTap2mPulse
          pulse2m(
          m=m,
          f=f,
          useFilter=false) annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=180,
              origin={-30,0})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_n(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,-20})));
      equation
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-100,-20},{-100,-50},{-80,-50},{-80,-80}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{68,-70},{0,-70},{0,-60},{-6.66134e-16,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, meanVoltage.u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v, rootMeanSquareVoltage.u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rectifier.dc_p, voltagesensor.p) annotation (Line(
            points={{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-50},{50,-50},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fire_p, rectifier.fire_p) annotation (Line(
            points={{-36,11},{-36,28}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(pulse2m.ac, sineVoltage_p.plug_p) annotation (Line(
            points={{-40,6.66134e-16},{-46,6.66134e-16},{-46,46},{-70,46},{-70,
                20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_p.plug_n, sineVoltage_n.plug_p) annotation (Line(
            points={{-70,1.33227e-15},{-70,-4},{-70,-4},{-70,-4},{-70,-4},{-70,
                -10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_n.plug_n, rectifier.ac_n) annotation (Line(
            points={{-70,-30},{-70,-40},{-50,-40},{-50,34},{-40,34}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_p.plug_p, rectifier.ac_p) annotation (Line(
            points={{-70,20},{-70,46},{-40,46}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.plug_p, sineVoltage_p.plug_n) annotation (Line(
            points={{-100,0},{-70,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, currentSensor.n) annotation (Line(
            points={{-100,-20},{-100,-50},{-10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(pulse2m.fire_n, rectifier.fire_n) annotation (Line(
            points={{-24,11},{-24,28}},
            color={255,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Documentation(info="<html>
<p><code>2*m</code> pulse thyristor center tap example template, where <code>m</code> is the number of phases.</p>
</html>"));
      end ThyristorCenterTap2mPulse;
    end ExampleTemplates;
  end ACDC;

  package DCAC "DC to AC converter examples"
    extends Modelica.Icons.ExamplesPackage;
    package SinglePhaseTwoLevel "Single phase two level inverter examples"
      extends Modelica.Icons.ExamplesPackage;
      model SinglePhaseTwoLevel_R
        "Single phase DC to AC converter with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.DCAC.ExampleTemplates.SinglePhaseTwoLevel(
            sine(amplitude=0.5, offset=0.5));
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,10})));
      equation
        connect(resistor.p, inverter.ac) annotation (Line(
            points={{40,20},{40,30},{-30,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{40,1.33227e-15},{40,-70},{10,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StartTime=0,
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>. The instantaneous voltage and current directly show the switch pattern of the inverter. The average voltage and average current reveal the fundamental wave of the voltage and current, each of them being basically in phase with the command <code>sine.y</code>.</p>
</html>"));
      end SinglePhaseTwoLevel_R;

      model SinglePhaseTwoLevel_RL
        "Single phase DC to AC converter with R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.DCAC.ExampleTemplates.SinglePhaseTwoLevel(
            sine(amplitude=0.5, offset=0.5));
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        parameter Modelica.SIunits.Inductance L=1 "Inductance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,10})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-22})));
      equation
        connect(resistor.p, inverter.ac) annotation (Line(
            points={{40,20},{40,30},{-30,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{40,0},{40,-12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{40,-32},{40,-70},{10,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StartTime=0,
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>Plot current <code>currentSensor.i</code>, avarage current <code>meanCurrent.y</code>, voltage <code>voltageSensor.v</code> and average voltage <code>meanVoltage.v</code>. The instantaneous voltage directly show the switch pattern of the inverter. The current shows a particular ripple determined by the input voltage and the switching frequency. The average voltage is basically in phase with the command <code>sine.y</code>. The average current has a phase shift due to the R-L load.</p>
</html>"));
      end SinglePhaseTwoLevel_RL;
    end SinglePhaseTwoLevel;

    package ExampleTemplates "Templates of examples"
      extends Modelica.Icons.Package;
      partial model SinglePhaseTwoLevel
        "Single phas two level inverter including control"
        extends Icons.ExampleTemplate;
        parameter Modelica.SIunits.Frequency f=1000 "Switching frequency";
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_n(V=
              50) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,10})));
        Modelica_Electrical_PowerConverters.DCAC.SinglePhase2Level inverter(
            useHeatPort=false)
          annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(extent={{10,-80},{-10,-60}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={70,-10})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-90,40})));
        Modelica_Electrical_PowerConverters.DCDC.Control.SignalPWM signalPWM(
            useConstantDutyCycle=false, f=f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-40,-30})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage_p(V=
              50) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-70,50})));
        Modelica.Blocks.Sources.Sine sine(freqHz=50)
          annotation (Placement(transformation(extent={{-30,-64},{-50,-44}})));
        Modelica.Blocks.Math.Mean meanCurrent(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={30,-90})));
        Modelica.Blocks.Math.Mean meanVoltage(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={82,50})));
      equation
        connect(voltageSensor.n, currentSensor.p) annotation (Line(
            points={{70,-20},{70,-70},{10,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(signalPWM.fire, inverter.fire_p) annotation (Line(
            points={{-46,-19},{-46,18}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(constantVoltage_p.n, constantVoltage_n.p) annotation (Line(
            points={{-70,40},{-70,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage_p.p, inverter.dc_p) annotation (Line(
            points={{-70,60},{-50,60},{-50,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage_n.n, inverter.dc_n) annotation (Line(
            points={{-70,0},{-50,0},{-50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, constantVoltage_p.n) annotation (Line(
            points={{-90,30},{-70,30},{-70,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.n, ground.p) annotation (Line(
            points={{-10,-70},{-90,-70},{-90,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.p, inverter.ac) annotation (Line(
            points={{70,4.44089e-16},{70,30},{-30,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sine.y, signalPWM.dutyCycle) annotation (Line(
            points={{-51,-54},{-60,-54},{-60,-30},{-52,-30}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(inverter.fire_n, signalPWM.notFire) annotation (Line(
            points={{-34,18},{-34,-19}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.v, meanVoltage.u) annotation (Line(
            points={{80,-10},{82,-10},{82,-8},{82,-8},{82,38},{82,38}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(meanCurrent.u, currentSensor.i) annotation (Line(
            points={{18,-90},{0,-90},{0,-80}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Documentation(info="<html>
<p>Single phase two level example template including supply and sensors; load is not yet included.</p>
</html>"));
      end SinglePhaseTwoLevel;
    end ExampleTemplates;
    annotation (Documentation(info="<html>
</html>"));
  end DCAC;

  package DCDC "DC to DC converter examples"
    extends Modelica.Icons.ExamplesPackage;
    package ChopperStepDown "Step down chopper"
      extends Modelica.Icons.ExamplesPackage;
      model ChopperStepDown_R "Step down chopper with resistive load"
        extends ExampleTemplates.ChopperStepDown;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,50})));
      equation
        connect(chopperStepDown.dc_p2, resistor.p) annotation (Line(
            points={{-40,6},{-30,6},{-30,70},{30,70},{30,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,40},{30,-6},{0,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StartTime=0,
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>This example demonstrates the switching on of a resistive load operated by a step down chopper.
DC output voltage is equal to <code>dutyCycle</code> times the input voltage.
Plot current <code>currentSensor.i</code>, averaged current <code>meanCurrent.y</code>, total voltage <code>voltageSensor.v</code> and voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end ChopperStepDown_R;

      model ChopperStepDown_RL "Step down chopper with R-L load"
        extends ExampleTemplates.ChopperStepDown;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        parameter Modelica.SIunits.Inductance L=1 "Inductance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,50})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L, i(fixed=true,
              start=0)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,10})));
      equation
        connect(chopperStepDown.dc_p2, resistor.p) annotation (Line(
            points={{-40,6},{-30,6},{-30,70},{30,70},{30,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,40},{30,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,1.33227e-15},{30,-6},{0,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StartTime=0,
            StopTime=0.1,
            Tolerance=1e-06,
            Interval=0.0002),
          Documentation(info="<html>
<p>This example demonstrates the switching on of an R-L load operated by a step down chopper.
DC output voltage is equal to <code>dutyCycle</code> times the input voltage.
Plot current <code>currentSensor.i</code>, averaged current <code>meanCurrent.y</code>, total voltage <code>voltageSensor.v</code> and voltage <code>meanVoltage.v</code>. The waveform the average current is determined by the time constant <code>L/R</code> of the load.</p>
</html>"));
      end ChopperStepDown_RL;
    end ChopperStepDown;

    package HBridge "H bridge converter"
      extends Modelica.Icons.ExamplesPackage;
      model HBridge_R "H bridge DC/DC converter with resistive load"
        extends
          Modelica_Electrical_PowerConverters.Examples.DCDC.ExampleTemplates.HBridge;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={20,50})));
      equation
        connect(resistor.p, hbridge.dc_p2) annotation (Line(
            points={{20,60},{20,70},{-30,70},{-30,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{20,40},{20,-6},{0,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=0.1,
            Interval=0.0002,
            Tolerance=1e-06),
          Documentation(info="<html>
<p>This example demonstrates the switching on of a resistive load operated by an H bridge.
DC output voltage is equal to <code>2 * (dutyCycle - 0.5)</code> times the input voltage.
Plot current <code>currentSensor.i</code>, averaged current <code>meanCurrent.y</code>, total voltage <code>voltageSensor.v</code> and voltage <code>meanVoltage.v</code>.</p>
</html>"));
      end HBridge_R;

      model HBridge_RL "H bridge DC/DC converter with R-L load"
        extends
          Modelica_Electrical_PowerConverters.Examples.DCDC.ExampleTemplates.HBridge;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Resistance R=100 "Resistance";
        parameter Modelica.SIunits.Inductance L=1 "Inductance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={20,50})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(i(fixed=true, start=
                0), L=L) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={20,10})));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{20,40},{20,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{20,0},{20,-6},{4.44089e-16,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, hbridge.dc_p2) annotation (Line(
            points={{20,60},{20,70},{-30,70},{-30,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          Documentation(info="<html>
<p>This example demonstrates the switching on of an R-L load operated by an H bridge.
DC output voltage is equal to <code>2 * (dutyCycle - 0.5)</code> times the input voltage.
Plot current <code>currentSensor.i</code>, averaged current <code>meanCurrent.y</code>, total voltage <code>voltageSensor.v</code> and voltage <code>meanVoltage.v</code>. The waveform the average current is determined by the time constant <code>L/R</code> of the load.</p>
</html>"),experiment(
            StopTime=0.1,
            Interval=0.0002,
            Tolerance=1e-06));
      end HBridge_RL;

      model HBridge_DC_Drive "H bridge DC/DC converter with DC drive"
        extends
          Modelica_Electrical_PowerConverters.Examples.DCDC.ExampleTemplates.HBridge(
            signalPWM(useConstantDutyCycle=false), constantVoltage(V=120));
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Inductance Ld=3*dcpmData.La
          "Smoothing inductance";
        final parameter Modelica.SIunits.Torque tauNominal=dcpmData.ViNominal*
            dcpmData.IaNominal/dcpmData.wNominal "Nominal torque";
        parameter Real dMin=0.2 "Minimum duty cycle";
        parameter Real dMax=1 - dMin "Maximum duty cycle";
        Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet
          dcpm(
          VaNominal=dcpmData.VaNominal,
          IaNominal=dcpmData.IaNominal,
          wNominal=dcpmData.wNominal,
          TaNominal=dcpmData.TaNominal,
          Ra=dcpmData.Ra,
          TaRef=dcpmData.TaRef,
          La=dcpmData.La,
          Jr=dcpmData.Jr,
          useSupport=false,
          Js=dcpmData.Js,
          frictionParameters=dcpmData.frictionParameters,
          coreParameters=dcpmData.coreParameters,
          strayLoadParameters=dcpmData.strayLoadParameters,
          brushParameters=dcpmData.brushParameters,
          ia(start=0, fixed=true),
          TaOperational=293.15,
          alpha20a=dcpmData.alpha20a,
          phiMechanical(fixed=true, start=0),
          wMechanical(fixed=true, start=0)) annotation (Placement(
              transformation(extent={{20,-80},{40,-60}}, rotation=0)));
        parameter
          Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData
          dcpmData "Data record of PM excited DC machine"
          annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque
          annotation (Placement(transformation(extent={{70,-80},{50,-60}})));
        Modelica.Blocks.Sources.TimeTable torqueTable(table=[0, 0; 6, 0; 7, -
              tauNominal; 9, -tauNominal; 10, +tauNominal; 15, tauNominal; 16,
              -tauNominal; 18, -tauNominal; 19, 0; 24, 0])
          annotation (Placement(transformation(extent={{100,-80},{80,-60}})));
        Modelica.Blocks.Sources.TimeTable dutyCycleTable(table=[0, 0.5; 3, 0.5;
              4, dMax; 12, dMax; 13, dMin; 21, dMin; 22, 0.5; 24, 0.5])
          annotation (Placement(transformation(extent={{-100,-70},{-80,-50}})));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=Ld) annotation (
            Placement(visible=true, transformation(
              origin={40,30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
      equation
        connect(inductor.n, dcpm.pin_ap) annotation (Line(
            points={{40,20},{40,-60},{36,-60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(dcpm.pin_an, currentSensor.p) annotation (Line(
            points={{24,-60},{24,-6},{4.44089e-16,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(dcpm.flange, torque.flange) annotation (Line(
            points={{40,-70},{50,-70}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(torque.tau, torqueTable.y) annotation (Line(
            points={{72,-70},{79,-70}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(inductor.p, hbridge.dc_p2) annotation (Line(
            points={{40,40},{40,70},{-30,70},{-30,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(dutyCycleTable.y, signalPWM.dutyCycle) annotation (Line(
            points={{-79,-60},{-62,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics),
          experiment(
            StopTime=24,
            Interval=0.0002,
            Tolerance=1e-06),
          Documentation(info="<html>
<p>This example of am H brdge with DC drive demonstrates the operation of the DC machine in four quadrants. 
The DC output voltage is equal to <code>2 * (dutyCycle - 0.5)</code> times the input voltage.</p>

<p>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <th><b>start time (s)</b></th>
    <th><b>machine speed</b>
    <th><b>machine torque</b>
    <th><b>mode</b></th>
  </tr>
  <tr>
    <td>0</td> <td>zero</td> <td>zero</td> <td></td>
  </tr>
  <tr>
    <td>3</td> <td>positive</td> <td>zero</td> <td></td>
  </tr>
  <tr>
    <td>6</td> <td>positive</td> <td>positive</td> <td>motor</td>
  </tr>
  <tr>
    <td>9.5</td> <td>positive</td> <td>negative</td> <td>generator</td>
  </tr>
  <tr>
    <td>12.5</td> <td>negative</td> <td>negative</td> <td>motor</td>
  </tr>
  <tr>
    <td>15.5</td> <td>negative</td> <td>positive</td> <td>generator</td>
  </tr>
  <tr>
    <td>19</td> <td>negative</td> <td>zero</td> <td></td>
  </tr>
  <tr>
    <td>22</td> <td>zero</td> <td>zero</td> <td></td>
  </tr>
</table></p>

<p>
Plot machine current <code>dcpm.ia</code>, averaged current <code>meanCurrent.y</code>, machine speed <code>dcpm.wMechanical</code>, avderage machine speed <code>dcpm.va</code> and torque <code>dcpm.tauElectrical</code>.</p>
</html>"));
      end HBridge_DC_Drive;
    end HBridge;

    package ExampleTemplates "Templates of examples"
      extends Modelica.Icons.Package;
      partial model ChopperStepDown "Step down chopper including control"
        extends Icons.ExampleTemplate;
        parameter Modelica.SIunits.Frequency f=1000 "Switching frequency";
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              100) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,0})));
        Modelica_Electrical_PowerConverters.DCDC.ChopperStepDown
          chopperStepDown(useHeatPort=false)
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(extent={{0,-16},{-20,4}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={60,10})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-40},{-70,-20}})));
        Modelica_Electrical_PowerConverters.DCDC.Control.SignalPWM signalPWM(
            constantDutyCycle=0.25, f=f) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-50,-60})));
        Modelica.Blocks.Math.Mean meanCurrent(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-10,-40})));
        Modelica.Blocks.Math.Mean meanVoltage(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={90,10})));
      equation
        connect(constantVoltage.p, chopperStepDown.dc_p1) annotation (Line(
            points={{-80,10},{-70,10},{-70,6},{-60,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, chopperStepDown.dc_n1) annotation (Line(
            points={{-80,-10},{-70,-10},{-70,-6},{-60,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(chopperStepDown.dc_p2, voltageSensor.p) annotation (Line(
            points={{-40,6},{-30,6},{-30,70},{60,70},{60,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.n, currentSensor.p) annotation (Line(
            points={{60,0},{60,-6},{0,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.n, chopperStepDown.dc_n2) annotation (Line(
            points={{-20,-6},{-39.8,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, ground.p) annotation (Line(
            points={{-80,-10},{-80,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.v, meanVoltage.u) annotation (Line(
            points={{70,10},{78,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(currentSensor.i, meanCurrent.u) annotation (Line(
            points={{-10,-16},{-10,-28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(signalPWM.fire, chopperStepDown.fire_p) annotation (Line(
            points={{-56,-49},{-56,-12}},
            color={255,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),        graphics), Documentation(info="<html>
<p>Step down chopper example template including supply and sensors; load is not yet included</p>
</html>"));
      end ChopperStepDown;

      partial model HBridge "H bridge DC/DC converter"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        parameter Modelica.SIunits.Frequency f=1000 "Switching frequency";
        Modelica_Electrical_PowerConverters.DCDC.HBridge hbridge(useHeatPort=
              false)
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=
              100) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,0})));
        Modelica_Electrical_PowerConverters.DCDC.Control.SignalPWM signalPWM(
            constantDutyCycle=0.6, f=f)
          annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-40},{-70,-20}})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(extent={{0,-16},{-20,4}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={60,10})));
        Modelica.Blocks.Math.Mean meanCurrent(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-10,-40})));
        Modelica.Blocks.Math.Mean meanVoltage(f=f, x0=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={90,10})));
      equation
        connect(hbridge.fire_p, signalPWM.fire) annotation (Line(
            points={{-56,-12},{-56,-49}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(ground.p, constantVoltage.n) annotation (Line(
            points={{-80,-20},{-80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.p, hbridge.dc_p1) annotation (Line(
            points={{-80,10},{-68,10},{-68,6},{-60,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, hbridge.dc_n1) annotation (Line(
            points={{-80,-10},{-68,-10},{-68,-6},{-60,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.n, currentSensor.p) annotation (Line(
            points={{60,0},{60,-6},{0,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.v, meanVoltage.u) annotation (Line(
            points={{70,10},{78,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(currentSensor.i, meanCurrent.u) annotation (Line(
            points={{-10,-16},{-10,-28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(hbridge.dc_p2, voltageSensor.p) annotation (Line(
            points={{-40,6},{-30,6},{-30,70},{60,70},{60,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.n, hbridge.dc_n2) annotation (Line(
            points={{-20,-6},{-40,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(signalPWM.notFire, hbridge.fire_n) annotation (Line(
            points={{-44,-49},{-44,-12}},
            color={255,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Documentation(info="<html>
<p>H bridge example template including suppy and sensors; load is not yet included</p>
</html>"));
      end HBridge;
    end ExampleTemplates;
  end DCDC;
  annotation (Documentation(info="<html>
<p>This is a collection of AC/DC, DC/DC and DC/AC converters.</p>
</html>"));
end Examples;


package ACDC "AC to DC converter"
  package Control "Control components for rectifiers"
    extends Modelica.Icons.Package;
    block Signal2mPulse "Generic control of 2*m pulse rectifiers"
      import Modelica.Constants.pi;
      extends Modelica_Electrical_PowerConverters.Icons.Control;
      parameter Integer m(final min=1) = 3 "Number of phases";
      parameter Boolean useConstantFiringAngle=true
        "Use constant firing angle instead of signal input";
      parameter Modelica.SIunits.Angle constantFiringAngle=0 "Firing angle"
        annotation (Dialog(enable=useConstantFiringAngle));
      parameter Boolean useFilter=true "Enable use of filter"
        annotation (Dialog(tab="Filter"));
      parameter Modelica.SIunits.Frequency f=50 "Frequency"
        annotation (Dialog(tab="Filter", enable=useFilter));
      parameter Modelica.SIunits.Frequency fCut=2*f
        "Cut off frequency of filter"
        annotation (Dialog(tab="Filter", enable=useFilter));
      parameter Modelica.SIunits.Voltage vStart[m]=zeros(m)
        "Start voltage of filter output"
        annotation (Dialog(tab="Filter", enable=useFilter));
      Modelica.Blocks.Interfaces.RealInput firingAngle if not
        useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
            transformation(
            extent={{20,-20},{-20,20}},
            rotation=270,
            origin={0,-120})));
      parameter Modelica.SIunits.Angle firingAngleMax(
        min=0,
        max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      Modelica.Blocks.Sources.Constant constantconstantFiringAngle(final k=
            constantFiringAngle) if useConstantFiringAngle annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=180,
            origin={-30,-80})));
      Modelica.Blocks.Logical.GreaterThreshold positiveThreshold[m](threshold=
            zeros(m)) annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={-60,10})));
      Modelica.Blocks.Logical.LessThreshold negativeThreshold[m](threshold=
            zeros(m)) annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={60,10})));
      Modelica.Blocks.Logical.Timer timerPositive[m] annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={-60,40})));
      Modelica.Blocks.Logical.Timer timerNegative[m] annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={60,40})));
      Modelica.Blocks.Logical.Greater greaterPositive[m] annotation (Placement(
            transformation(
            extent={{10,10},{-10,-10}},
            rotation=270,
            origin={-60,80})));
      Modelica.Blocks.Logical.Greater negativeEqual[m] annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={60,80})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,110})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,110})));
      Modelica.Blocks.Math.Gain gain(final k=1/2/pi/f) annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={0,10})));
      Modelica.Blocks.Routing.Replicator replicator(final nout=m) annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={0,40})));
      Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi,
            firingAngleMax), final uMin=0) annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={0,-20})));
      Modelica.Blocks.Interfaces.RealInput v[m] "Voltages" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-120,0})));
      Filter filter[m](
        each final f=f,
        each final fCut=2*f,
        yStart=vStart) if useFilter annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-80,-80})));
      Modelica.Blocks.Routing.RealPassThrough realPassThrough[m] if not
        useFilter "Pass through in case filter is off"
        annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));
    equation
      connect(positiveThreshold.y, timerPositive.u) annotation (Line(
          points={{-60,21},{-60,28}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeThreshold.y, timerNegative.u) annotation (Line(
          points={{60,21},{60,28}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(timerPositive.y, greaterPositive.u1) annotation (Line(
          points={{-60,51},{-60,68}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeEqual.u1, timerNegative.y) annotation (Line(
          points={{60,68},{60,51}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterPositive.y, fire_p) annotation (Line(
          points={{-60,91},{-60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeEqual.y, fire_n) annotation (Line(
          points={{60,91},{60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(gain.y, replicator.u) annotation (Line(
          points={{1.33227e-15,21},{1.33227e-15,28},{-2.66454e-15,28}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, greaterPositive.u2) annotation (Line(
          points={{1.55431e-15,51},{1.55431e-15,60},{-52,60},{-52,68}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, negativeEqual.u2) annotation (Line(
          points={{1.55431e-15,51},{1.55431e-15,52},{0,52},{0,60},{52,60},{52,
              68}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, gain.u) annotation (Line(
          points={{1.55431e-15,-9},{1.55431e-15,-2},{-2.66454e-15,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(firingAngle, limiter.u) annotation (Line(
          points={{-8.88178e-16,-120},{0,-120},{0,-32},{-2.66454e-15,-32}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(constantconstantFiringAngle.y, limiter.u) annotation (Line(
          points={{-19,-80},{0,-80},{0,-32},{-2.66454e-15,-32}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(v, filter.u) annotation (Line(
          points={{-120,0},{-100,0},{-100,-80},{-92,-80}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(filter.y, positiveThreshold.u) annotation (Line(
          points={{-69,-80},{-60,-80},{-60,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(filter.y, negativeThreshold.u) annotation (Line(
          points={{-69,-80},{-60,-80},{-60,-50},{-52,-50},{-52,-50},{60,-50},{
              60,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(realPassThrough.u, v) annotation (Line(
          points={{-92,-50},{-100,-50},{-100,0},{-120,0},{-120,8.88178e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(realPassThrough.y, positiveThreshold.u) annotation (Line(
          points={{-69,-50},{-60,-50},{-60,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(realPassThrough.y, negativeThreshold.u) annotation (Line(
          points={{-69,-50},{-56,-50},{-56,-50},{60,-50},{60,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
                    points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{
                -40,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Line(
                    points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Text(
                    extent={{-40,60},{40,0}},
                    lineColor={255,0,255},
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid,
                    textString="2*%m%")}),
        Documentation(revisions="<html>
</html>", info="<html>

<p>
General information about controllers is summarized in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control\">Control</a>.
</p>
</html>"));
    end Signal2mPulse;

    model VoltageBridge2Pulse "Control of 2 pulse bridge rectifier"
      import Modelica.Constants.pi;
      extends Modelica_Electrical_PowerConverters.Icons.Control;
      parameter Modelica.SIunits.Frequency f=50 "Frequency";
      parameter Boolean useConstantFiringAngle=true
        "Use constant firing angle instead of signal input";
      parameter Modelica.SIunits.Angle constantFiringAngle=0 "Firing angle"
        annotation (Dialog(enable=useConstantFiringAngle));
      parameter Modelica.SIunits.Angle firingAngleMax(
        min=0,
        max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      parameter Boolean useFilter=true "Enable use of filter"
        annotation (Dialog(tab="Filter"));
      parameter Modelica.SIunits.Frequency fCut=2*f
        "Cut off frequency of filter"
        annotation (Dialog(tab="Filter", enable=useFilter));
      parameter Modelica.SIunits.Voltage vStart=0
        "Start voltage of filter output"
        annotation (Dialog(tab="Filter", enable=useFilter));
      Modelica.Blocks.Interfaces.RealInput firingAngle if not
        useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-120})));
      Signal2mPulse twoPulse(
        final useConstantFiringAngle=useConstantFiringAngle,
        final f=f,
        final constantFiringAngle=constantFiringAngle,
        final firingAngleMax=firingAngleMax,
        final m=1,
        final useFilter=useFilter,
        final fCut=fCut,
        final vStart=fill(vStart, 1)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={0,0})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-80,0})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_p annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,110})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_n annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,110})));
    equation
      connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
          points={{-70,-2.22045e-15},{-60,-2.22045e-15},{-60,0},{-12,0},{-12,
              8.88178e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.p, ac_p) annotation (Line(
          points={{-80,10},{-80,60},{-100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, ac_n) annotation (Line(
          points={{-80,-10},{-80,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(firingAngle, twoPulse.firingAngle) annotation (Line(
          points={{8.88178e-16,-120},{8.88178e-16,-12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twoPulse.fire_n[1], fire_n) annotation (Line(
          points={{6,11},{6,80},{60,80},{60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.fire_p[1], fire_p) annotation (Line(
          points={{-6,11},{-6,80},{-60,80},{-60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
                    points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{
                -40,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Line(
                    points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Text(
                    extent={{-40,60},{40,0}},
                    lineColor={255,0,255},
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid,
                    textString="2")}),
        Documentation(revisions="<html>
</html>", info="<html>

<p>
General information about controllers is summarized in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control\">Control</a>.
</p>

<p>
This model provides two firing signal for Graetz bridge thyristor and half bridge rectifiers. The boolean
signal <code>fire_p</code> is assigned to the thyristors connected with the positive DC output pin.  
The boolean
signal <code>fire_n</code> is assigned to the thyristors connected with the negative DC output pin.  
</p>
</html>"));
    end VoltageBridge2Pulse;

    model VoltageBridge2mPulse "Control of 2*m pulse bridge rectifier"
      import Modelica.Constants.pi;
      extends Modelica_Electrical_PowerConverters.Icons.Control;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Frequency f=50 "Frequency";
      parameter Boolean useConstantFiringAngle=true
        "Use constant firing angle instead of signal input";
      parameter Modelica.SIunits.Angle constantFiringAngle=0 "Firing angle"
        annotation (Dialog(enable=useConstantFiringAngle));
      parameter Modelica.SIunits.Angle firingAngleMax(
        min=0,
        max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      parameter Boolean useFilter=true "Enable use of filter"
        annotation (Dialog(tab="Filter"));
      parameter Modelica.SIunits.Frequency fCut=2*f
        "Cut off frequency of filter"
        annotation (Dialog(tab="Filter", enable=useFilter));
      parameter Modelica.SIunits.Voltage vStart[m]=zeros(m)
        "Start voltage of filter output"
        annotation (Dialog(tab="Filter", enable=useFilter));
      Modelica.Blocks.Interfaces.RealInput firingAngle if not
        useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-120})));
      Signal2mPulse twomPulse(
        final useConstantFiringAngle=useConstantFiringAngle,
        final f=f,
        final constantFiringAngle=constantFiringAngle,
        final firingAngleMax=firingAngleMax,
        final m=m,
        useFilter=useFilter,
        final fCut=fCut,
        final vStart=vStart) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={0,10})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.MultiDelta delta(final m=m)
        "Delta connection" annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-80,10})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor(final
          m=m) "Voltage sensor" annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=270,
            origin={-44,10})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,110})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,110})));
    equation
      connect(ac, voltageSensor.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{-100,-4.44089e-16},{-44,-4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.plug_p, delta.plug_n) annotation (Line(
          points={{-44,-4.44089e-16},{-54,-4.44089e-16},{-54,0},{-62,0},{-62,-4.44089e-16},
              {-80,-4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(delta.plug_p, voltageSensor.plug_n) annotation (Line(
          points={{-80,20},{-44,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.v, twomPulse.v) annotation (Line(
          points={{-33,10},{-12,10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(firingAngle, twomPulse.firingAngle) annotation (Line(
          points={{1.11022e-15,-120},{1.11022e-15,-26},{0,-26},{0,-14},{
              4.44089e-16,-14},{4.44089e-16,-2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twomPulse.fire_n, fire_n) annotation (Line(
          points={{6,21},{6,80},{60,80},{60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twomPulse.fire_p, fire_p) annotation (Line(
          points={{-6,21},{-6,80},{-60,80},{-60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
                    points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{
                -40,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Line(
                    points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Text(
                    extent={{-40,60},{40,0}},
                    lineColor={255,0,255},
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid,
                    textString="2*%m%")}),
        Documentation(info="<html>

<p>
General information about controllers is summarized in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control\">Control</a>.
</p>

<p>
Half of the semiconductors of the <code>2*m</code> pulse bridge rectifier are connected with the positive DC output pin (firing signal <code>fire_p</code>). The other half of the simconductors is connected with the negative DC output pin (firing signal <code>fire_n</code>). Parameter <code>m</code> indicates the number of phases</code>.
</p>
</html>", revisions="<html>
</html>"));
    end VoltageBridge2mPulse;

    model VoltageCenterTap2mPulse "Control of 2*m pulse cetner tap rectifier"
      extends Modelica_Electrical_PowerConverters.Icons.Control;
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Frequency f=50 "Frequency";
      parameter Boolean useConstantFiringAngle=true
        "Use constant firing angle instead of signal input";
      parameter Modelica.SIunits.Angle constantFiringAngle=0 "Firing angle"
        annotation (Dialog(enable=useConstantFiringAngle));
      parameter Modelica.SIunits.Angle firingAngleMax(
        min=0,
        max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      parameter Boolean useFilter=true "Enable use of filter"
        annotation (Dialog(tab="Filter"));
      parameter Modelica.SIunits.Frequency fCut=2*f
        "Cut off frequency of filter"
        annotation (Dialog(tab="Filter", enable=useFilter));
      parameter Modelica.SIunits.Voltage vStart[m]=zeros(m)
        "Start voltage of filter output"
        annotation (Dialog(tab="Filter", enable=useFilter));
      Modelica.Blocks.Interfaces.RealInput firingAngle if not
        useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-120})));
      Signal2mPulse twomPulse(
        final useConstantFiringAngle=useConstantFiringAngle,
        final f=f,
        final constantFiringAngle=constantFiringAngle,
        final firingAngleMax=firingAngleMax,
        final m=m,
        final useFilter=useFilter,
        final fCut=2*f,
        final vStart=vStart) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={10,0})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,110})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,110})));
      Modelica.Electrical.MultiPhase.Basic.MultiDelta delta(final m=m)
        "Delta connection" annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-80,0})));
      Modelica.Electrical.MultiPhase.Sensors.PotentialSensor voltageSensor(
          final m=m) "Voltage sensor"
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Blocks.Math.Gain gain[m](final k=fill(-1, m))
        annotation (Placement(transformation(extent={{-28,-10},{-8,10}})));
    equation
      connect(firingAngle, twomPulse.firingAngle) annotation (Line(
          points={{8.88178e-16,-120},{8.88178e-16,-80},{10,-80},{10,-12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.plug_p, delta.plug_p) annotation (Line(
          points={{-60,6.66134e-16},{-66,6.66134e-16},{-66,0},{-68,0},{-68,
              4.44089e-16},{-70,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac, delta.plug_n) annotation (Line(
          points={{-100,4.44089e-16},{-90,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(gain.y, twomPulse.v) annotation (Line(
          points={{-7,0},{-2,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.phi, gain.u) annotation (Line(
          points={{-39,0},{-30,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twomPulse.fire_n, fire_n) annotation (Line(
          points={{16,11},{16,80},{60,80},{60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twomPulse.fire_p, fire_p) annotation (Line(
          points={{4,11},{4,80},{-60,80},{-60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
                    points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{
                -40,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Line(
                    points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},
                    color={255,0,255},
                    smooth=Smooth.None),Text(
                    extent={{-40,60},{40,0}},
                    lineColor={255,0,255},
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid,
                    textString="2*%m%")}),
        Documentation(revisions="<html>
</html>", info="<html>

<p>
General information about controllers is summarized in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control\">Control</a>.
</p>
</html>"));
    end VoltageCenterTap2mPulse;

    block Filter "PT1 + allpass filter"
      extends Modelica.Blocks.Interfaces.SISO;
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Frequency f=50 "Mains Frequency";
      parameter Modelica.SIunits.Frequency fCut=2*f "Cut off frequency";
      final parameter Integer na(final min=2) = 2 "Count of 1st order allpass";
      final parameter Modelica.SIunits.Frequency fa=f/tan(pi/na - atan(f/fCut)/
          (2*na));
      parameter Real yStart=0 "Start value of output"
        annotation (Dialog(enable=useFilter));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(
        final k=1,
        final T=1/(2*pi*fCut),
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=yStart)
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Blocks.Continuous.TransferFunction transferFunction[na](
        each final b={-1/(2*pi*fa),1},
        each final a={+1/(2*pi*fa),1},
        each initType=Modelica.Blocks.Types.Init.InitialOutput,
        each y_start=yStart)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      for j in 1:na - 1 loop
        connect(transferFunction[j].y, transferFunction[j + 1].u);
      end for;
      connect(u, firstOrder.u) annotation (Line(
          points={{-120,0},{-62,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(firstOrder.y, transferFunction[1].u) annotation (Line(
          points={{-39,0},{38,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(transferFunction[na].y, y) annotation (Line(
          points={{61,0},{110,0}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Documentation(info="<html>
<p>First order filter with cut-off frequency <code>fCut</code>. The phase shift of the filter is compensated by a series of two first order allpass filters tuned on suppy frequency <code>f</code>.</p>
</html>"),
        Icon(graphics={Polygon(
                    visible=true,
                    lineColor={192,192,192},
                    fillColor={192,192,192},
                    fillPattern=FillPattern.Solid,
                    points={{-80,90},{-88,68},{-72,68},{-80,90}}),Line(
                    visible=true,
                    points={{-80,78},{-80,-90}},
                    color={192,192,192}),Polygon(
                    visible=true,
                    lineColor={192,192,192},
                    fillColor={192,192,192},
                    fillPattern=FillPattern.Solid,
                    points={{90,-80},{68,-72},{68,-88},{90,-80}}),Line(
                    visible=true,
                    points={{-90,-80},{82,-80}},
                    color={192,192,192}),Rectangle(
                    visible=true,
                    lineColor={160,160,164},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Backward,
                    extent={{-80,-80},{22,8}}),Line(
                    visible=true,
                    origin={3.333,-8.667},
                    points={{-83.333,34.667},{24.667,34.667},{42.667,-71.333}},
                    color={0,0,127},
                    smooth=Smooth.Bezier)}));
    end Filter;
    annotation (Documentation(info="<html>
<p>
A generic controller with signal input and <code>2*m</code> firing signals is provided in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control.Signal2mPulse\">Signal2mPulse</a>,
where <code>m</code> is the arbitrary number of phases</code>. 
Additinal topology specific controllers with electrical connectors are also included.
</p>

<h4>Filters</h4>

<p>
Each controller is equipped with an optional 
<a href=\"modelica://Modelica_Electrical_PowerConverters.ACDC.Control.Filter\">filter</a> 
to filter the input voltages. By default the filter is enabled.  
</p>

<p>
Such filter is needed if the electrical grid includes a significant voltage drop across the grid impedance 
distoring the input voltage wave form of the rectifier. The filter included in the PowerConverters library is first order filter with additional compensation of the filter specific phase lag. 
However, it important to note that the transients of the filters may cause some initial effects which deteriorate after
certein periods.
</p>

<h4>Enable</h4>

<p>
The topology specific controllers allow enabling and disabling of the firing signals. The internal enabling signal of the controllers is either derived from the parameter <code>constantEnable</code>,
if <code>useConstantEnable = true</code>. For if <code>useConstantEnable = false</code> the internal 
enabling signal is taken from the optional signal input <code>enable</code>. 
</p>
</html>"));
  end Control;
  extends Modelica.Icons.Package;
  model DiodeBridge2Pulse "Two pulse Graetz diode rectifier bridge"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    Modelica.Electrical.Analog.Interfaces.PositivePin ac_p "Positive AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin ac_n "Negative AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC output"
      annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
    Modelica.SIunits.Current iAC=ac_p.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_p1(
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort,
      final Ron=RonDiode)
      "Diode connecting the positve AC input pin with postitive DC output"
      annotation (Placement(visible=true, transformation(
          origin={10,50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_p2(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort)
      "Diode connecting the negative AC input pin with postitive DC output"
      annotation (Placement(visible=true, transformation(
          origin={40,50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n1(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort)
      "Diode connecting the positve AC input pin with negative DC output"
      annotation (Placement(visible=true, transformation(
          origin={10,-50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n2(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort)
      "Diode connecting the negative AC input pin with negative DC output"
      annotation (Placement(visible=true, transformation(
          origin={40,-50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
  equation
    if not useHeatPort then
      LossPower = diode_p1.LossPower + diode_p2.LossPower + diode_n1.LossPower
         + diode_n2.LossPower;
    end if;
    connect(diode_p2.n, diode_p1.n) annotation (Line(
        points={{40,60},{10,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.p, diode_n2.p) annotation (Line(
        points={{10,-60},{40,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n2.n, diode_p2.p) annotation (Line(
        points={{40,-40},{40,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_p1.p, diode_n1.n) annotation (Line(
        points={{10,40},{10,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_p1.n, dc_p) annotation (Line(
        points={{10,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.p, dc_n) annotation (Line(
        points={{10,-60},{102,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.heatPort, heatPort) annotation (Line(
        points={{20,-50},{20,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_n2.heatPort, heatPort) annotation (Line(
        points={{50,-50},{50,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p1.heatPort, heatPort) annotation (Line(
        points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p2.heatPort, heatPort) annotation (Line(
        points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac_p, diode_p1.p) annotation (Line(
        points={{-100,60},{-60,60},{-60,20},{10,20},{10,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac_n, diode_p2.p) annotation (Line(
        points={{-100,-60},{-60,-60},{-60,-20},{40,-20},{40,40}},
        color={0,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a two pulse Graetz diode rectifier bridge. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2Pulse\">Examples.ACDC.ThyristorCenterTap2Pulse</a>.
</p>
</html>"));
  end DiodeBridge2Pulse;

  model ThyristorBridge2Pulse "Two pulse Graetz thyristor rectifier bridge"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p1=true
      "Boolean start value of variable thyristor_p1.off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_p2=true
      "Boolean start value of variable thyristor_p2.off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_n1=true
      "Boolean start value of variable thyristor_n1.off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_n2=true
      "Boolean start value of variable thyristor_n2.off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2;
    Modelica.Electrical.Analog.Interfaces.PositivePin ac_p "Positive AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin ac_n "Negative AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC output"
      annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Fire signal for positive potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Fire signal for negative potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
    Modelica.SIunits.Current iAC=ac_p.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p1(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_p1, fixed=true))
      "Thyristor connecting the positve AC input pin with postitive DC output"
      annotation (Placement(visible=true, transformation(
          origin={-20,50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p2(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_p2, fixed=true))
      "Thyristor connecting the negative AC input pin with postitive DC output"
      annotation (Placement(visible=true, transformation(
          origin={20,50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n1(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_n1, fixed=true))
      "Thyristor connecting the positve AC input with negative DC output"
      annotation (Placement(visible=true, transformation(
          origin={-20,-50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n2(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_n2, fixed=true))
      "Thyristor connecting the negative AC input with negative DC output"
      annotation (Placement(visible=true, transformation(
          origin={20,-50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
  equation
    if not useHeatPort then
      LossPower = thyristor_p1.LossPower + thyristor_p2.LossPower +
        thyristor_n1.LossPower + thyristor_n2.LossPower;
    end if;
    connect(thyristor_p2.n, thyristor_p1.n) annotation (Line(
        points={{20,60},{-20,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n1.p, thyristor_n2.p) annotation (Line(
        points={{-20,-60},{20,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n2.n, thyristor_p2.p) annotation (Line(
        points={{20,-40},{20,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p1.p, thyristor_n1.n) annotation (Line(
        points={{-20,40},{-20,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p1.n, dc_p) annotation (Line(
        points={{-20,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n1.p, dc_n) annotation (Line(
        points={{-20,-60},{102,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n1.heatPort, heatPort) annotation (Line(
        points={{-10,-50},{0,-50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_n2.heatPort, heatPort) annotation (Line(
        points={{10,-50},{0,-50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p1.heatPort, heatPort) annotation (Line(
        points={{-10,50},{0,50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p2.heatPort, heatPort) annotation (Line(
        points={{10,50},{0,50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac_p, thyristor_p1.p) annotation (Line(
        points={{-100,60},{-100,20},{-20,20},{-20,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac_n, thyristor_n2.n) annotation (Line(
        points={{-100,-60},{-100,-20},{20,-20},{20,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p1.fire) annotation (Line(
        points={{-60,-69},{-60,57},{-31,57}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_n2.fire) annotation (Line(
        points={{-60,-69},{-60,-30},{40,-30},{40,-43},{31,-43}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_p2.fire) annotation (Line(
        points={{60,-69},{60,57},{31,57}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_n1.fire) annotation (Line(
        points={{60,-69},{60,-66},{-40,-66},{-40,-43},{-31,-43}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,12},{0,28}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a two pulse Graetz thyristor rectifier bridge. The firing signal <code>fire_p</code> are connected
with thyristor <code>thyristor_p1</code> and <code>thyristor_n2</code>. 
The firing signal <code>fire_n</code> are connected
with thyristor <code>thyristor_p2</code> and <code>thyristor_n1</code>. See example 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2Pulse\">Examples.ACDC.ThyristorCenterTap2Pulse</a>.
</p>
</html>"));
  end ThyristorBridge2Pulse;

  model HalfControlledBridge2Pulse
    "Two pulse Graetz half controlled rectifier bridge"
    import Modelica.Constants.pi;
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p1=true
      "Boolean start value of variable thyristor_p1.off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_p2=true
      "Boolean start value of variable thyristor_p2.off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
      final T=293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2;
    Modelica.Electrical.Analog.Interfaces.PositivePin ac_p "Positive AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin ac_n "Negative AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC output"
      annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Fire signal for positive potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Fire signal for negative potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
    Modelica.SIunits.Current iAC=ac_p.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p1(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_p1, fixed=true)) annotation (Placement(visible=
            true, transformation(
          origin={-20,50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p2(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_p2, fixed=true)) annotation (Placement(visible=
            true, transformation(
          origin={20,50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n1(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort) "Diode connected to negative DC potential"
      annotation (Placement(visible=true, transformation(
          origin={-20,-50},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n2(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort) "Diode connected to negative DC potential"
      annotation (Placement(visible=true, transformation(
          origin={20,-50},
          extent={{-10,10},{10,-10}},
          rotation=90)));
  equation
    if not useHeatPort then
      LossPower = thyristor_p1.LossPower + thyristor_p2.LossPower + diode_n1.LossPower
         + diode_n2.LossPower;
    end if;
    connect(thyristor_p2.n, thyristor_p1.n) annotation (Line(
        points={{20,60},{-20,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.p, diode_n2.p) annotation (Line(
        points={{-20,-60},{20,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n2.n, thyristor_p2.p) annotation (Line(
        points={{20,-40},{20,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p1.p, diode_n1.n) annotation (Line(
        points={{-20,40},{-20,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p1.n, dc_p) annotation (Line(
        points={{-20,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.p, dc_n) annotation (Line(
        points={{-20,-60},{102,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n1.heatPort, heatPort) annotation (Line(
        points={{-10,-50},{0,-50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_n2.heatPort, heatPort) annotation (Line(
        points={{10,-50},{0,-50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p1.heatPort, heatPort) annotation (Line(
        points={{-10,50},{0,50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p2.heatPort, heatPort) annotation (Line(
        points={{10,50},{0,50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac_p, thyristor_p1.p) annotation (Line(
        points={{-100,60},{-100,20},{-20,20},{-20,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac_n, diode_n2.n) annotation (Line(
        points={{-100,-60},{-100,-20},{20,-20},{20,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p1.fire) annotation (Line(
        points={{-60,-69},{-60,57},{-31,57}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_p2.fire) annotation (Line(
        points={{60,-69},{60,57},{31,57}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-44,50},{36,2}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-44,26},{36,26}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{16,50},{16,2}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{16,26},{-24,50},{-24,2},{16,26}},
                color={0,0,255},
                smooth=Smooth.None),Rectangle(
                extent={{-44,2},{36,-54}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-44,-30},{36,-30}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{16,-6},{16,-54}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{16,-30},{-24,-6},{-24,-54},{16,-30}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-4,-18},{-4,-2}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a two pulse Graetz half controlled rectifier bridge. The firing signal <code>fire_p</code> is connected
with thyristor <code>thyristor_p1</code>. 
The firing signal <code>fire_n</code> is connected
with thyristor <code>thyristor_p2</code>. 
The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2Pulse\">Examples.ACDC.ThyristorCenterTap2Pulse</a>.
</p>
</html>"));
  end HalfControlledBridge2Pulse;

  model DiodeCenterTap2Pulse "Two pulse diode rectifier with center tap"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    Modelica.Electrical.Analog.Interfaces.PositivePin ac_p "Positive AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin ac_n "Negative AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
    Modelica.SIunits.Current iAC=ac_p.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_p(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort)
      "Diodes conducting positive pin AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={0,60},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort)
      "Diodes conducting negative pin AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={0,-60},
          extent={{-10,-10},{10,10}},
          rotation=0)));
  equation
    if not useHeatPort then
      LossPower = diode_p.LossPower + diode_n.LossPower;
    end if;
    connect(ac_p, diode_p.p) annotation (Line(
        points={{-100,60},{-10,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac_n, diode_n.p) annotation (Line(
        points={{-100,-60},{-10,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_p.n, dc_p) annotation (Line(
        points={{10,60},{100,60},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.n, dc_p) annotation (Line(
        points={{10,-60},{100,-60},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.heatPort, heatPort) annotation (Line(
        points={{6.66134e-16,-70},{6.66134e-16,-100},{0,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p.heatPort, heatPort) annotation (Line(
        points={{6.66134e-16,50},{6.66134e-16,40},{20,40},{20,-100},{
            4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (
      Icon(coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.1,
          grid={2,2}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Diagram(coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=false,
          initialScale=0.1,
          grid={2,2}), graphics),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a two pulse diode rectifier with center tap. In order to operate this rectifier a voltage with center tap is required. The center tap has to be connected with the negative pin of the load. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2Pulse\">Examples.ACDC.ThyristorCenterTap2Pulse</a>.
</p>
</html>"));
  end DiodeCenterTap2Pulse;

  model ThyristorCenterTap2Pulse
    "Two pulse thyristor rectifier with center tap"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p=true
      "Boolean start value of variable thyristor_p.off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_n=true
      "Boolean start value of variable thyristor_n.off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2;
    Modelica.Electrical.Analog.Interfaces.PositivePin ac_p "Positive AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin ac_n "Negative AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Postive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Fire signal for positive potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Fire signal for negative potential semiconductors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
    Modelica.SIunits.Current iAC=ac_p.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_p, fixed=true))
      "Thyristors conducting positive pin AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={0,60},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n(
      final Ron=RonThyristor,
      final Goff=GoffThyristor,
      final Vknee=VkneeThyristor,
      final useHeatPort=useHeatPort,
      final off(start=offStart_n, fixed=true))
      "Thyristors conducting negative pin AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={0,-60},
          extent={{-10,-10},{10,10}},
          rotation=0)));
  equation
    if not useHeatPort then
      LossPower = thyristor_p.LossPower + thyristor_n.LossPower;
    end if;
    connect(ac_p, thyristor_p.p) annotation (Line(
        points={{-100,60},{-10,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac_n, thyristor_n.p) annotation (Line(
        points={{-100,-60},{-10,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p.n, dc_p) annotation (Line(
        points={{10,60},{100,60},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n.n, dc_p) annotation (Line(
        points={{10,-60},{100,-60},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n.heatPort, heatPort) annotation (Line(
        points={{6.66134e-16,-70},{6.66134e-16,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p.heatPort, heatPort) annotation (Line(
        points={{6.66134e-16,50},{6.66134e-16,40},{20,40},{20,-100},{
            4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p.fire) annotation (Line(
        points={{-60,-69},{-60,80},{7,80},{7,71}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_n.fire) annotation (Line(
        points={{60,-69},{60,-40},{7,-40},{7,-49}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Icon(coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.1,
          grid={2,2}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,12},{0,28}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Diagram(coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=false,
          initialScale=0.1,
          grid={2,2}), graphics),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>This a two pulse thyristor rectifier with center tap. In order to operate this rectifier a voltage with center tap is required. The center tap has to be connected with the negative pin of the load. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2Pulse\">Examples.ACDC.ThyristorCenterTap2Pulse</a>.
</p>
</html>"));
  end ThyristorCenterTap2Pulse;

  model DiodeCenterTapmPulse "m pulse diode rectifier with center tap"
    import Modelica.Constants.pi;
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      "AC input"
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to positive DC potential" annotation (Placement(visible=
           true, transformation(
          origin={-10,0},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
      annotation (Placement(transformation(extent={{70,10},{90,-10}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{20,-100},{40,-80}})));
  equation
    assert(mod(m, 2) == 1,
      "DiodeCenterTapmPulse: only odd phase numbers are allowed");
    if not useHeatPort then
      LossPower = sum(diode.idealDiode.LossPower);
    end if;
    connect(diode.plug_n, star.plug_p) annotation (Line(
        points={{0,0},{22,0},{22,-6.66134e-16},{70,-6.66134e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star.pin_n, dc_p) annotation (Line(
        points={{90,-6.66134e-16},{100,-6.66134e-16},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{30,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode.heatPort, thermalCollector.port_a) annotation (Line(
        points={{-10,-10},{-10,-20},{30,-20},{30,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac, diode.plug_p) annotation (Line(
        points={{-100,0},{-20,0}},
        color={0,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>.
</p>

<p>
This is a m pulse diode rectifier with center tap. All voltage sources must have one interconnected plug (tap). This rectifiers works only with odd number of phases due the symmetry constrains of even phase numbers implemented in
<a href=\"modelica://Modelica.Electrical.MultiPhase.Functions.symmetricOrientation\">symmetricOrientation</a>. 
The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTapmPulse\">Examples.ACDC.ThyristorCenterTapmPulse</a>.
</p>
</html>"));
  end DiodeCenterTapmPulse;

  model ThyristorCenterTapmPulse "m pulse thyristor rectifier with center tap"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart[m]=fill(true, m)
      "Boolean start value of variable thyristor_p[:].off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable1m;

    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      "AC input"
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Fire signals for positive potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart, fixed=true))
      "Thyristors conducting AC potentials" annotation (Placement(visible=true,
          transformation(
          origin={-10,0},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
      annotation (Placement(transformation(extent={{70,10},{90,-10}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{20,-100},{40,-80}})));
  equation
    assert(mod(m, 2) == 1,
      "DiodeCenterTapmPulse: only odd phase numbers are allowed");
    if not useHeatPort then
      LossPower = sum(thyristor.idealThyristor.LossPower);
    end if;
    connect(thyristor.plug_n, star.plug_p) annotation (Line(
        points={{0,0},{70,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star.pin_n, dc_p) annotation (Line(
        points={{90,4.44089e-16},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{30,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor.heatPort, thermalCollector.port_a) annotation (Line(
        points={{-10,-10},{-10,-20},{30,-20},{30,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac, thyristor.plug_p) annotation (Line(
        points={{-100,0},{-20,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor.fire) annotation (Line(
        points={{-60,-69},{-60,20},{-3,20},{-3,11}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,12},{0,28}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a m pulse thyristor rectifier with center tap. All voltage sources must have one interconnected plug (tap). This rectifiers works only with odd number of phases due the symmetry constrains of even phase numbers implemented in
<a href=\"modelica://Modelica.Electrical.MultiPhase.Functions.symmetricOrientation\">symmetricOrientation</a>. 
See example
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTapmPulse\">Examples.ACDC.ThyristorCenterTapmPulse</a>.
</p>
</html>"));
  end ThyristorCenterTapmPulse;

  model DiodeBridge2mPulse "2*m pulse diode rectifier bridge"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC output"
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      "AC input"
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to positive DC potential" annotation (Placement(visible=
           true, transformation(
          origin={0,40},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to negative DC potential" annotation (Placement(visible=
           true, transformation(
          origin={0,-40},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
      annotation (Placement(transformation(extent={{70,70},{90,50}})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
      annotation (Placement(transformation(extent={{70,-50},{90,-70}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{10,-100},{30,-80}})));
  equation
    if not useHeatPort then
      LossPower = sum(diode_p.idealDiode.LossPower) + sum(diode_n.idealDiode.LossPower);
    else
      for k in 1:m loop
      end for;
    end if;
    connect(ac, diode_p.plug_p) annotation (Line(
        points={{-100,4.44089e-16},{-100,0},{0,0},{0,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_p.plug_p, diode_n.plug_n) annotation (Line(
        points={{0,30},{0,-30},{1.11022e-15,-30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_p.plug_n, star_p.plug_p) annotation (Line(
        points={{0,50},{0,60},{70,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.pin_n, dc_p) annotation (Line(
        points={{90,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.plug_p, star_n.plug_p) annotation (Line(
        points={{0,-50},{0,-60},{70,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_n) annotation (Line(
        points={{90,-60},{100,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
        points={{20,-80},{20,-40},{10,-40}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalCollector.port_b, heatPort) annotation (Line(
        points={{20,-100},{0,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{10,40},{20,40},{20,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a 2*m pulse diode rectifier bridge. In order to operate this rectifier a voltage source with center tap is required. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorBridge2mPulse\">Examples.ACDC.ThyristorBridge2mPulse</a>. It is important to note that for multi phase circuits with even phase numbers greater than three the 
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.MultiStarResistance\">MultiStarResistance</a> shall be used for grounding the voltage sources. 
</p>
</html>"));
  end DiodeBridge2mPulse;

  model ThyristorBridge2mPulse "2*m pulse thyristor rectifier bridge"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p[m]=fill(true, m)
      "Boolean start value of variable thyristor_p[:].off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_n[m]=fill(true, m)
      "Boolean start value of variable thyristor_n[:].off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2m;
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Fire signals for positive potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-118})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
      "Fire signasl for negative potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_p(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart_p, fixed=true))
      "Thyristors connected to positive DC potential" annotation (Placement(
          visible=true, transformation(
          origin={0,40},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_n(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart_n, fixed=true))
      "Thyristors connected to negative DC potential" annotation (Placement(
          visible=true, transformation(
          origin={0,-10},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
      annotation (Placement(transformation(extent={{70,70},{90,50}})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
      annotation (Placement(transformation(extent={{70,-20},{90,-40}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalConnector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{10,-100},{30,-80}})));
  equation
    if not useHeatPort then
      LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(thyristor_n.idealThyristor.LossPower);
    end if;
    connect(ac, thyristor_p.plug_p) annotation (Line(
        points={{-100,0},{-2.22045e-16,0},{-2.22045e-16,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p.plug_p, thyristor_n.plug_n) annotation (Line(
        points={{-2.22045e-16,30},{8.88178e-16,30},{8.88178e-16,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
        points={{1.33227e-15,50},{1.33227e-15,60},{70,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.pin_n, dc_p) annotation (Line(
        points={{90,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n.plug_p, star_n.plug_p) annotation (Line(
        points={{-2.22045e-16,-20},{-2.22045e-16,-30},{70,-30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_n) annotation (Line(
        points={{90,-30},{100,-30},{100,-60},{100,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalConnector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{20,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalConnector.port_a, thyristor_n.heatPort) annotation (Line(
        points={{20,-80},{20,-10},{10,-10}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p.heatPort, thermalConnector.port_a) annotation (Line(
        points={{10,40},{20,40},{20,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p.fire) annotation (Line(
        points={{-60,-69},{-60,47},{-11,47}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_n.fire) annotation (Line(
        points={{60,-69},{60,-40},{-20,-40},{-20,-3},{-11,-3}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,12},{0,28}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a 2*m pulse thyristor rectifier bridge. In order to operate this rectifier a voltage source with center tap is required. It is important to note that for multi phase circuits with phase even phase numbers greater than three the 
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.MultiStarResistance\">MultiStarResistance</a> shall be used for grounding the voltage sources. 
See example
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorBridge2mPulse\">Examples.ACDC.ThyristorBridge2mPulse</a>.
</p>
</html>"));
  end ThyristorBridge2mPulse;

  model HalfControlledBridge2mPulse
    "2*m pulse half controlled rectifier bridge"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p[m]=fill(true, m)
      "Boolean start value of variable thyristor_p[:].off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable1m;
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC output"
      annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      "AC input"
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Fire signals for positive potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
      annotation (Placement(transformation(extent={{70,70},{90,50}})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
      annotation (Placement(transformation(extent={{70,-50},{90,-70}})));
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_p(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart_p, fixed=true))
      "Thyristors connected to positive DC potential" annotation (Placement(
          visible=true, transformation(
          origin={0,40},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to negative DC potential" annotation (Placement(visible=
           true, transformation(
          origin={0,-40},
          extent={{-10,-10},{10,10}},
          rotation=90)));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{10,-100},{30,-80}})));
  equation
    if not useHeatPort then
      LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(diode_n.idealDiode.LossPower);
    end if;
    connect(ac, thyristor_p.plug_p) annotation (Line(
        points={{-100,4.44089e-16},{0,4.44089e-16},{0,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p.plug_p, diode_n.plug_n) annotation (Line(
        points={{0,30},{0,-30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
        points={{0,50},{0,60},{70,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.pin_n, dc_p) annotation (Line(
        points={{90,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.plug_p, star_n.plug_p) annotation (Line(
        points={{0,-50},{0,-50},{0,-50},{0,-60},{70,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_n) annotation (Line(
        points={{90,-60},{102,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{20,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
        points={{20,-80},{20,-40},{10,-40}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{10,40},{20,40},{20,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p.fire) annotation (Line(
        points={{-60,-69},{-60,47},{-11,47}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-46,52},{34,4}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-46,28},{34,28}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{14,52},{14,4}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{14,28},{-26,52},{-26,4},{14,28}},
                color={0,0,255},
                smooth=Smooth.None),Rectangle(
                extent={{-46,4},{34,-52}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-46,-28},{34,-28}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{14,-4},{14,-52}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{14,-28},{-26,-4},{-26,-52},{14,-28}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-6,-16},{-6,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>


<p>
This is a 2*m pulse half controlled rectifier bridge. In order to operate this rectifier a voltage source with center tap is required. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorBridge2mPulse\">Examples.ACDC.ThyristorBridge2mPulse</a>. It is important to note that for multi phase circuits with even phase numbers greater than three the 
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.MultiStarResistance\">MultiStarResistance</a> shall be used for grounding the voltage sources. 
</p>
</html>"));
  end HalfControlledBridge2mPulse;

  model DiodeCenterTap2mPulse "2*m pulse diode rectifier with center tap"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac_p(final m=m)
      "Positive potential AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.MultiPhase.Interfaces.NegativePlug ac_n(final m=m)
      "Negative potential AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac_p.pin[:].v - ac_n.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac_p.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to positive DC potential" annotation (Placement(visible=
           true, transformation(
          origin={-10,60},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      each final useHeatPort=useHeatPort)
      "Diodes connected to negative DC potential" annotation (Placement(visible=
           true, transformation(
          origin={-10,-60},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
      annotation (Placement(transformation(extent={{70,70},{90,50}})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
      annotation (Placement(transformation(extent={{72,-50},{92,-70}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{20,-100},{40,-80}})));
  equation
    if not useHeatPort then
      LossPower = sum(diode_p.idealDiode.LossPower) + sum(diode_n.idealDiode.LossPower);
    end if;
    connect(diode_p.plug_n, star_p.plug_p) annotation (Line(
        points={{0,60},{70,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.pin_n, dc_p) annotation (Line(
        points={{90,60},{100,60},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{30,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
        points={{30,-80},{-10,-80},{-10,-70}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{-10,50},{-10,40},{30,40},{30,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac_p, diode_p.plug_p) annotation (Line(
        points={{-100,60},{-20,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_p) annotation (Line(
        points={{92,-60},{100,-60},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.plug_p, ac_n) annotation (Line(
        points={{-20,-60},{-100,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(diode_n.plug_n, star_n.plug_p) annotation (Line(
        points={{4.44089e-16,-60},{72,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a 2*m pulse diode rectifier with center tap. In order to operate this rectifier a voltage source with center tap is required. The center tap has to be connected with the negative pin of the load. The circuit topology is the same as in 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2mPulse\">Examples.ACDC.ThyristorCenterTap2mPulse</a>.
</p>
</html>"));
  end DiodeCenterTap2mPulse;

  model ThyristorCenterTap2mPulse
    "2*m pulse thyristor rectifier with center tap"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    import Modelica.Constants.pi;
    parameter Integer m(final min=3) = 3 "Number of phases";
    parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
      "Closed thyristor resistance";
    parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
      "Opened thyristor conductance";
    parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
      "Thyristor forward threshold voltage";
    parameter Boolean offStart_p[m]=fill(true, m)
      "Boolean start value of variable thyristor_p[:].off"
      annotation (choices(checkBox=true));
    parameter Boolean offStart_n[m]=fill(true, m)
      "Boolean start value of variable thyristor_n[:].off"
      annotation (choices(checkBox=true));
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2m;
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac_p(final m=m)
      "Positive potential AC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.MultiPhase.Interfaces.NegativePlug ac_n(final m=m)
      "Negative potential AC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Fire signals for positive potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
      "Fire signasl for negative potential semiconductors" annotation (
        Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac_p.pin[:].v - ac_n.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac_p.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_p(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart_p, fixed=true))
      "Thyristors conducting positive plug AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={-10,60},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_n(
      final m=m,
      final Ron=fill(RonThyristor, m),
      final Goff=fill(GoffThyristor, m),
      final Vknee=fill(VkneeThyristor, m),
      each final useHeatPort=useHeatPort,
      final off(start=offStart_n, fixed=true))
      "Thyristors conducting negative plug AC potentials" annotation (Placement(
          visible=true, transformation(
          origin={-10,-60},
          extent={{10,10},{-10,-10}},
          rotation=180)));
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
      annotation (Placement(transformation(extent={{70,70},{90,50}})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
      annotation (Placement(transformation(extent={{72,-50},{92,-70}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{20,-100},{40,-80}})));
  equation
    if not useHeatPort then
      LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(thyristor_n.idealThyristor.LossPower);
    end if;
    connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
        points={{0,60},{70,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.pin_n, dc_p) annotation (Line(
        points={{90,60},{100,60},{100,4.44089e-16}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{30,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalCollector.port_a, thyristor_n.heatPort) annotation (Line(
        points={{30,-80},{-10,-80},{-10,-70}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thyristor_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{-10,50},{-10,40},{30,40},{30,-80}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(ac_p, thyristor_p.plug_p) annotation (Line(
        points={{-100,60},{-20,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_p) annotation (Line(
        points={{92,-60},{100,-60},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n.plug_p, ac_n) annotation (Line(
        points={{-20,-60},{-100,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(thyristor_n.plug_n, star_n.plug_p) annotation (Line(
        points={{4.44089e-16,-60},{72,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, thyristor_p.fire) annotation (Line(
        points={{-60,-69},{-60,80},{-3,80},{-3,71}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, thyristor_n.fire) annotation (Line(
        points={{60,-69},{60,-40},{-3,-40},{-3,-49}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Rectangle(
                extent={{-40,24},{40,-24}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-40,0},{40,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,24},{20,-24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{20,0},{-20,24},{-20,-24},{20,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,12},{0,28}},
                color={0,0,255},
                smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>

<p>
This is a 2*m pulse thyristor rectifier with center tap. In order to operate this rectifier a voltage source with center tap is required. The center tap has to be connected with the negative pin of the load. See example 
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.ACDC.ThyristorCenterTap2mPulse\">Examples.ACDC.ThyristorCenterTap2mPulse</a>.
</p>
</html>"));
  end ThyristorCenterTap2mPulse;
  annotation (Documentation(info="<html>
<p>
General information about AC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.ACDCConcept\">AC/DC converter concept</a>
</p>
</html>"));
end ACDC;


package DCAC "DC to AC converters"
  extends Modelica.Icons.Package;
  model SinglePhase2Level "Single phase DC to AC converter"
    extends Modelica.Blocks.Icons.Block;
    parameter Modelica.SIunits.Resistance RonTransistor=1e-05
      "Transistor closed resistance";
    parameter Modelica.SIunits.Conductance GoffTransistor=1e-05
      "Transistor opened conductance";
    parameter Modelica.SIunits.Voltage VkneeTransistor=0
      "Transistor threshold voltage";
    parameter Modelica.SIunits.Resistance RonDiode=1e-05
      "Diode closed resistance";
    parameter Modelica.SIunits.Conductance GoffDiode=1e-05
      "Diode opened conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode=0 "Diode threshold voltage";
    // parameter Boolean useEnable "Enables enable signal connector";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2;
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC input"
      annotation (Placement(transformation(extent={{-110,110},{-90,90}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC input"
      annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin ac "AC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Firing signals of positive potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Firing signals of negative potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC=ac.v "AC voltages";
    Modelica.SIunits.Current iAC=ac.i "AC currents";
    Modelica.Electrical.Analog.Ideal.IdealGTOThyristor transistor_p(
      final Ron=RonTransistor,
      final Goff=GoffTransistor,
      final Vknee=VkneeTransistor,
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={30,20})));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_p(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={70,20})));
    Modelica.Electrical.Analog.Ideal.IdealGTOThyristor transistor_n(
      final Ron=RonTransistor,
      final Goff=GoffTransistor,
      final Vknee=VkneeTransistor,
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={30,-20})));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode_n(
      final Ron=RonDiode,
      final Goff=GoffDiode,
      final Vknee=VkneeDiode,
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={70,-20})));
  equation
    if not useHeatPort then
      LossPower = transistor_p.LossPower + diode_n.LossPower + transistor_n.LossPower
         + diode_n.LossPower;
    end if;
    connect(transistor_p.p, dc_p) annotation (Line(
        points={{30,30},{50,30},{50,60},{-100,60},{-100,100}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.n, dc_n) annotation (Line(
        points={{30,-30},{50,-30},{50,-60},{-100,-60},{-100,-100}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.p, diode_p.n) annotation (Line(
        points={{30,30},{70,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.n, diode_p.p) annotation (Line(
        points={{30,10},{70,10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.p, diode_n.n) annotation (Line(
        points={{30,-10},{70,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.n, diode_n.p) annotation (Line(
        points={{30,-30},{70,-30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.n, transistor_n.p) annotation (Line(
        points={{30,10},{50,10},{50,-10},{30,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(ac, transistor_p.n) annotation (Line(
        points={{100,0},{50,0},{50,10},{30,10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.heatPort, heatPort) annotation (Line(
        points={{40,20},{40,0},{4.44089e-16,0},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(transistor_n.heatPort, heatPort) annotation (Line(
        points={{40,-20},{40,-40},{4.44089e-16,-40},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p.heatPort, heatPort) annotation (Line(
        points={{80,20},{80,-40},{4.44089e-16,-40},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_n.heatPort, heatPort) annotation (Line(
        points={{80,-20},{80,-40},{4.44089e-16,-40},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(andCondition_p.y, transistor_p.fire) annotation (Line(
        points={{-60,-69},{-60,13},{19,13}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, transistor_n.fire) annotation (Line(
        points={{60,-69},{60,-50},{10,-50},{10,-27},{19,-27}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Line(
                points={{-100,-100},{100,100}},
                color={0,0,127},
                smooth=Smooth.None),Rectangle(
                extent={{-40,40},{40,-40}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-20,20},{-20,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-28,20},{-28,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-40,0},{-28,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-20,4},{0,24},{0,40}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-20,-4},{0,-24},{0,-40}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-4,-20},{-10,-8},{-16,-14},{-4,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,-24},{10,-24},{10,24},{0,24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,8},{20,8}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{10,8},{0,-8},{20,-8},{10,8}},
                color={0,0,255},
                smooth=Smooth.None),Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC")}),
      Documentation(info="<html>
<p>
This is a single phase two level inverter. The boolean signals <code>fire_p</code> and <code>fire_n</code> shall not be <code>true</code> at the same time to avoid DC bus short circuits. The inverter consists of two transistors and two anti parallel free wheeling diodes.
</p>

<p>
An example of a single phase inverter with PWM voltage control is included in
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.DCAC.SinglePhaseTwoLevel\">Examples.DCAC.SinglePhaseTwoLevel</a>.
</p>
</html>"));
  end SinglePhase2Level;

  model MultiPhase2Level "Multi phase DC to AC converter"
    extends Modelica.Blocks.Icons.Block;
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2m;
    parameter Modelica.SIunits.Resistance RonTransistor=1e-05
      "Transistor closed resistance";
    parameter Modelica.SIunits.Conductance GoffTransistor=1e-05
      "Transistor opened conductance";
    parameter Modelica.SIunits.Voltage VkneeTransistor=0
      "Transistor threshold voltage";
    parameter Modelica.SIunits.Resistance RonDiode=1e-05
      "Diode closed resistance";
    parameter Modelica.SIunits.Conductance GoffDiode=1e-05
      "Diode opened conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode=0 "Diode threshold voltage";
    // parameter Boolean useEnable "Enables enable signal connector";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);

    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p "Positive DC input"
      annotation (Placement(transformation(extent={{-110,110},{-90,90}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n "Negative DC input"
      annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
    Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
      "AC output"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Firing signals of positive potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
      "Firing signals of negative potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
    Modelica.SIunits.Current iDC=dc_p.i "DC current";
    Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
    Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
    Modelica.Electrical.MultiPhase.Ideal.IdealGTOThyristor transistor_p(
      final m=m,
      final Ron=fill(RonTransistor, m),
      final Goff=fill(GoffTransistor, m),
      final Vknee=fill(VkneeTransistor, m),
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={30,20})));
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={70,20})));
    Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m) annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={50,50})));
    Modelica.Electrical.MultiPhase.Ideal.IdealGTOThyristor transistor_n(
      final m=m,
      final Ron=fill(RonTransistor, m),
      final Goff=fill(GoffTransistor, m),
      final Vknee=fill(VkneeTransistor, m),
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={30,-20})));
    Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
      final m=m,
      final Ron=fill(RonDiode, m),
      final Goff=fill(GoffDiode, m),
      final Vknee=fill(VkneeDiode, m),
      final useHeatPort=useHeatPort) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={70,-20})));
    Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m) annotation (
        Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={50,-50})));
    Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(
        final m=m) if useHeatPort
      annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  equation
    if not useHeatPort then
      LossPower = sum(transistor_p.idealGTOThyristor.LossPower) + sum(diode_n.idealDiode.LossPower)
         + sum(transistor_n.idealGTOThyristor.LossPower) + sum(diode_n.idealDiode.LossPower);
    end if;
    connect(transistor_p.plug_p, star_p.plug_p) annotation (Line(
        points={{30,30},{50,30},{50,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_p.plug_p, diode_p.plug_n) annotation (Line(
        points={{50,40},{50,30},{70,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.plug_n, star_n.plug_p) annotation (Line(
        points={{30,-30},{50,-30},{50,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.plug_p, diode_n.plug_p) annotation (Line(
        points={{50,-40},{50,-30},{70,-30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.plug_n, diode_p.plug_p) annotation (Line(
        points={{30,10},{70,10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.plug_p, diode_n.plug_n) annotation (Line(
        points={{30,-10},{70,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(star_n.pin_n, dc_n) annotation (Line(
        points={{50,-60},{50,-66},{-100,-66},{-100,-100}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_p, star_p.pin_n) annotation (Line(
        points={{-100,100},{-100,70},{50,70},{50,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_p.plug_n, ac) annotation (Line(
        points={{30,10},{50,10},{50,0},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor_n.plug_p, ac) annotation (Line(
        points={{30,-10},{50,-10},{50,0},{100,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(heatPort, thermalCollector.port_b) annotation (Line(
        points={{4.44089e-16,-100},{0,-100},{0,-60},{6.66134e-16,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(thermalCollector.port_a, transistor_n.heatPort) annotation (Line(
        points={{6.66134e-16,-40},{6.66134e-16,-36},{40,-36},{40,-20}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_n.heatPort, thermalCollector.port_a) annotation (Line(
        points={{80,-20},{80,-36},{6.66134e-16,-36},{6.66134e-16,-40}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(transistor_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{40,20},{40,4},{6.66134e-16,4},{6.66134e-16,-40}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
        points={{80,20},{80,4},{6.66134e-16,4},{6.66134e-16,-40}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(andCondition_p.y, transistor_p.fire) annotation (Line(
        points={{-60,-69},{-60,13},{19,13}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, transistor_n.fire) annotation (Line(
        points={{60,-69},{60,-64},{16,-64},{16,-27},{19,-27}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={Line(
                points={{-100,-100},{100,100}},
                color={0,0,127},
                smooth=Smooth.None),Rectangle(
                extent={{-40,40},{40,-40}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Line(
                points={{-20,20},{-20,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-28,20},{-28,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-40,0},{-28,0}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-20,4},{0,24},{0,40}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-20,-4},{0,-24},{0,-40}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{-4,-20},{-10,-8},{-16,-14},{-4,-20}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,-24},{10,-24},{10,24},{0,24}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{0,8},{20,8}},
                color={0,0,255},
                smooth=Smooth.None),Line(
                points={{10,8},{0,-8},{20,-8},{10,8}},
                color={0,0,255},
                smooth=Smooth.None),Text(
                extent={{-100,70},{0,50}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="DC"),Text(
                extent={{0,-50},{100,-70}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                textString="AC")}),
      Documentation(info="<html>
<p>
This is a multi phase two level inverter. The boolean signals <code>fire_p[k]</code> and <code>fire_n[k]</code> for any phase <code>k</code> shall not be <code>true</code> at the same time to avoid DC bus short circuits. The inverter consists of <code>2*m</code> transistors and two anti parallel free wheeling diodes, respectively, where <code>m</code> is the number of phases.
</p>
</html>"));
  end MultiPhase2Level;
  annotation (Documentation(info="<html>
<p>
General information about DC/AC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.DCACConcept\">DC/AC converter concept</a> 
</p>
</html>"));
end DCAC;


package DCDC "DC to DC converters"
  extends Modelica.Icons.Package;
  package Control "Control components for DC to DC converters"
    extends Modelica.Icons.Package;
    model SignalPWM
      "Generates a pulse width modulated (PWM) boolean fire signal"
      extends Modelica_Electrical_PowerConverters.Icons.Control;
      parameter Boolean useConstantDutyCycle=true "Enables constant duty cycle";
      parameter Real constantDutyCycle=0 "Constant duty cycle"
        annotation (Dialog(enable=useConstantDutyCycle));
      parameter Modelica.SIunits.Frequency f=1000 "Switching frequency";
      parameter Modelica.SIunits.Time startTime=0 "Start time";
      Modelica.Blocks.Interfaces.RealInput dutyCycle if not
        useConstantDutyCycle "Duty cycle"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.BooleanOutput fire "Firing PWM signal"
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-60,110})));
      Modelica.Blocks.Interfaces.BooleanOutput notFire "Firing PWM signal"
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,110})));
      Modelica.Blocks.Sources.Constant const(final k=constantDutyCycle) if
        useConstantDutyCycle
        annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1, uMin=0)
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Blocks.Logical.Less greaterEqual annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={22,-8})));
      Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(final startTime=
            startTime, final samplePeriod=1/f)
        annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
      Modelica.Blocks.Sources.SawTooth sawtooth(
        final period=1/f,
        final amplitude=1,
        final nperiod=-1,
        final offset=0,
        final startTime=startTime) annotation (Placement(visible=true,
            transformation(
            origin={-50,-50},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Blocks.Logical.Not inverse annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={52,20})));
    equation
      connect(const.y, limiter.u) annotation (Line(
          points={{-79,-50},{-70,-50},{-70,0},{-62,0}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(dutyCycle, limiter.u) annotation (Line(
          points={{-120,1.11022e-15},{-96,1.11022e-15},{-96,0},{-70,0},{-70,
              6.66134e-16},{-62,6.66134e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, zeroOrderHold.u) annotation (Line(
          points={{-39,6.66134e-16},{-38,6.66134e-16},{-38,4.44089e-16},{-36,
              4.44089e-16},{-36,8.88178e-16},{-32,8.88178e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(zeroOrderHold.y, greaterEqual.u2) annotation (Line(
          points={{-9,6.66134e-16},{-4,6.66134e-16},{-4,-4.44089e-16},{0,-4.44089e-16},
              {0,4.44089e-16},{10,4.44089e-16}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(sawtooth.y, greaterEqual.u1) annotation (Line(
          points={{-39,-50},{0,-50},{0,-8},{10,-8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterEqual.y, inverse.u) annotation (Line(
          points={{33,-8},{52,-8},{52,8}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(greaterEqual.y, fire) annotation (Line(
          points={{33,-8},{36,-8},{36,80},{-60,80},{-60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(inverse.y, notFire) annotation (Line(
          points={{52,31},{52,80},{60,80},{60,110}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
                    points={{-100,0},{-98,0},{12,0}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{-60,-22},{-60,-64},{44,-64},{44,-36}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{-80,-16},{-80,-20},{-40,20},{-40,-20},{-36,-16}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{-62,0},{-76,4},{-76,-4},{-62,0}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{44,-36},{44,-36},{40,-50},{44,-50},{48,-50},{44,-36}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{20,-20},{22,-20},{24,-20},{24,20},{44,20},{44,-20},
                {64,-20},{64,-16}},
                    color={255,0,255},
                    smooth=Smooth.None),Line(
                    points={{-40,-16},{-40,-20},{0,20},{0,-20},{4,-16}},
                    color={0,0,255},
                    smooth=Smooth.None),Line(
                    points={{60,-20},{62,-20},{64,-20},{64,20},{84,20},{84,-20},
                {84,-20},{88,-20}},
                    color={255,0,255},
                    smooth=Smooth.None)}),
        Documentation(info="<html>
<p>
This controller can be used both for DC/DC and AC/DC converters. 
The signal input of the PWM controller is the duty cycle; the duty cycle is the ratio of the on time 
to the switching period. The output firing signal is strictly determined by the actual duty cycle, indicated as <code>d</code> in Fig.&nbsp;1. 
</p>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig. 1:</b> Firing (<code>fire</code>) and inverse firing (<code>notFire</code>) signal of PWM control; <code>d</code> = duty cycle; <code>f</code> = switching period </caption>
  <tr>
    <td>
      <img src=\"modelica://Modelica_Electrical_PowerConverters/Resources/Images/dutyCycle.png\">
    </td>
  </tr>
</table>

<p>
The firing signal is generated by comparing the sampled duty cycle input with a periodic saw tooth signal [<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.References\">Williams2006</a>]. 
</p>
</html>"));
    end SignalPWM;
    annotation (Documentation(info="<html>
<p>
Currently there is only one PWM method provided in this library.
</p>
</html>"));
  end Control;

  model ChopperStepDown "Step down chopper"
    import Modelica.Constants.pi;
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    parameter Modelica.SIunits.Resistance RonTransistor=1e-05
      "Transistor closed resistance";
    parameter Modelica.SIunits.Conductance GoffTransistor=1e-05
      "Transistor opened conductance";
    parameter Modelica.SIunits.Voltage VkneeTransistor=0
      "Transistor threshold voltage";
    parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
      "Closed diode resistance";
    parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
      "Opened diode conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
      "Diode forward threshold voltage";
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable1;
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p1 "Positive DC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n1 "Negative DC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p2 "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n2
      "Negative DC output"
      annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    Modelica.SIunits.Voltage vDCi=dc_p1.v - dc_n1.v "DC voltage side 1";
    Modelica.SIunits.Current iDCi=dc_p1.i "DC current side 1";
    Modelica.SIunits.Voltage vDCo=dc_p2.v - dc_n2.v "DC voltages side 2";
    Modelica.SIunits.Current iDCo=dc_p2.i "DC current side 2";
    Modelica.Electrical.Analog.Ideal.IdealGTOThyristor transistor(
      useHeatPort=useHeatPort,
      Ron=RonTransistor,
      Goff=GoffTransistor,
      Vknee=VkneeTransistor) "Switching transistor" annotation (Placement(
          visible=true, transformation(
          origin={0,60},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Electrical.Analog.Ideal.IdealDiode diode(
      Ron=RonDiode,
      Goff=GoffDiode,
      Vknee=VkneeDiode,
      useHeatPort=useHeatPort) "Free wheeling diode" annotation (Placement(
          visible=true, transformation(
          origin={40,0},
          extent={{-10,10},{10,-10}},
          rotation=90)));
  equation
    if not useHeatPort then
      LossPower = diode.LossPower + transistor.LossPower;
    end if;
    connect(diode.n, transistor.n) annotation (Line(
        points={{40,10},{40,60},{10,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor.n, dc_p2) annotation (Line(
        points={{10,60},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(transistor.heatPort, heatPort) annotation (Line(
        points={{6.66134e-16,50},{0,50},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(diode.heatPort, heatPort) annotation (Line(
        points={{30,2.22045e-16},{30,0},{0,0},{0,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(dc_p1, transistor.p) annotation (Line(
        points={{-100,60},{-10,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_n1, diode.p) annotation (Line(
        points={{-100,-60},{40,-60},{40,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_n1, dc_n2) annotation (Line(
        points={{-100,-60},{102,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, transistor.fire) annotation (Line(
        points={{-60,-69},{-60,80},{7,80},{7,71}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),     graphics),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}), graphics={
          Text(
            extent={{-100,70},{0,50}},
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="DC in"),
          Text(
            extent={{0,-50},{100,-70}},
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="DC out"),
          Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
          Rectangle(
            extent={{-40,40},{40,-40}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-20,20},{-20,-20}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{-28,20},{-28,-20}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{-40,0},{-28,0}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{-20,4},{0,24},{0,40}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{-20,-4},{0,-24},{0,-40}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{-4,-20},{-10,-8},{-16,-14},{-4,-20}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{0,-24},{10,-24},{10,24},{0,24}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{0,8},{20,8}},
            color={0,0,255},
            smooth=Smooth.None),
          Line(
            points={{10,8},{0,-8},{20,-8},{10,8}},
            color={0,0,255},
            smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
This is a conventional step down chopper model. It consists of a transistor and free wheeling diode. 
</p>
</html>"));
  end ChopperStepDown;

  model HBridge "H bridge (four quadrant converter)"
    extends Modelica_Electrical_PowerConverters.Icons.Converter;
    extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T=
          293.15);
    parameter Modelica.SIunits.Resistance RonTransistor=1e-05
      "Transistor closed resistance";
    parameter Modelica.SIunits.Conductance GoffTransistor=1e-05
      "Transistor opened conductance";
    parameter Modelica.SIunits.Voltage VkneeTransistor=0
      "Transistor threshold voltage";
    parameter Modelica.SIunits.Resistance RonDiode=1e-05
      "Diode closed resistance";
    parameter Modelica.SIunits.Conductance GoffDiode=1e-05
      "Diode opened conductance";
    parameter Modelica.SIunits.Voltage VkneeDiode=0 "Diode threshold voltage";
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable2;

    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p1 "Positive DC input"
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n1 "Negative DC input"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Electrical.Analog.Interfaces.NegativePin dc_n2
      "Negative DC output"
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Electrical.Analog.Interfaces.PositivePin dc_p2 "Postive DC output"
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    DCAC.SinglePhase2Level inverter_p(
      final RonTransistor=RonTransistor,
      final GoffTransistor=GoffTransistor,
      final VkneeTransistor=VkneeTransistor,
      final RonDiode=RonDiode,
      final GoffDiode=GoffDiode,
      final VkneeDiode=VkneeDiode,
      final useHeatPort=useHeatPort)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    DCAC.SinglePhase2Level inverter_n(
      final RonTransistor=RonTransistor,
      final GoffTransistor=GoffTransistor,
      final VkneeTransistor=VkneeTransistor,
      final RonDiode=RonDiode,
      final GoffDiode=GoffDiode,
      final VkneeDiode=VkneeDiode,
      final useHeatPort=useHeatPort)
      annotation (Placement(transformation(extent={{-58,-40},{-38,-20}})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Firing signals of positive potential leg" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Firing signals of negative potential leg" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
  equation
    if not useHeatPort then
      LossPower = inverter_p.LossPower + inverter_n.LossPower;
    end if;
    connect(inverter_n.heatPort, heatPort) annotation (Line(
        points={{-48,-40},{-48,-70},{-10,-70},{-10,-100},{4.44089e-16,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(inverter_p.heatPort, heatPort) annotation (Line(
        points={{-10,20},{-10,-100},{0,-100}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(dc_p1, inverter_p.dc_p) annotation (Line(
        points={{-100,60},{-70,60},{-70,40},{-20,40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_p1, inverter_n.dc_p) annotation (Line(
        points={{-100,60},{-70,60},{-70,-20},{-58,-20}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_n1, inverter_n.dc_n) annotation (Line(
        points={{-100,-60},{-80,-60},{-80,-40},{-58,-40}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(dc_n1, inverter_p.dc_n) annotation (Line(
        points={{-100,-60},{-80,-60},{-80,20},{-20,20}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(inverter_p.ac, dc_p2) annotation (Line(
        points={{4.44089e-16,30},{100,30},{100,60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(inverter_n.ac, dc_n2) annotation (Line(
        points={{-38,-30},{100,-30},{100,-60}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, inverter_p.fire_n) annotation (Line(
        points={{60,-69},{60,-50},{-4,-50},{-4,18}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_n.y, inverter_n.fire_p) annotation (Line(
        points={{60,-69},{60,-50},{-54,-50},{-54,-42}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, inverter_n.fire_n) annotation (Line(
        points={{-60,-69},{-60,-60},{-42,-60},{-42,-42}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(andCondition_p.y, inverter_p.fire_p) annotation (Line(
        points={{-60,-69},{-60,-60},{-16,-60},{-16,18}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}}), graphics),
      Icon(graphics={
          Rectangle(
            extent={{-40,40},{40,-40}},
            lineColor={255,255,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{0,-50},{100,-70}},
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="DC out"),
          Text(
            extent={{-100,70},{0,50}},
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="DC in"),
          Line(
            points={{-20,30},{-20,-30}},
            color={0,0,127},
            smooth=Smooth.None),
          Line(
            points={{20,30},{20,-30}},
            color={0,0,127},
            smooth=Smooth.None),
          Line(
            points={{-20,0},{20,0}},
            color={0,0,127},
            smooth=Smooth.None)}),
      Documentation(info="<html>
<p>
The H bridge is a four quadrant DC/DC converter. It consists of two single phase DC/AC converters which are controlled differently; see Fig.&nbsp;1.</p>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig. 1:</b> H bridge</caption>
  <tr>
    <td>
      <img src=\"modelica://Modelica_Electrical_PowerConverters/Resources/Images/Hbridge.png\">
    </td>
  </tr>
</table>

<p>If the firing inputs <code>fire_p</code> and <code>fire_n</code> are inverse, the two legs are controlled symmetrically so that full positive or negative output voltage can be generated. See examples
<a href=\"modelica://Modelica_Electrical_PowerConverters.Examples.DCDC.HBridge\">DCDC.HBridge</a>.
</p>
</html>"));
  end HBridge;
  annotation (Documentation(info="<html>
<p>
General information about DC/DC converters can be found at the
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.DCDCConcept\">DC/DC converter concept</a>
</p>
</html>"));
end DCDC;


package Interfaces "Interfaces and partial models"
  extends Modelica.Icons.InterfacesPackage;
  model EnableLogic
    "Partial model providing enable parameter and optional enable input"
    parameter Boolean useConstantEnable=true
      "true = disabled boolean input, use constantEnable";
    parameter Boolean constantEnable=true "Constant enabling of firing signals"
      annotation (Dialog(enable=useConstantEnable));
    parameter Integer m(final min=1) = 3 "Number of phases";
    Modelica.Blocks.Sources.BooleanConstant enableConstantSource(final k=
          constantEnable) if useConstantEnable
      "Constant enable signal of fire and notFire" annotation (Placement(
          transformation(
          extent={{-10,10.5},{10,-10.5}},
          rotation=180,
          origin={69.5,0})));
    Modelica.Blocks.Interfaces.BooleanInput enable if not useConstantEnable
      "Enables fire and notFire" annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={100,-120})));
    Modelica.Blocks.Routing.BooleanReplicator booleanReplicator(final nout=m)
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={0,0})));
    Modelica.Blocks.Interfaces.BooleanOutput internalEnable[m]
      "m replicated enable signals" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-110,0})));
  equation
    connect(internalEnable, booleanReplicator.y) annotation (Line(
        points={{-110,4.44089e-16},{-11,4.44089e-16}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enable, booleanReplicator.u) annotation (Line(
        points={{100,-120},{100,-80},{40,-80},{40,-40},{40,8.88178e-16},{26,8.88178e-16},
            {12,8.88178e-16}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableConstantSource.y, booleanReplicator.u) annotation (Line(
        points={{58.5,1.77636e-15},{58.5,8.88178e-16},{12,8.88178e-16}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>This model provides an internal enable signal derived either from a parameter or an optional signal input. 
For <code>useConstantEnable = true</code> the internal signal <code>internalEnable</code> is equal to the parameter <code>constantEnable</code>. 
For <code>useConstantEnable = false</code> the internal signal 
<code>internalEnable</code> is equal to the external signal input <code>enable</code>.</p>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={255,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{-58,60},{58,-60}},
            lineColor={255,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="enable")}));
  end EnableLogic;

  partial model Enable
    "Partial model providing enable parameter and optional enable input"
    parameter Boolean useConstantEnable=true
      "true = disabled boolean input, use constantEnable"
      annotation (Dialog(tab="Enable"));
    parameter Boolean constantEnable=true "Constant enabling of firing signals"
      annotation (Dialog(tab="Enable", enable=useConstantEnable));
    parameter Integer m(final min=1) = 3 "Number of phases";
    EnableLogic enableLogic(
      final useConstantEnable=useConstantEnable,
      final constantEnable=constantEnable,
      final m=m) "Enabling logic"
      annotation (Placement(transformation(extent={{80,-90},{100,-70}})));
    Modelica.Blocks.Interfaces.BooleanInput enable if  not useConstantEnable
      "Enables fire and notFire" annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={100,-120})));
  equation
    connect(enableLogic.enable, enable) annotation (Line(
        points={{100,-92},{100,-120}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>
This partial model provides paramters and the conditional input signal for the enabling models:</p>
<p>
<ul>
<li><a href=\"modelica://Modelica_Electrical_PowerConverters.Interfaces.Enable1\">Enable1</a></li>
<li><a href=\"modelica://Modelica_Electrical_PowerConverters.Interfaces.Enable2\">Enable2</a></li>
<li><a href=\"modelica://Modelica_Electrical_PowerConverters.Interfaces.Enable1m\">Enable1m</a></li>
<li><a href=\"modelica://Modelica_Electrical_PowerConverters.Interfaces.Enable2m\">Enable2m</a></li>
</ul>
</p>
</html>"));
  end Enable;

  partial model Enable1
    "Partial model providing enable parameter and optional enable input for one firing signal"
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable(final m=1);
    Modelica.Blocks.Logical.And andCondition_p
      "And condition for positive firing signal" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=270,
          origin={-60,-80})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p
      "Firing signal of positive potential transistor" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));

  equation
    connect(andCondition_p.u1, fire_p) annotation (Line(
        points={{-60,-92},{-60,-120}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.enable, enable) annotation (Line(
        points={{100,-92},{100,-120}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.internalEnable[1], andCondition_p.u2) annotation (Line(
        points={{79,-80},{76,-80},{76,-96},{40,-96},{40,-76},{-40,-76},{-40,
            -100},{-52,-100},{-52,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>This partial model provides the enabling logic for one firing signal.</p>
</html>"));
  end Enable1;

  partial model Enable2
    "Partial model providing enable parameter and optional enable input for two firing signals"
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable1;
    Modelica.Blocks.Logical.And andCondition_n
      "And condition for negative firing signal" annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={60,-80})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n
      "Firing signal of negative potential transistor" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
  equation
    connect(fire_n, andCondition_n.u1) annotation (Line(
        points={{60,-120},{60,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.internalEnable[1], andCondition_n.u2) annotation (Line(
        points={{79,-80},{76,-80},{76,-96},{52,-96},{52,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>This partial model provides the enabling logic for two firing signal.</p>
</html>"));
  end Enable2;

  partial model Enable1m
    "Partial model providing enable parameter and optional enable input for m firing signals"
    parameter Boolean useConstantEnable=true
      "true = disabled boolean input, use constantEnable"
      annotation (Dialog(tab="Enable"));
    parameter Boolean constantEnable=true "Constant enabling of firing signals"
      annotation (Dialog(tab="Enable", enable=useConstantEnable));
    parameter Integer m(min=1) = 3 "Number of phases";
    Modelica.Blocks.Logical.And andCondition_p[m]
      "And condition for m positive firing signals" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=270,
          origin={-60,-80})));
    Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
      "Firing signals of positive potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-60,-120})));
    EnableLogic enableLogic(
      final useConstantEnable=useConstantEnable,
      final constantEnable=constantEnable,
      final m=m) "Enabling logic"
      annotation (Placement(transformation(extent={{80,-90},{100,-70}})));
    Modelica.Blocks.Interfaces.BooleanInput enable if  not useConstantEnable
      "Enables fire and notFire" annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={100,-120})));
  equation
    connect(andCondition_p.u1, fire_p) annotation (Line(
        points={{-60,-92},{-60,-120}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.enable, enable) annotation (Line(
        points={{100,-92},{100,-120}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.internalEnable, andCondition_p.u2) annotation (Line(
        points={{79,-80},{76,-80},{76,-96},{40,-96},{40,-76},{-40,-76},{-40,
            -100},{-52,-100},{-52,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>This partial model provides the enabling logic for <code>m</code> firing signal.</p>
</html>"));
  end Enable1m;

  partial model Enable2m
    "Partial model providing enable parameter and optional enable input for 2*m firing signals"
    extends Modelica_Electrical_PowerConverters.Interfaces.Enable1m;
    Modelica.Blocks.Logical.And andCondition_n[m]
      "And condition for m negative firing signals" annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={60,-80})));
    Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
      "Firing signals of negative potential transistors" annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
  equation
    connect(fire_n, andCondition_n.u1) annotation (Line(
        points={{60,-120},{60,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(enableLogic.internalEnable, andCondition_n.u2) annotation (Line(
        points={{79,-80},{76,-80},{76,-96},{52,-96},{52,-92}},
        color={255,0,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(info="<html>
<p>This partial model provides the enabling logic for <code>2*m</code> firing signal.</p>
</html>"));
  end Enable2m;
end Interfaces;


package Icons "Icons"
  extends Modelica.Icons.Package;
  partial model ExampleTemplate "Example template"
    annotation (Icon(graphics={Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={175,175,175},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Polygon(
                points={{-36,-60},{-36,60},{64,0},{-36,-60}},
                lineColor={175,175,175},
                smooth=Smooth.None,
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid),Rectangle(
                extent={{-4,46},{14,-44}},
                lineColor={255,255,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
  end ExampleTemplate;

  partial model Converter "Converter icon"
    annotation (Icon(graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-100,-100},{100,100}},
            color={0,0,127},
            smooth=Smooth.None),
          Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255})}));
  end Converter;

  partial block Control "Control icon"
    annotation (Icon(graphics={Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),Text(
                extent={{-150,-140},{150,-180}},
                textString="%name",
                lineColor={0,0,255})}));
  end Control;
end Icons;


annotation (
  Icon(coordinateSystem(
      extent={{-100,-100},{100,100}},
      preserveAspectRatio=true,
      initialScale=0.1,
      grid={2,2}), graphics),
  Diagram(coordinateSystem(
      extent={{-100,-100},{100,100}},
      preserveAspectRatio=true,
      initialScale=0.1,
      grid={2,2})),
  preferredView="info",
  version="1.2.0",
  versionBuild=0,
  versionDate="2014-04-06",
  uses(Modelica(version="3.2.2")),
  Documentation(info="<html>
<p>
Copyright &copy; 2013-2014, <a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.Contact\">Christian Kral</a> and
<a href=\"modelica://Modelica_Electrical_PowerConverters.UsersGuide.Contact\">Anton Haumer</a>
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>"));
end Modelica_Electrical_PowerConverters;
