within ;
package Modelica_Electrical_PowerConverters
  "Rectifiers, Inverters and DC/DC converters"
  extends Modelica.Icons.Package;
  package UsersGuide "User's Guide"
    extends Modelica.Icons.Information;
    class Concept "Power converter concept"
      extends Modelica.Icons.Information;
      annotation (Documentation(info="<html>


</html>"));
    end Concept;

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

<h5>Version 1.0.0, 2014-XX-XX</h5>
<ul>
<li>Initial version</li>
</ul>


</html>"));
    end ReleaseNotes;

    class References "References"
      extends Modelica.Icons.References;
      annotation (Documentation(info="<html>
<h4>References</h4>

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
</html>"));
    end References;
    annotation (Documentation(info="<html>
<p>
This is the library of power converters for single and multi phase electrical systems. 
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
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-40,70})));
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
              points={{-40,59},{-40,10}},
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
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
        end Thyristor1Pulse_R_Characteristic;
      end Thyristor1Pulse;

      package ThyristorCenterTap2Pulse "Examples of Power Electronics with M2C"
        extends Modelica.Icons.ExamplesPackage;
        model ThyristorCenterTap2Pulse_R
          "Two pulse thyristor rectifier with center tap and resistive load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
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
              Interval=0.0002));
        end ThyristorCenterTap2Pulse_R;

        model ThyristorCenterTap2Pulse_RL
          "Two pulse thyristor rectifier with center tap and R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
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
              Interval=0.0002));
        end ThyristorCenterTap2Pulse_RL;

        model ThyristorCenterTap2Pulse_RLV
          "Two pulse thyristor rectifier with center tap and R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
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
              Interval=0.0002));
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
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
              points={{-30,59},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(resistor.p, rectifier.dc_p) annotation (Line(
              points={{30,40},{-10,40},{-10,0},{-20,0}},
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
              Interval=0.0002));
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

      package ThyristorBridge2Pulse "Two pulse Graetz bridge"
        extends Modelica.Icons.ExamplesPackage;
        model ThyristorBridge2Pulse_R
          "Graetz thyristor bridge rectifier with resistive load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
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
              Interval=0.0002));
        end ThyristorBridge2Pulse_R;

        model ThyristorBridge2Pulse_RL
          "Graetz thyristor bridge rectifier with R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
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
              Interval=0.0002));
        end ThyristorBridge2Pulse_RL;

        model ThyristorBridge2Pulse_RLV
          "Graetz thyristor bridge rectifier with R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
              pulse2(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
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
              Interval=0.0002));
        end ThyristorBridge2Pulse_RLV;

        model ThyristorBridge2Pulse_RLV_Characteristic
          "Characteristic of Graetz thyristor bridge rectifier with R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2Pulse(
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
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
              points={{-30,59},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(resistor.p, rectifier.dc_p) annotation (Line(
              points={{30,40},{-10,40},{-10,6},{-20,6}},
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
              Interval=0.0002));
        end ThyristorBridge2Pulse_RLV_Characteristic;

        model ThyristorBridge2Pulse_DC_Drive
          "Graetz thyristor bridge feeding a DC drive"
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Voltage Vrms=dcpmData.VaNominal/(2/pi*sin(
              pi/2)*sqrt(2)) "RMS supply voltage";
          parameter Modelica.SIunits.Frequency f=50 "Frequency";
          parameter Modelica.SIunits.ApparentPower SMains=250E3
            "Mains short circuit apparent power";
          parameter Real lamdaMains=0.1 "Mains short circuit power factor";
          final parameter Modelica.SIunits.Impedance ZMains=Vrms^2/SMains
            "Mains short circuit impedance";
          final parameter Modelica.SIunits.Resistance RMains=ZMains*lamdaMains
            "Mains resistance" annotation (Evaluate=true);
          final parameter Modelica.SIunits.Inductance LMains=ZMains*sqrt(1 -
              lamdaMains^2)/(2*pi*f) "Mains inductance"
            annotation (Evaluate=true);
          parameter Modelica.SIunits.Inductance Ld=10*dcpmData.La
            "Smoothing inductance" annotation (Evaluate=true);
          final parameter Modelica.SIunits.Torque tauNominal=dcpmData.ViNominal
              *dcpmData.IaNominal/dcpmData.wNominal "Nominal torque";
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
            pulse2(f=f, useConstantFiringAngle=false) annotation (Placement(
                transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
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
            ia(start=0, fixed=true)) annotation (Placement(transformation(
                  extent={{10,-90},{30,-70}}, rotation=0)));
          parameter
            Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData
            dcpmData
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
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-30,70})));
          Modelica.Electrical.Analog.Basic.Resistor rMains(R=RMains)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-80,30})));
          Modelica.Electrical.Analog.Basic.Inductor lMains(L=LMains)
            annotation (Placement(transformation(
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
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
              points={{-26,19},{-26,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
              points={{-40,36},{-60,36},{-60,6},{-40,6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
              points={{-40,-6},{-50,-6},{-50,24},{-40,24}},
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
              points={{-30,59},{-30,40}},
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
            __Dymola_experimentSetupOutput);
        end ThyristorBridge2Pulse_DC_Drive;
      end ThyristorBridge2Pulse;

      package ThyristorBridge2mPulse "2*m pulse thyristor bridge"
        extends Modelica.Icons.ExamplesPackage;
        model ThyristorBridge2mPulse_R
          "2*m pulse thyristor rectifier bridge with resistive load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
              pulse2m(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorBridge2mPulse_R;

        model ThyristorBridge2mPulse_RL
          "2*m pulse thyristor rectifier bridge with R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
              pulse2m(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorBridge2mPulse_RL;

        model ThyristorBridge2mPulse_RLV
          "2*m pulse thyristor rectifier bridge with R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
              pulse2m(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,6},{-20,6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorBridge2mPulse_RLV;

        model ThyristorBridge2mPulse_RLV_Characteristic
          "Characteristic of 2*m pulse thyristor rectifier bridge with R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorBridge2mPulse(
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
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
          connect(currentSensor.p, constantVoltage.n) annotation (Line(
              points={{10,-40},{30,-40}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ramp.y, pulse2m.firingAngle) annotation (Line(
              points={{-30,59},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(resistor.p, rectifier.dc_p) annotation (Line(
              points={{30,40},{-10,40},{-10,6},{-20,6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=10,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorBridge2mPulse_RLV_Characteristic;

        model ThyristorBridge2mPulse_DC_Drive
          "2m pulse thyristor bridge feeding a DC drive"
          import Modelica_Electrical_PowerConverters;
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Integer m(final min=3) = 3 "Number of phases";
          parameter Modelica.SIunits.Voltage Vrms=dcpmData.VaNominal/(2*m/pi*
              sin(pi/(2*m))*Modelica.Electrical.MultiPhase.Functions.factorY2D(
              m)*sqrt(2)) "RMS supply voltage";
          parameter Modelica.SIunits.Frequency f=50 "Frequency";
          parameter Modelica.SIunits.ApparentPower SMains=250E3
            "Mains short circuit apparent power";
          parameter Real lamdaMains=0.1 "Mains short circuit power factor";
          final parameter Modelica.SIunits.Impedance ZMains=Vrms^2/SMains*m
            "Mains short circuit impedance";
          final parameter Modelica.SIunits.Resistance RMains=ZMains*lamdaMains
            "Mains resistance" annotation (Evaluate=true);
          final parameter Modelica.SIunits.Inductance LMains=ZMains*sqrt(1 -
              lamdaMains^2)/(2*pi*f) "Mains inductance"
            annotation (Evaluate=true);
          parameter Modelica.SIunits.Inductance Ld=3*dcpmData.La
            "Smoothing inductance" annotation (Evaluate=true);
          final parameter Modelica.SIunits.Torque tauNominal=dcpmData.ViNominal
              *dcpmData.IaNominal/dcpmData.wNominal "Nominal torque";
          output Modelica.SIunits.AngularVelocity w(displayUnit="rpm") = dcpm.wMechanical;
          output Modelica.SIunits.Torque tau=dcpm.tauShaft;
          Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
                visible=true, transformation(
                origin={-80,-60},
                extent={{-10,-10},{10,10}},
                rotation=0)));
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
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
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
            f=f,
            useConstantFiringAngle=false,
            m=m) annotation (Placement(transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
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
            ia(start=0, fixed=true)) annotation (Placement(transformation(
                  extent={{10,-90},{30,-70}}, rotation=0)));
          parameter
            Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData
            dcpmData
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
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-30,70})));
          Modelica.Electrical.MultiPhase.Basic.Resistor rMains(m=m, R=fill(
                RMains, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-80,30})));
          Modelica.Electrical.MultiPhase.Basic.Inductor lMains(m=m, L=fill(
                LMains, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-80,60})));
          Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation
            (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-80,-30})));
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
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
              points={{-26,19},{-26,12}},
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
              points={{-20,6},{-10,6},{-10,40},{30,40},{30,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(currentSensor.p, dcpm.pin_an) annotation (Line(
              points={{10,-40},{10,-70},{14,-70}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(const.y, pulse2.firingAngle) annotation (Line(
              points={{-30,59},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(ground.p, star.pin_n) annotation (Line(
              points={{-80,-50},{-80,-40}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sinevoltage.plug_n, star.plug_p) annotation (Line(
              points={{-80,-10},{-80,-20}},
              color={0,0,255},
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
              points={{-80,70},{-60,70},{-60,0},{-40,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rectifier.ac, pulse2.ac) annotation (Line(
              points={{-40,0},{-60,0},{-60,30},{-40,30}},
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
            __Dymola_experimentSetupOutput);
        end ThyristorBridge2mPulse_DC_Drive;
      end ThyristorBridge2mPulse;

      package ThyristorCenterTapmPulse
        "m pulse thyristor rectifier with center tap"
        extends Modelica.Icons.ExamplesPackage;
        model ThyristorCenterTapmPulse_R
          "2*m pulse thyristor rectifier with center tap and resistive load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
              pulsem(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorCenterTapmPulse_R;

        model ThyristorCenterTapmPulse_RL
          "2*m pulse thyristor rectifier with center tap and R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
              pulsem(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorCenterTapmPulse_RL;

        model ThyristorCenterTapmPulse_RLV
          "2*m pulse thyristor rectifier with center tap and R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTapmPulse(
              pulsem(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
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
          connect(currentSensor.p, constantVoltage.n) annotation (Line(
              points={{10,-50},{30,-50},{30,-40}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(resistor.p, rectifier.dc_p) annotation (Line(
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ramp.y, pulsem.firingAngle) annotation (Line(
              points={{-30,59},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=10,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorCenterTapmPulse_RLV_Characteristic;
      end ThyristorCenterTapmPulse;

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
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorCenterTap2mPulse_R;

        model ThyristorCenterTap2mPulse_RL
          "m pulse thyristor rectifier bridge with R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
              pulse2m(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ThyristorCenterTap2mPulse_RL;

        model ThyristorCenterTap2mPulse_RLV
          "m pulse thyristor rectifier bridge with R-L load and voltage"
          extends
            Modelica_Electrical_PowerConverters.Examples.ACDC.ExampleTemplates.ThyristorCenterTap2mPulse(
              pulse2m(constantFiringAngle=constantFiringAngle));
          extends Modelica.Icons.Example;
          import Modelica.Constants.pi;
          parameter Modelica.SIunits.Angle constantFiringAngle=90*pi/180
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
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
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
          Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10) annotation
            (Placement(transformation(
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
          connect(currentSensor.p, constantVoltage.n) annotation (Line(
              points={{10,-50},{30,-50},{30,-40}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ramp.y, pulse2m.firingAngle) annotation (Line(
              points={{-30,59},{-30,40},{-30,40}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(resistor.p, rectifier.dc_p) annotation (Line(
              points={{30,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StopTime=10,
              Tolerance=1e-06,
              Interval=0.0002));
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
            pulse2(f=f) annotation (Placement(transformation(
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
              points={{-44,-11},{-44,-20},{-3,-20},{-3,29}},
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
              points={{-80,10},{-60,10},{-60,6},{-50,6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sinevoltage.n, pulse2.ac_n) annotation (Line(
              points={{-80,-10},{-60,-10},{-60,-6},{-50,-6}},
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
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
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
            pulse2(f=f) annotation (Placement(transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
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
          connect(sinevoltage.p, rectifier.ac_p) annotation (Line(
              points={{-80,10},{-62,10},{-62,6},{-40,6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sinevoltage.n, rectifier.ac_n) annotation (Line(
              points={{-80,-10},{-62,-10},{-62,-6},{-40,-6}},
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
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
              points={{-26,19},{-26,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
              points={{-40,36},{-60,36},{-60,6},{-40,6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
              points={{-40,-6},{-56,-6},{-56,24},{-40,24}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Icon(coordinateSystem(
                extent={{-100,-100},{100,100}},
                preserveAspectRatio=true,
                initialScale=0.1,
                grid={2,2})), Diagram(coordinateSystem(
                extent={{-100,-100},{100,100}},
                preserveAspectRatio=false,
                initialScale=0.1,
                grid={2,2}), graphics));
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
          Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage2(V=sqrt(2)
                *Vrms, freqHz=f) annotation (Placement(visible=true,
                transformation(
                origin={-80,-13.9999},
                extent={{-9.999890000000001,-10},{10,10}},
                rotation=-90)));
          Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage1(V=sqrt(2)
                *Vrms, freqHz=f) annotation (Placement(visible=true,
                transformation(
                origin={-80,14},
                extent={{-10,-10},{10,10}},
                rotation=-90)));
          Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTap2Pulse
            rectifier
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
            pulse2(f=f) annotation (Placement(transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
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
          connect(sinevoltage1.n, sinevoltage2.p) annotation (Line(
              points={{-80,4},{-80,-4.00001}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ground.p, sinevoltage1.n) annotation (Line(
              points={{-90,-40},{-90,0},{-80,0},{-80,4}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sinevoltage1.p, rectifier.ac_p) annotation (Line(
              points={{-80,24},{-80,32},{-60,32},{-60,6},{-40,6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sinevoltage2.n, rectifier.ac_n) annotation (Line(
              points={{-80,-23.9999},{-80,-30},{-60,-30},{-60,-6},{-40,-6}},
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
              points={{50,20},{50,40},{-10,40},{-10,0},{-20,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pulse2.fire_p, rectifier.fire_p) annotation (Line(
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2.fire_n, rectifier.fire_n) annotation (Line(
              points={{-26,19},{-26,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(rectifier.ac_n, pulse2.ac_n) annotation (Line(
              points={{-40,-6},{-46,-6},{-46,24},{-40,24}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pulse2.ac_p, rectifier.ac_p) annotation (Line(
              points={{-40,36},{-50,36},{-50,6},{-40,6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Icon(coordinateSystem(
                extent={{-100,-100},{100,100}},
                preserveAspectRatio=true,
                initialScale=0.1,
                grid={2,2})), Diagram(coordinateSystem(
                extent={{-100,-100},{100,100}},
                preserveAspectRatio=false,
                initialScale=0.1,
                grid={2,2}), graphics));
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
          Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
                transformation(extent={{-90,-80},{-70,-60}})));
          Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation
            (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-80,-40})));
          Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
            final m=m,
            V=fill(sqrt(2)*Vrms, m),
            phase=-
                Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),

            freqHz=fill(f, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-80,-10})));

          Modelica_Electrical_PowerConverters.ACDC.ThyristorBridge2mPulse
            rectifier(final m=m)
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
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
            pulse2m(m=m, f=f) annotation (Placement(transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
        equation
          connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
              points={{-80,-20},{-80,-30}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(star.pin_n, ground.p) annotation (Line(
              points={{-80,-50},{-80,-60}},
              color={0,0,255},
              smooth=Smooth.None));
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
              points={{-80,0},{-40,0}},
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
          connect(currentSensor.p, voltagesensor.n) annotation (Line(
              points={{10,-40},{50,-40},{50,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pulse2m.fire_p, rectifier.fire_p) annotation (Line(
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2m.fire_n, rectifier.fire_n) annotation (Line(
              points={{-26,19},{-26,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2m.ac, sineVoltage.plug_p) annotation (Line(
              points={{-40,30},{-80,30},{-80,4.44089e-16}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), Icon(
                coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}), graphics));
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
          Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
                transformation(extent={{-80,-100},{-60,-80}})));
          Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation
            (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-70,-30})));
          Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_p(
            final m=m,
            V=fill(sqrt(2)*Vrms, m),
            phase=-
                Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),

            freqHz=fill(f, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-70,0})));

          Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTapmPulse
            rectifier(final m=m)
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
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
            constantFiringAngle=1.5707963267949) annotation (Placement(
                transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
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
              points={{-20,8.88178e-16},{-10,8.88178e-16},{-10,40},{50,40},{50,
                  20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(currentSensor.p, voltagesensor.n) annotation (Line(
              points={{10,-50},{50,-50},{50,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pulsem.fire_p, rectifier.fire_p) annotation (Line(
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulsem.ac, sineVoltage_p.plug_p) annotation (Line(
              points={{-40,30},{-70,30},{-70,10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(star.plug_p, sineVoltage_p.plug_n) annotation (Line(
              points={{-70,-20},{-70,-10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(star.pin_n, currentSensor.n) annotation (Line(
              points={{-70,-40},{-70,-50},{-10,-50}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(rectifier.ac, sineVoltage_p.plug_p) annotation (Line(
              points={{-40,0},{-50,0},{-50,30},{-70,30},{-70,10}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), Icon(
                coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}), graphics));
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
          Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
                transformation(extent={{-90,-100},{-70,-80}})));
          Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation
            (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-100,-10})));
          Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_p(
            final m=m,
            V=fill(sqrt(2)*Vrms, m),
            phase=-
                Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),

            freqHz=fill(f, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-70,10})));

          Modelica_Electrical_PowerConverters.ACDC.ThyristorCenterTap2mPulse
            rectifier(final m=m)
            annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
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
            constantFiringAngle=1.5707963267949) annotation (Placement(
                transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={-30,30})));
          Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_n(
            final m=m,
            V=fill(sqrt(2)*Vrms, m),
            phase=-
                Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),

            freqHz=fill(f, m)) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-70,-20})));

        equation
          connect(star.pin_n, ground.p) annotation (Line(
              points={{-100,-20},{-100,-70},{-80,-70},{-80,-80}},
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
              points={{-20,8.88178e-16},{-10,8.88178e-16},{-10,40},{50,40},{50,
                  20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(currentSensor.p, voltagesensor.n) annotation (Line(
              points={{10,-50},{50,-50},{50,0}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(pulse2m.fire_p, rectifier.fire_p) annotation (Line(
              points={{-34,19},{-34,12}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(pulse2m.ac, sineVoltage_p.plug_p) annotation (Line(
              points={{-40,30},{-70,30},{-70,20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sineVoltage_p.plug_n, sineVoltage_n.plug_p) annotation (Line(
              points={{-70,1.33227e-15},{-70,-4},{-70,-4},{-70,-4},{-70,-4},{-70,
                  -10}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sineVoltage_n.plug_n, rectifier.ac_n) annotation (Line(
              points={{-70,-30},{-70,-40},{-50,-40},{-50,-6},{-40,-6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(sineVoltage_p.plug_p, rectifier.ac_p) annotation (Line(
              points={{-70,20},{-70,30},{-50,30},{-50,6},{-40,6}},
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
              points={{-26,19},{-26,12}},
              color={255,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), Icon(
                coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}}), graphics));
        end ThyristorCenterTap2mPulse;
      end ExampleTemplates;
    end ACDC;

    package DCAC "DC to AC converters examples"
      extends Modelica.Icons.ExamplesPackage;
      package SinglePhaseTwoLevel "Single phase two level inverter examples"
        extends Modelica.Icons.ExamplesPackage;
        model SinglePhaseTwoLevel_R
          "Single phase DC to AC converter with resistive load"
          extends
            Modelica_Electrical_PowerConverters.Examples.DCAC.ExampleTemplates.SinglePhaseTwoLevel(
              sine(amplitude=0.5, offset=0.5));
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
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StartTime=0,
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end SinglePhaseTwoLevel_R;

        model SinglePhaseTwoLevel_RL
          "Single phase DC to AC converter with R-L load"
          extends
            Modelica_Electrical_PowerConverters.Examples.DCAC.ExampleTemplates.SinglePhaseTwoLevel(
              sine(amplitude=0.5, offset=0.5));
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
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StartTime=0,
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end SinglePhaseTwoLevel_RL;
      end SinglePhaseTwoLevel;

      package ExampleTemplates "Templates for examples"
        extends Modelica.Icons.Package;
        partial model SinglePhaseTwoLevel
          "Single phas two level inverter including control"
          extends Icons.ExampleTemplate;
          extends Modelica.Icons.Example;
          Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage2(V
              =50) annotation (Placement(transformation(
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
              f=1000, useConstantDutyCycle=false) annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=0,
                origin={-60,-30})));
          Modelica.Blocks.Logical.Not inverse annotation (Placement(
                transformation(
                extent={{-10,10},{10,-10}},
                rotation=90,
                origin={-30,-10})));
          Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage1(V
              =50) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-70,50})));
          Modelica.Blocks.Sources.Sine sine(freqHz=50) annotation (Placement(
                transformation(extent={{-50,-64},{-70,-44}})));
        equation
          connect(voltageSensor.n, currentSensor.p) annotation (Line(
              points={{70,-20},{70,-70},{10,-70}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(signalPWM.fire, inverter.fire_p) annotation (Line(
              points={{-49,-30},{-44,-30},{-44,18}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(inverse.u, signalPWM.fire) annotation (Line(
              points={{-30,-22},{-30,-30},{-49,-30}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(inverse.y, inverter.fire_n) annotation (Line(
              points={{-30,1},{-30,10},{-36,10},{-36,18}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(constantVoltage1.n, constantVoltage2.p) annotation (Line(
              points={{-70,40},{-70,20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(constantVoltage1.p, inverter.dc_p) annotation (Line(
              points={{-70,60},{-50,60},{-50,40}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(constantVoltage2.n, inverter.dc_n) annotation (Line(
              points={{-70,0},{-50,0},{-50,20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(ground.p, constantVoltage1.n) annotation (Line(
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
              points={{-71,-54},{-80,-54},{-80,-30},{-72,-30}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics));
        end SinglePhaseTwoLevel;
      end ExampleTemplates;
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
                origin={30,40})));
        equation
          connect(chopperStepDown.dc_p2, resistor.p) annotation (Line(
              points={{-40,6},{-30,6},{-30,50},{30,50}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(resistor.n, currentSensor.p) annotation (Line(
              points={{30,30},{30,-6},{0,-6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StartTime=0,
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
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
                origin={30,40})));
          Modelica.Electrical.Analog.Basic.Inductor inductor(L=L) annotation (
              Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={30,10})));
        equation
          connect(chopperStepDown.dc_p2, resistor.p) annotation (Line(
              points={{-40,6},{-30,6},{-30,50},{30,50}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(resistor.n, inductor.p) annotation (Line(
              points={{30,30},{30,20}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(inductor.n, currentSensor.p) annotation (Line(
              points={{30,0},{30,-6},{0,-6}},
              color={0,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics), experiment(
              StartTime=0,
              StopTime=0.1,
              Tolerance=1e-06,
              Interval=0.0002));
        end ChopperStepDown_RL;
      end ChopperStepDown;

      package ExampleTemplates "Templates for examples"
        extends Modelica.Icons.Package;
        partial model ChopperStepDown "Step down chopper including control"
          extends Icons.ExampleTemplate;
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
          Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
                transformation(extent={{-90,-40},{-70,-20}})));
          Modelica_Electrical_PowerConverters.DCDC.Control.SignalPWM signalPWM(
              constantDutyCycle=0.25, f=1000) annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={-50,30})));
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
              points={{-40,6},{-30,6},{-30,50},{60,50},{60,20}},
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
          connect(signalPWM.fire, chopperStepDown.fire) annotation (Line(
              points={{-50,19},{-50,12}},
              color={255,0,255},
              smooth=Smooth.None));
          annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}}), graphics));
        end ChopperStepDown;
      end ExampleTemplates;
    end DCDC;
  end Examples;

  package ACDC "AC to DC converter"
    package Control "Control components for rectifiers"
      extends Modelica.Icons.Package;
      block Signal2mPulse "Boolean impulses for 2*m pulse rectifiers"
        import Modelica.Constants.pi;
        parameter Integer m(final min=1) = 3 "Number of phases";
        parameter Boolean useConstantFiringAngle=true
          "Use constant firing angle instead of signal input";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
        parameter Modelica.SIunits.Angle constantFiringAngle=0 "Firing angle"
          annotation (Dialog(enable=useConstantFiringAngle));
        Modelica.Blocks.Interfaces.RealInput firingAngle if not
          useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,100})));
        parameter Modelica.SIunits.Angle firingAngleMax(
          min=0,
          max=Modelica.Constants.pi) = Modelica.Constants.pi
          "Maximum firing angle";
        Modelica.Blocks.Sources.Constant constantconstantFiringAngle(final k=
              constantFiringAngle) if useConstantFiringAngle annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-30,90})));
        Modelica.Blocks.Logical.GreaterThreshold positiveThreshold[m](threshold
            =zeros(m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-2})));
        Modelica.Blocks.Logical.LessThreshold negativeThreshold[m](threshold=
              zeros(m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,0})));
        Modelica.Blocks.Logical.Timer timerPositive[m] annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-32})));
        Modelica.Blocks.Logical.Timer timerNegative[m] annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-32})));
        Modelica.Blocks.Logical.Greater greaterPositive[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-72})));
        Modelica.Blocks.Logical.Greater negativeEqual[m] annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-72})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-110})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-110})));
        Modelica.Blocks.Math.Gain gain(final k=1/2/pi/f) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,0})));
        Modelica.Blocks.Routing.Replicator replicator(final nout=m) annotation
          (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-30})));
        Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi,
              firingAngleMax), final uMin=0) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,50})));
        Modelica.Blocks.Interfaces.RealInput v[m] "Voltages" annotation (
            Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=0,
              origin={-100,0})));
        Filter filter[m](each final f=f, each final fCut=2*f) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,20})));
      equation
        connect(positiveThreshold.y, timerPositive.u) annotation (Line(
            points={{-40,-13},{-40,-14},{-40,-14},{-40,-16},{-40,-16},{-40,-20}},

            color={255,0,255},
            smooth=Smooth.None));

        connect(negativeThreshold.y, timerNegative.u) annotation (Line(
            points={{40,-11},{40,-20}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(timerPositive.y, greaterPositive.u1) annotation (Line(
            points={{-40,-43},{-40,-48},{-42,-48},{-42,-52},{-40,-52},{-40,-60}},

            color={0,0,127},
            smooth=Smooth.None));

        connect(negativeEqual.u1, timerNegative.y) annotation (Line(
            points={{40,-60},{40,-43}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(greaterPositive.y, fire_p) annotation (Line(
            points={{-40,-83},{-40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(negativeEqual.y, fire_n) annotation (Line(
            points={{40,-83},{40,-88},{40,-88},{40,-96},{40,-96},{40,-110},{40,
                -110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(gain.y, replicator.u) annotation (Line(
            points={{-2.22045e-15,-11},{-2.22045e-15,-18},{2.22045e-15,-18}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(replicator.y, greaterPositive.u2) annotation (Line(
            points={{-1.33227e-15,-41},{-1.33227e-15,-50},{-48,-50},{-48,-60}},

            color={0,0,127},
            smooth=Smooth.None));

        connect(replicator.y, negativeEqual.u2) annotation (Line(
            points={{-1.33227e-15,-41},{-1.33227e-15,-50},{32,-50},{32,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(limiter.y, gain.u) annotation (Line(
            points={{-1.9984e-15,39},{-1.9984e-15,12},{2.22045e-15,12}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(firingAngle, limiter.u) annotation (Line(
            points={{8.88178e-16,100},{0,100},{0,70},{2.22045e-15,70},{
                2.22045e-15,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(constantconstantFiringAngle.y, limiter.u) annotation (Line(
            points={{-30,79},{-30,70},{0,70},{0,62},{2.22045e-15,62}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(v, filter.u) annotation (Line(
            points={{-100,0},{-80,0},{-80,20},{-72,20}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(filter.y, positiveThreshold.u) annotation (Line(
            points={{-49,20},{-40,20},{-40,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(filter.y, negativeThreshold.u) annotation (Line(
            points={{-49,20},{40,20},{40,12}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,
                0,255}),Line(
                      points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},
                  {-40,-60}},
                      color={255,0,255},
                      smooth=Smooth.None),Line(
                      points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,
                  -60}},
                      color={255,0,255},
                      smooth=Smooth.None),Text(
                      extent={{-40,60},{40,0}},
                      lineColor={255,0,255},
                      fillColor={0,0,255},
                      fillPattern=FillPattern.Solid,
                      textString="2*%m%")}));
      end Signal2mPulse;

      model VoltageBridge2Pulse "Control for 2 pulse bridge rectifier"
        import Modelica.Constants.pi;
        parameter Boolean useConstantFiringAngle=true
          "Use constant firing angle instead of signal input";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
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
        Modelica.Blocks.Interfaces.RealInput firingAngle if not
          useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,100})));
        Signal2mPulse twoPulse(
          final useConstantFiringAngle=useConstantFiringAngle,
          final f=f,
          final constantFiringAngle=constantFiringAngle,
          final firingAngleMax=firingAngleMax,
          final m=1) annotation (Placement(transformation(
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
              rotation=270,
              origin={-40,-110})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_n annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-110})));
      equation
        connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
            points={{-70,-2.22045e-15},{-60,-2.22045e-15},{-60,0},{-10,0},{-10,
                6.66134e-16}},
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
        connect(twoPulse.fire_p[1], fire_p) annotation (Line(
            points={{-4,-11},{-4,-60},{-40,-60},{-40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(twoPulse.fire_n[1], fire_n) annotation (Line(
            points={{4,-11},{4,-60},{40,-60},{40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(firingAngle, twoPulse.firingAngle) annotation (Line(
            points={{8.88178e-16,100},{8.88178e-16,10}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255}),
              Line(
                points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,
                    -60}},
                color={255,0,255},
                smooth=Smooth.None),
              Line(
                points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},

                color={255,0,255},
                smooth=Smooth.None),
              Text(
                extent={{-40,60},{40,0}},
                lineColor={255,0,255},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                textString="2")}));
      end VoltageBridge2Pulse;

      model VoltageBridge2mPulse "Control for 2*m pulse bridge rectifier"
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Boolean useConstantFiringAngle=true
          "Use constant firing angle instead of signal input";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
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
        Modelica.Blocks.Interfaces.RealInput firingAngle if not
          useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,100})));
        Signal2mPulse twomPulse(
          final useConstantFiringAngle=useConstantFiringAngle,
          final f=f,
          final constantFiringAngle=constantFiringAngle,
          final firingAngleMax=firingAngleMax,
          final m=m) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={0,10})));
        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Electrical.MultiPhase.Basic.Delta delta(final m=m)
          "Delta connection" annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-80,10})));
        Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensor(
            final m=m) "Voltage sensor" annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=270,
              origin={-44,10})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-110})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-110})));
      equation
        connect(ac, voltageSensor.plug_p) annotation (Line(
            points={{-100,4.44089e-16},{-100,-4.44089e-16},{-44,-4.44089e-16}},

            color={0,0,255},
            smooth=Smooth.None));

        connect(voltageSensor.plug_p, delta.plug_n) annotation (Line(
            points={{-44,-4.44089e-16},{-54,-4.44089e-16},{-54,0},{-62,0},{-62,
                -4.44089e-16},{-80,-4.44089e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_p, voltageSensor.plug_n) annotation (Line(
            points={{-80,20},{-44,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensor.v, twomPulse.v) annotation (Line(
            points={{-33,10},{-10,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(twomPulse.fire_n, fire_n) annotation (Line(
            points={{4,-1},{4,-60},{40,-60},{40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(twomPulse.fire_p, fire_p) annotation (Line(
            points={{-4,-1},{-4,-60},{-40,-60},{-40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(firingAngle, twomPulse.firingAngle) annotation (Line(
            points={{0,100},{0,40},{8.88178e-16,40},{8.88178e-16,20}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,127},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                lineColor={0,0,255}),
              Line(
                points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,
                    -60}},
                color={255,0,255},
                smooth=Smooth.None),
              Line(
                points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,-60}},

                color={255,0,255},
                smooth=Smooth.None),
              Text(
                extent={{-40,60},{40,0}},
                lineColor={255,0,255},
                fillColor={0,0,255},
                fillPattern=FillPattern.Solid,
                textString="2*%m%")}));
      end VoltageBridge2mPulse;

      model VoltageCenterTap2mPulse
        "Control for 2*m pulse cetner tap rectifier"
        import Modelica.Constants.pi;
        parameter Integer m(final min=3) = 3 "Number of phases";
        parameter Boolean useConstantFiringAngle=true
          "Use constant firing angle instead of signal input";
        parameter Modelica.SIunits.Frequency f=50 "Frequency";
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
        Modelica.Blocks.Interfaces.RealInput firingAngle if not
          useConstantFiringAngle "Firing angle (rad)" annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=270,
              origin={0,100})));
        Signal2mPulse twomPulse(
          final useConstantFiringAngle=useConstantFiringAngle,
          final f=f,
          final constantFiringAngle=constantFiringAngle,
          final firingAngleMax=firingAngleMax,
          final m=m) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={10,0})));
        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_p[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,-110})));
        Modelica.Blocks.Interfaces.BooleanOutput fire_n[m] annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-110})));
        Modelica.Electrical.MultiPhase.Basic.Delta delta(final m=m)
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
        connect(twomPulse.fire_n, fire_n) annotation (Line(
            points={{14,-11},{14,-60},{40,-60},{40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(twomPulse.fire_p, fire_p) annotation (Line(
            points={{6,-11},{6,-60},{-40,-60},{-40,-110}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(firingAngle, twomPulse.firingAngle) annotation (Line(
            points={{0,100},{0,40},{10,40},{10,10}},
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
            points={{-7,0},{-4.44089e-16,0}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltageSensor.phi, gain.u) annotation (Line(
            points={{-39,0},{-30,0}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={Rectangle(
                      extent={{-100,100},{100,-100}},
                      lineColor={0,0,127},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Text(
                      extent={{-150,150},{150,110}},
                      textString="%name",
                      lineColor={0,0,255}),Line(
                      points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},
                  {-40,-60}},
                      color={255,0,255},
                      smooth=Smooth.None),Line(
                      points={{20,-20},{20,-44},{40,-44},{40,-60},{20,-60},{20,
                  -60}},
                      color={255,0,255},
                      smooth=Smooth.None),Text(
                      extent={{-40,60},{40,0}},
                      lineColor={255,0,255},
                      fillColor={0,0,255},
                      fillPattern=FillPattern.Solid,
                      textString="2*%m%")}));
      end VoltageCenterTap2mPulse;

      block Filter "PT1 + allpass filter"
        extends Modelica.Blocks.Interfaces.SISO;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Frequency f=50 "Mains Frequency";
        parameter Modelica.SIunits.Frequency fCut=2*f "Cut off frequency";
        final parameter Integer na(final min=2) = 2
          "Count of 1st order allpass";
        final parameter Modelica.SIunits.Frequency fa=f/tan(pi/na - atan(f/fCut)
            /(2*na));
        parameter Real yStart=0 "Start value of output";
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
                  {100,100}}),graphics),
          Documentation(info="<html>
<p>PT1-filter with cut-off frequency fCut. </p>
<p>The phase shift is compensated by a series of two 1st order allpass filters for frequency f.</p>
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
    end Control;
    extends Modelica.Icons.Package;
    model DiodeBridge2Pulse "Two pulse Graetz diode rectifier bridge "
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
        "Closed diode resistance";
      parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
        "Opened diode conductance";
      parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
        "Diode forward threshold voltage";

      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        "Positive AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        "Negative AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Postive DC output"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC output"
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
      Modelica.SIunits.Current iAC=ac_p.i "AC currents";
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_p1(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort)
        "Diode connecting the positve AC input pin with postitive DC output"
        annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_p2(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort)
        "Diode connecting the negative AC input pin with postitive DC output"
        annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n1(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort)
        "Diode connecting the positve AC input pin with negative DC output"
        annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n2(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort)
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
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end DiodeBridge2Pulse;

    model ThyristorBridge2Pulse "Two pulse Graetz thyristor rectifier bridge"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
        "Thyristor forward threshold voltage";
      parameter Boolean offStart_p1=true
        "Boolean off start value for thyristor_p1";
      parameter Boolean offStart_p2=true
        "Boolean off start value for thyristor_p2";
      parameter Boolean offStart_n1=true
        "Boolean off start value for thyristor_n1";
      parameter Boolean offStart_n2=true
        "Boolean off start value for thyristor_n2";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        "Positive AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        "Negative AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Postive DC output"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC output"
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p
        "Fire signal for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n
        "Fire signal for negative potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
      Modelica.SIunits.Current iAC=ac_p.i "AC currents";
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p1(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_p1, fixed=true))
        "Thyristor connecting the positve AC input pin with postitive DC output"
        annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p2(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_p2, fixed=true))
        "Thyristor connecting the negative AC input pin with postitive DC output"
        annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n1(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_n1, fixed=true))
        "Thyristor connecting the positve AC input with negative DC output"
        annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n2(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_n2, fixed=true))
        "Thyristor connecting the negative AC input with negative DC output"
        annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
    equation
      if not useHeatPort then
        LossPower = thyristor_p1.LossPower + thyristor_p2.LossPower +
          thyristor_n1.LossPower + thyristor_n2.LossPower;
      end if;
      connect(thyristor_p2.n, thyristor_p1.n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n1.p, thyristor_n2.p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n2.n, thyristor_p2.p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p1.p, thyristor_n1.n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_n2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, thyristor_p1.p) annotation (Line(
          points={{-100,60},{-80,60},{-80,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, thyristor_n2.n) annotation (Line(
          points={{-100,-60},{-80,-60},{-80,-20},{40,-20},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p1.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_n, thyristor_p2.fire) annotation (Line(
          points={{40,120},{40,80},{60,80},{60,56},{56,56},{56,57},{51,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_p, thyristor_n2.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,-43},{29,-43}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_n, thyristor_n1.fire) annotation (Line(
          points={{40,120},{40,80},{60,80},{60,-10},{-10,-10},{-10,-43},{-1,-43}},

          color={255,0,255},
          smooth=Smooth.None));

      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
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
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              lineColor={0,0,255}),
            Rectangle(
              extent={{-38,26},{42,-22}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-38,2},{42,2}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{22,26},{22,-22}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{22,2},{-18,26},{-18,-22},{22,2}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{2,14},{2,30}},
              color={0,0,255},
              smooth=Smooth.None)}));
    end ThyristorBridge2Pulse;

    model HalfBridge2Pulse "Two pulse Graetz half rectifier bridge "
      import Modelica.Constants.pi;
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
        "Boolean off start value for thyristor_p1";
      parameter Boolean offStart_p2=true
        "Boolean off start value for thyristor_p2";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        "Positive AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        "Negative AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Postive DC output"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC output"
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p
        "Fire signal for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n
        "Fire signal for negative potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
      Modelica.SIunits.Current iAC=ac_p.i "AC currents";
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p1(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_p1, fixed=true)) annotation (Placement(visible=true,
            transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p2(
        Ron=RonThyristor,
        Goff=GoffThyristor,
        Vknee=VkneeThyristor,
        useHeatPort=useHeatPort,
        off(start=offStart_p2, fixed=true)) annotation (Placement(visible=true,
            transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n1(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort) "Diode connected to negative DC potential"
        annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n2(
        Ron=RonDiode,
        Goff=GoffDiode,
        Vknee=VkneeDiode,
        useHeatPort=useHeatPort) "Diode connected to negative DC potential"
        annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
    equation
      if not useHeatPort then
        LossPower = thyristor_p1.LossPower + thyristor_p2.LossPower + diode_n1.LossPower
           + diode_n2.LossPower;
      end if;
      connect(thyristor_p2.n, thyristor_p1.n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n1.p, diode_n2.p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n2.n, thyristor_p2.p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p1.p, diode_n1.n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p1.n, dc_p) annotation (Line(
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
      connect(thyristor_p1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, thyristor_p1.p) annotation (Line(
          points={{-100,60},{-80,60},{-80,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, diode_n2.n) annotation (Line(
          points={{-100,-60},{-80,-60},{-80,-20},{40,-20},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p1.fire) annotation (Line(
          points={{-40,120},{-40,80},{26,80},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(thyristor_p2.fire, fire_n) annotation (Line(
          points={{51,57},{56,57},{56,80},{40,80},{40,120}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-44,48},{36,0}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-44,24},{36,24}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,48},{16,0}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,24},{-24,48},{-24,0},{16,24}},
                  color={0,0,255},
                  smooth=Smooth.None),Rectangle(
                  extent={{-44,0},{36,-56}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-44,-32},{36,-32}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,-8},{16,-56}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,-32},{-24,-8},{-24,-56},{16,-32}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{-4,-20},{-4,-4}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end HalfBridge2Pulse;

    model DiodeCenterTap2Pulse "Two pulse diode rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
        "Closed diode resistance";
      parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
        "Opened diode conductance";
      parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
        "Diode forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        "Positive AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        "Negative AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
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
            origin={10,60},
            extent={{-10,10},{10,-10}},
            rotation=0)));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n(
        final Ron=RonDiode,
        final Goff=GoffDiode,
        final Vknee=VkneeDiode,
        final useHeatPort=useHeatPort)
        "Diodes conducting negative pin AC potentials" annotation (Placement(
            visible=true, transformation(
            origin={10,-60},
            extent={{-10,-10},{10,10}},
            rotation=0)));
    equation
      if not useHeatPort then
        LossPower = diode_p.LossPower + diode_n.LossPower;
      end if;
      connect(ac_p, diode_p.p) annotation (Line(
          points={{-100,60},{0,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, diode_n.p) annotation (Line(
          points={{-100,-60},{0,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_p.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_p.heatPort, heatPort) annotation (Line(
          points={{10,70},{30,70},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Icon(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.1,
            grid={2,2}), graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
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
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None)}), Diagram(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=false,
            initialScale=0.1,
            grid={2,2}), graphics));
    end DiodeCenterTap2Pulse;

    model ThyristorCenterTap2Pulse
      "Two pulse thyristor rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
        "Thyristor forward threshold voltage";
      parameter Boolean offStart_p=true
        "Boolean off start value for thyristor_p1";
      parameter Boolean offStart_n=true
        "Boolean off start value for thyristor_p2";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        "Positive AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        "Negative AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Postive DC output"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p
        "Fire signal for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n
        "Fire signal for negative potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
      Modelica.SIunits.Current iAC=ac_p.i "AC currents";
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_p(
        final Ron=RonThyristor,
        final Goff=GoffThyristor,
        final Vknee=VkneeThyristor,
        final useHeatPort=useHeatPort,
        off(start=offStart_p, fixed=true))
        "Thyristors conducting positive pin AC potentials" annotation (
          Placement(visible=true, transformation(
            origin={10,60},
            extent={{-10,10},{10,-10}},
            rotation=0)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor thyristor_n(
        final Ron=RonThyristor,
        final Goff=GoffThyristor,
        final Vknee=VkneeThyristor,
        final useHeatPort=useHeatPort,
        off(start=offStart_n, fixed=true))
        "Thyristors conducting negative pin AC potentials" annotation (
          Placement(visible=true, transformation(
            origin={10,-60},
            extent={{-10,-10},{10,10}},
            rotation=0)));
    equation
      if not useHeatPort then
        LossPower = thyristor_p.LossPower + thyristor_n.LossPower;
      end if;
      connect(ac_p, thyristor_p.p) annotation (Line(
          points={{-100,60},{-4.44089e-16,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, thyristor_n.p) annotation (Line(
          points={{-100,-60},{-4.44089e-16,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p.heatPort, heatPort) annotation (Line(
          points={{10,70},{10,80},{30,80},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p.fire) annotation (Line(
          points={{-40,120},{-40,40},{17,40},{17,49}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_n, thyristor_n.fire) annotation (Line(
          points={{40,120},{40,-40},{17,-40},{17,-49}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Icon(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=true,
            initialScale=0.1,
            grid={2,2}), graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
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
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{2,14},{2,30}},
                  color={0,0,255},
                  smooth=Smooth.None)}), Diagram(coordinateSystem(
            extent={{-100,-100},{100,100}},
            preserveAspectRatio=false,
            initialScale=0.1,
            grid={2,2}), graphics));
    end ThyristorCenterTap2Pulse;

    model DiodeBridge2mPulse "2*m pulse diode rectifier bridge"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
        "Closed diode resistance";
      parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
        "Opened diode conductance";
      parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
        "Diode forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Postive DC output"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC output"
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
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
        "Diodes connected to positive DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        each final useHeatPort=useHeatPort)
        "Diodes connected to negative DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
        annotation (Placement(transformation(extent={{20,70},{40,50}})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
        annotation (Placement(transformation(extent={{20,-50},{40,-70}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(diode_p.idealDiode.LossPower) + sum(diode_n.idealDiode.LossPower);
      end if;
      connect(ac, diode_p.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_p.plug_p, diode_n.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_p.plug_n, star_p.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.plug_p, star_n.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end DiodeBridge2mPulse;

    model ThyristorBridge2mPulse "2*m pulse thyristor rectifier bridge"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
        "Thyristor forward threshold voltage";
      parameter Boolean offStart_p[m]=fill(true, m)
        "Boolean off start value for thyristor_p";
      parameter Boolean offStart_n[m]=fill(true, m)
        "Boolean off start value for thyristor_n";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
        "Fire signals for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
        "Fire signasl for negative potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
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
        final offStart=offStart_p)
        "Thyristors connected to positive DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_n(
        final m=m,
        final Ron=fill(RonThyristor, m),
        final Goff=fill(GoffThyristor, m),
        final Vknee=fill(VkneeThyristor, m),
        each final useHeatPort=useHeatPort,
        final offStart=offStart_n)
        "Thyristors connected to negative DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
        annotation (Placement(transformation(extent={{20,70},{40,50}})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
        annotation (Placement(transformation(extent={{20,-50},{40,-70}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(thyristor_n.idealThyristor.LossPower);
      end if;
      connect(ac, thyristor_p.plug_p) annotation (Line(
          points={{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p.plug_p, thyristor_n.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n.plug_p, star_n.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, thyristor_n.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p.fire) annotation (Line(
          points={{-40,120},{-40,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_n, thyristor_n.fire) annotation (Line(
          points={{40,120},{40,80},{-20,80},{-20,-33},{-1,-33}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
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
              extent={{-100,70},{0,50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="AC"),
            Text(
              extent={{0,-50},{100,-70}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="DC"),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              lineColor={0,0,255}),
            Rectangle(
              extent={{-38,26},{42,-22}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-38,2},{42,2}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{22,26},{22,-22}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{22,2},{-18,26},{-18,-22},{22,2}},
              color={0,0,255},
              smooth=Smooth.None),
            Line(
              points={{2,14},{2,30}},
              color={0,0,255},
              smooth=Smooth.None)}));
    end ThyristorBridge2mPulse;

    model HalfBridge2mPulse "2*m pulse half rectifier bridge"
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
        "Boolean off start value for thyristor_p";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC output"
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        "AC input"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
        "Fire signals for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
      Modelica.SIunits.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC[m]=ac.pin[:].v "AC voltages";
      Modelica.SIunits.Current iAC[m]=ac.pin[:].i "AC currents";
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
        annotation (Placement(transformation(extent={{20,70},{40,50}})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
        annotation (Placement(transformation(extent={{20,-50},{40,-70}})));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_p(
        final m=m,
        final Ron=fill(RonThyristor, m),
        final Goff=fill(GoffThyristor, m),
        final Vknee=fill(VkneeThyristor, m),
        each final useHeatPort=useHeatPort,
        final offStart=offStart_p)
        "Thyristors connected to positive DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        each final useHeatPort=useHeatPort)
        "Diodes connected to negative DC potential" annotation (Placement(
            visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(diode_n.idealDiode.LossPower);
      end if;
      connect(ac, thyristor_p.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{10,4.44089e-16},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p.plug_p, diode_n.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.plug_p, star_n.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p.fire) annotation (Line(
          points={{-40,120},{-40,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-44,48},{36,0}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-44,24},{36,24}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,48},{16,0}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,24},{-24,48},{-24,0},{16,24}},
                  color={0,0,255},
                  smooth=Smooth.None),Rectangle(
                  extent={{-44,0},{36,-56}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-44,-32},{36,-32}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,-8},{16,-56}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{16,-32},{-24,-8},{-24,-56},{16,-32}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{-4,-20},{-4,-4}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end HalfBridge2mPulse;

    model DiodeCenterTap2mPulse "2*m pulse diode rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
        "Closed diode resistance";
      parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
        "Opened diode conductance";
      parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
        "Diode forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac_p(final m=m)
        "Positive potential AC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.MultiPhase.Interfaces.NegativePlug ac_n(final m=m)
        "Negative potential AC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC[m]=ac_p.pin[:].v - ac_n.pin[:].v
        "AC voltages";
      Modelica.SIunits.Current iAC[m]=ac_p.pin[:].i "AC currents";
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        each final useHeatPort=useHeatPort)
        "Diodes connected to positive DC potential" annotation (Placement(
            visible=true, transformation(
            origin={-10,60},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        each final useHeatPort=useHeatPort)
        "Diodes connected to negative DC potential" annotation (Placement(
            visible=true, transformation(
            origin={-10,-60},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
        annotation (Placement(transformation(extent={{10,70},{30,50}})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
        annotation (Placement(transformation(extent={{12,-50},{32,-70}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(diode_p.idealDiode.LossPower) + sum(diode_n.idealDiode.LossPower);
      end if;
      connect(diode_p.plug_n, star_p.plug_p) annotation (Line(
          points={{0,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.pin_n, dc_p) annotation (Line(
          points={{30,60},{100,60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, diode_n.heatPort) annotation (Line(
          points={{60,-80},{-10,-80},{-10,-70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{-10,50},{-10,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, diode_p.plug_p) annotation (Line(
          points={{-100,60},{-20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_p) annotation (Line(
          points={{32,-60},{100,-60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.plug_p, ac_n) annotation (Line(
          points={{-20,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(diode_n.plug_n, star_n.plug_p) annotation (Line(
          points={{4.44089e-16,-60},{12,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end DiodeCenterTap2mPulse;

    model ThyristorCenterTap2mPulse
      "2*m pulse thyristor rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
        "Thyristor forward threshold voltage";
      parameter Boolean offStart_p[m]=fill(true, m)
        "Boolean off start value for thyristor_p";
      parameter Boolean offStart_n[m]=fill(true, m)
        "Boolean off start value for thyristor_n";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
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
            rotation=270,
            origin={-40,120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
        "Fire signasl for negative potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={40,120})));
      Modelica.SIunits.Voltage vDC=dc_p.v "DC voltage";
      Modelica.SIunits.Current iDC=dc_p.i "DC current";
      Modelica.SIunits.Voltage vAC[m]=ac_p.pin[:].v - ac_n.pin[:].v
        "AC voltages";
      Modelica.SIunits.Current iAC[m]=ac_p.pin[:].i "AC currents";
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_p(
        final m=m,
        final Ron=fill(RonThyristor, m),
        final Goff=fill(GoffThyristor, m),
        final Vknee=fill(VkneeThyristor, m),
        each final useHeatPort=useHeatPort,
        final offStart=offStart_p)
        "Thyristors conducting positive plug AC potentials" annotation (
          Placement(visible=true, transformation(
            origin={-10,60},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor thyristor_n(
        final m=m,
        final Ron=fill(RonThyristor, m),
        final Goff=fill(GoffThyristor, m),
        final Vknee=fill(VkneeThyristor, m),
        each final useHeatPort=useHeatPort,
        final offStart=offStart_n)
        "Thyristors conducting negative plug AC potentials" annotation (
          Placement(visible=true, transformation(
            origin={-10,-60},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m)
        annotation (Placement(transformation(extent={{10,70},{30,50}})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m)
        annotation (Placement(transformation(extent={{12,-50},{32,-70}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(thyristor_p.idealThyristor.LossPower) + sum(thyristor_n.idealThyristor.LossPower);
      end if;
      connect(thyristor_p.plug_n, star_p.plug_p) annotation (Line(
          points={{0,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.pin_n, dc_p) annotation (Line(
          points={{30,60},{100,60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, thyristor_n.heatPort) annotation (Line(
          points={{60,-80},{-10,-80},{-10,-70}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{-10,50},{-10,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(fire_p, thyristor_p.fire) annotation (Line(
          points={{-40,120},{-40,80},{-2,80},{-2,71},{-3,71}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_n, thyristor_n.fire) annotation (Line(
          points={{40,120},{40,-40},{-2,-40},{-2,-49},{-3,-49}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(ac_p, thyristor_p.plug_p) annotation (Line(
          points={{-100,60},{-20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_p) annotation (Line(
          points={{32,-60},{100,-60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n.plug_p, ac_n) annotation (Line(
          points={{-20,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(thyristor_n.plug_n, star_n.plug_p) annotation (Line(
          points={{4.44089e-16,-60},{12,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{2,14},{2,30}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end ThyristorCenterTap2mPulse;

    model DiodeCenterTapmPulse "m pulse diode rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonDiode(final min=0) = 1e-05
        "Closed diode resistance";
      parameter Modelica.SIunits.Conductance GoffDiode(final min=0) = 1e-05
        "Opened diode conductance";
      parameter Modelica.SIunits.Voltage VkneeDiode(final min=0) = 0
        "Diode forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
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
        "Diodes connected to positive DC potential" annotation (Placement(
            visible=true, transformation(
            origin={-10,0},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
        annotation (Placement(transformation(extent={{10,10},{30,-10}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(diode.idealDiode.LossPower);
      end if;
      connect(diode.plug_n, star.plug_p) annotation (Line(
          points={{0,0},{10,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star.pin_n, dc_p) annotation (Line(
          points={{30,0},{100,0},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode.heatPort, thermalCollector.port_a) annotation (Line(
          points={{-10,-10},{-10,-20},{60,-20},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac, diode.plug_p) annotation (Line(
          points={{-100,0},{-20,0}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end DiodeCenterTapmPulse;

    model ThyristorCenterTapmPulse
      "m pulse thyristor rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3) = 3 "Number of phases";
      parameter Modelica.SIunits.Resistance RonThyristor(final min=0) = 1e-05
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance GoffThyristor(final min=0) = 1e-05
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage VkneeThyristor(final min=0) = 0
        "Thyristor forward threshold voltage";
      parameter Boolean offStart[m]=fill(true, m) "Boolean off start value";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC output"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        "AC input"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
        "Fire signals for positive potential semiconductors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={-40,120})));
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
        final offStart=offStart) "Thyristors conducting AC potentials"
        annotation (Placement(visible=true, transformation(
            origin={-10,0},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
        annotation (Placement(transformation(extent={{10,10},{30,-10}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(thyristor.idealThyristor.LossPower);
      end if;
      connect(thyristor.plug_n, star.plug_p) annotation (Line(
          points={{0,0},{10,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star.pin_n, dc_p) annotation (Line(
          points={{30,4.44089e-16},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thyristor.heatPort, thermalCollector.port_a) annotation (Line(
          points={{-10,-10},{-10,-20},{60,-20},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(fire_p, thyristor.fire) annotation (Line(
          points={{-40,120},{-40,80},{-2,80},{-2,11},{-3,11}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(ac, thyristor.plug_p) annotation (Line(
          points={{-100,0},{-20,0}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="AC"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
                  extent={{-38,26},{42,-22}},
                  lineColor={255,255,255},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-38,2},{42,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,26},{22,-22}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{22,2},{-18,26},{-18,-22},{22,2}},
                  color={0,0,255},
                  smooth=Smooth.None),Line(
                  points={{2,14},{2,30}},
                  color={0,0,255},
                  smooth=Smooth.None)}));
    end ThyristorCenterTapmPulse;
  end ACDC;

  package DCAC "DC to AC converters"
    extends Modelica.Icons.Package;
    model SinglePhase2Level "Single phase DC to AC converter"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer m(final min=1) = 3 "Number of phases";
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
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC input"
        annotation (Placement(transformation(extent={{-110,110},{-90,90}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC input"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac "AC output"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p
        "Firing signals of positive potential transistors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-40,-120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n
        "Firing signals of negative potential transistors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={40,-120})));
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
            origin={20,20})));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_p(
        final Ron=RonDiode,
        final Goff=GoffDiode,
        final Vknee=VkneeDiode,
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,20})));
      Modelica.Electrical.Analog.Ideal.IdealGTOThyristor transistor_n(
        final Ron=RonTransistor,
        final Goff=GoffTransistor,
        final Vknee=VkneeTransistor,
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={20,-20})));
      Modelica.Electrical.Analog.Ideal.IdealDiode diode_n(
        final Ron=RonDiode,
        final Goff=GoffDiode,
        final Vknee=VkneeDiode,
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,-20})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
    equation
      if not useHeatPort then
        LossPower = transistor_p.LossPower + diode_n.LossPower + transistor_n.LossPower
           + diode_n.LossPower;
      end if;
      connect(fire_n, transistor_n.fire) annotation (Line(
          points={{40,-120},{40,-80},{-20,-80},{-20,-27},{9,-27}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_p, transistor_p.fire) annotation (Line(
          points={{-40,-120},{-40,13},{9,13}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{0,-100},{0,-62},{0,-62},{0,-60},{
              4.44089e-16,-60}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, transistor_n.heatPort) annotation (Line(
          points={{0,-40},{0,-36},{30,-36},{30,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_n.heatPort, thermalCollector.port_a) annotation (Line(
          points={{70,-20},{70,-36},{0,-36},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(transistor_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{30,20},{30,4},{0,4},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{70,20},{70,4},{0,4},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(transistor_p.p, dc_p) annotation (Line(
          points={{20,30},{40,30},{40,70},{-100,70},{-100,100}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.n, dc_n) annotation (Line(
          points={{20,-30},{40,-30},{40,-70},{-100,-70},{-100,-100}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_p.p, diode_p.n) annotation (Line(
          points={{20,30},{60,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_p.n, diode_p.p) annotation (Line(
          points={{20,10},{60,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.p, diode_n.n) annotation (Line(
          points={{20,-10},{60,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.n, diode_n.p) annotation (Line(
          points={{20,-30},{60,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_p.n, transistor_n.p) annotation (Line(
          points={{20,10},{40,10},{40,-10},{20,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac, transistor_p.n) annotation (Line(
          points={{100,0},{40,0},{40,10},{20,10}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
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
</html>"));
    end SinglePhase2Level;

    model MultiPhase2Level "Multi phase DC to AC converter"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer m(final min=1) = 3 "Number of phases";
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
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        "Positive DC input"
        annotation (Placement(transformation(extent={{-110,110},{-90,90}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        "Negative DC input"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        "AC output"
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.BooleanInput fire_p[m]
        "Firing signals of positive potential transistors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-40,-120})));
      Modelica.Blocks.Interfaces.BooleanInput fire_n[m]
        "Firing signals of negative potential transistors" annotation (
          Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={40,-120})));
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
            origin={20,20})));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_p(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,20})));
      Modelica.Electrical.MultiPhase.Basic.Star star_p(final m=m) annotation (
          Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={40,50})));
      Modelica.Electrical.MultiPhase.Ideal.IdealGTOThyristor transistor_n(
        final m=m,
        final Ron=fill(RonTransistor, m),
        final Goff=fill(GoffTransistor, m),
        final Vknee=fill(VkneeTransistor, m),
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={20,-20})));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode diode_n(
        final m=m,
        final Ron=fill(RonDiode, m),
        final Goff=fill(GoffDiode, m),
        final Vknee=fill(VkneeDiode, m),
        final useHeatPort=useHeatPort) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={60,-20})));
      Modelica.Electrical.MultiPhase.Basic.Star star_n(final m=m) annotation (
          Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=90,
            origin={40,-50})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector
        thermalCollector(final m=m) if useHeatPort
        annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
    equation
      if not useHeatPort then
        LossPower = sum(transistor_p.idealGTOThyristor.LossPower) + sum(diode_n.idealDiode.LossPower)
           + sum(transistor_n.idealGTOThyristor.LossPower) + sum(diode_n.idealDiode.LossPower);
      end if;
      connect(transistor_p.plug_p, star_p.plug_p) annotation (Line(
          points={{20,30},{40,30},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_p.plug_p, diode_p.plug_n) annotation (Line(
          points={{40,40},{40,30},{60,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.plug_n, star_n.plug_p) annotation (Line(
          points={{20,-30},{40,-30},{40,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.plug_p, diode_n.plug_p) annotation (Line(
          points={{40,-40},{40,-30},{60,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_p.plug_n, diode_p.plug_p) annotation (Line(
          points={{20,10},{60,10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.plug_p, diode_n.plug_n) annotation (Line(
          points={{20,-10},{60,-10}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(star_n.pin_n, dc_n) annotation (Line(
          points={{40,-60},{40,-60},{40,-66},{40,-66},{40,-70},{-100,-70},{-100,
              -100}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(dc_p, star_p.pin_n) annotation (Line(
          points={{-100,100},{-100,70},{40,70},{40,60},{40,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_p.plug_n, ac) annotation (Line(
          points={{20,10},{40,10},{40,0},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(transistor_n.plug_p, ac) annotation (Line(
          points={{20,-10},{40,-10},{40,0},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(fire_n, transistor_n.fire) annotation (Line(
          points={{40,-120},{40,-80},{-20,-80},{-20,-27},{9,-27}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(fire_p, transistor_p.fire) annotation (Line(
          points={{-40,-120},{-40,13},{9,13}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{0,-100},{0,-62},{0,-62},{0,-60},{
              4.44089e-16,-60}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, transistor_n.heatPort) annotation (Line(
          points={{0,-40},{0,-36},{30,-36},{30,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_n.heatPort, thermalCollector.port_a) annotation (Line(
          points={{70,-20},{70,-36},{0,-36},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(transistor_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{30,20},{30,4},{0,4},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(diode_p.heatPort, thermalCollector.port_a) annotation (Line(
          points={{70,20},{70,4},{0,4},{0,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Line(
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
</html>"));
    end MultiPhase2Level;
  end DCAC;

  package DCDC "DC to DC converters"
    extends Modelica.Icons.Package;
    package Control "Control components for DC to DC converters"
      extends Modelica.Icons.Package;
      model SignalPWM
        "Generates a pulse width modulated (PWM) boolean fire signal"
        parameter Boolean useConstantDutyCycle=true
          "Enables constant duty cycle";
        parameter Real constantDutyCycle=0 "Constant duty cycle"
          annotation (Dialog(enable=useConstantDutyCycle));
        parameter Modelica.SIunits.Frequency f=50 "Switching frequency";
        parameter Modelica.SIunits.Time startTime=0 "Start time";
        Modelica.Blocks.Interfaces.BooleanOutput fire "Firing PWM signal"
          annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        Modelica.Blocks.Interfaces.RealInput dutyCycle if not
          useConstantDutyCycle "Duty cycle"
          annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
        Modelica.Blocks.Sources.Constant const(final k=constantDutyCycle) if
          useConstantDutyCycle
          annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1, uMin=0)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        Modelica.Blocks.Sources.SampleTrigger sampleTrigger(final period=1/f,
            final startTime=startTime)
          annotation (Placement(transformation(extent={{-100,-60},{-80,-40}})));
        Modelica.Blocks.Logical.Timer timer
          annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
        Modelica.Blocks.Logical.FallingEdge fallingEdge
          annotation (Placement(transformation(extent={{-68,-60},{-48,-40}})));
        Modelica.Blocks.Logical.Not invert
          annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
        Modelica.Blocks.Math.Gain gain(final k=f)
          annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
        Modelica.Blocks.Logical.Less greaterEqual
          annotation (Placement(transformation(extent={{50,-40},{70,-60}})));
        Modelica.Blocks.Logical.And andCondition
          annotation (Placement(transformation(extent={{72,-10},{92,10}})));
        Modelica.Blocks.Sources.BooleanStep enable(final startTime=startTime,
            final startValue=false) "Enable signal of fire after start time"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,50})));
        Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(final startTime=
              startTime, final samplePeriod=1/f)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        connect(const.y, limiter.u) annotation (Line(
            points={{-79,50},{-70,50},{-70,8.88178e-16},{-42,8.88178e-16}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(dutyCycle, limiter.u) annotation (Line(
            points={{-120,8.88178e-16},{-42,8.88178e-16}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sampleTrigger.y, fallingEdge.u) annotation (Line(
            points={{-79,-50},{-70,-50}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(fallingEdge.y, invert.u) annotation (Line(
            points={{-47,-50},{-42,-50}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(invert.y, timer.u) annotation (Line(
            points={{-19,-50},{-12,-50}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(timer.y, gain.u) annotation (Line(
            points={{11,-50},{18,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(gain.y, greaterEqual.u1) annotation (Line(
            points={{41,-50},{48,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(andCondition.y, fire) annotation (Line(
            points={{93,0},{110,0}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(greaterEqual.y, andCondition.u2) annotation (Line(
            points={{71,-50},{80,-50},{80,-20},{60,-20},{60,-8},{70,-8}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(enable.y, andCondition.u1) annotation (Line(
            points={{60,39},{60,0},{70,0}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(limiter.y, zeroOrderHold.u) annotation (Line(
            points={{-19,0},{-12,0}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(zeroOrderHold.y, greaterEqual.u2) annotation (Line(
            points={{11,0},{40,0},{40,-42},{48,-42}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent=
                  {{-100,-100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics={Rectangle(
                      extent={{-100,100},{100,-100}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Line(
                      points={{-100,0},{-98,0},{-62,0}},
                      color={0,0,255},
                      smooth=Smooth.None),Line(
                      points={{-60,-4},{-60,-48},{40,-48},{40,-24}},
                      color={0,0,255},
                      smooth=Smooth.None),Line(
                      points={{-80,-16},{-80,-20},{-40,20},{-40,-20},{-36,-16}},
                      color={0,0,255},
                      smooth=Smooth.None),Line(
                      points={{-62,0},{-76,4},{-76,-4},{-62,0}},
                      color={0,0,255},
                      smooth=Smooth.None),Line(
                      points={{40,-24},{40,-24},{36,-38},{40,-38},{44,-38},{40,
                  -24}},
                      color={0,0,255},
                      smooth=Smooth.None),Line(
                      points={{36,-20},{38,-20},{40,-20},{40,20},{60,20},{60,-20},
                  {80,-20},{80,-16}},
                      color={255,0,255},
                      smooth=Smooth.None)}));
      end SignalPWM;
    end Control;

    model ChopperStepDown "Step down chopper"
      import Modelica.Constants.pi;
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
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(final T
          =293.15);
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p1
        "Positive DC input"
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n1
        "Negative DC input"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p2
        "Postive DC output"
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
      Modelica.Blocks.Interfaces.BooleanInput fire annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,120})));
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
      connect(transistor.fire, fire) annotation (Line(
          points={{7,71},{7,85.5},{1.11022e-15,85.5},{1.11022e-15,120}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(
                  points={{-100,-100},{100,100}},
                  color={0,0,127},
                  smooth=Smooth.None),Text(
                  extent={{-100,70},{0,50}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC in"),Text(
                  extent={{0,-50},{100,-70}},
                  lineColor={0,0,127},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  textString="DC out"),Text(
                  extent={{-150,150},{150,110}},
                  textString="%name",
                  lineColor={0,0,255}),Rectangle(
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
                  smooth=Smooth.None)}));
    end ChopperStepDown;
  end DCDC;

  package Utilities "Utilities for operating power converters"
    extends Modelica.Icons.Package;
  end Utilities;

  package Icons "Icons"
    extends Modelica.Icons.Package;
    model ExampleTemplate
      annotation (Icon(graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={175,175,175},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-36,-60},{-36,60},{64,0},{-36,-60}},
              lineColor={175,175,175},
              smooth=Smooth.None,
              fillColor={175,175,175},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-4,46},{14,-44}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end ExampleTemplate;
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
    version="1.0.0",
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

