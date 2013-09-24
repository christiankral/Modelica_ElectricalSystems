within ;
package QuasiStationaryFundamentalWave 
  extends Modelica.Icons.Package;


  package UsersGuide "User's Guide"
    extends Modelica.Icons.Information;
    class Concept "Fundamental wave concept"
      extends Modelica.Icons.Information;
      annotation (Documentation(info="<html>
<h5>Reference frames</h5>

<p>Quasi stationary  magnetic ports contain the complex magnetic flux (flow variable) and the comlplex magnetic potential difference (potential variable) and a reference ange. The relationship between the different complex phasors with respect to different refrences will be explained by means of the complex magnetic flux. The same transformation relationships the also apply to the complex magnetic potential difference. However, the discussed relationsships are important for handling connectors in the air gap model, transform equations into the rotor fixed reference frame, etc. </p>

<p>
Let us assume that the air gap model contains stator and rotor magnetic ports which relate to the different sides of the machine. The anlge relationship between these ports is
</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/gamma_relationship.png\"/>, 
</p>

<p>where
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/gamma_s.png\"/> 
is the connector reference angle of the stator ports,
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/gamma_r.png\"/> 
is the connector reference angle of the rotor ports, and
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/gamma_mechanical.png\"/> 
is the difference of the mechanical angles of the flange and the support, respectively, 
multiplied by the number of pole pairs,
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/p.png\"/>. 
The stator and rotor reference angles are directly related with the electrical frequencies of the 
electric circuits of the stator, 
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/f_s.png\"/>,
and rotor, 
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/f_r.png\"/>,
respectively, by means of:
</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/gamma_f.png\"/>
</p>

<p>
This is a strict consequence of the elctro magnetic coupling between the quasi stationary electric and the quasi stationary magnetic domain.</p>


<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig. 1:</b> Reference frames of the quasi stationary fundamental wave library</caption>
  <tr>
    <td>
      <img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/ReferenceFrames.png\"/>
    </td>
  </tr>
</table>

<p>
The complex magnetic flux with respect a stator and rotor magnetic port are equal, 
</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/Phi(ref)=Phi,re+jPhi,im.png\"/>,
</p>

<p>
but the reference phase angles are different according to the relationship explained above. The stator and rotor reference angles refer to quasi stationary magnetic connectors. The complex magnetic flux of the (stator) port with respect to the <b>stator fixed</b> reference frame is then calculated by</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/Phi_s_ref.png\"/>.
</p>

<p>
The complex magnetic flux of the (rotor) magnetic port with respect to the <b>rotor fixed</b> reference frame is then calculated by</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/Phi_r_ref.png\"/>.
</p>

<p>
The two stator and rotor fixed complex fluxes are related by</p>

<p>
<img src=\"modelica://QuasiStationaryFundamentalWave/Resources/Images/Phi_r_s.png\"/>.
</p>

</html>"));
    end Concept;

    class Contact "Contact"
      extends Modelica.Icons.Contact;
      annotation (Documentation(info="<html>
<h4>Contact</h4>

<p>
Dr. Christian Kral<br>
1060 Vienna, Austria<br>
email: <a HREF=\"mailto:dr.christian.kral@gmail.com\">dr.christian.kral@gmail.com</a><br>
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
<h5>Version 0.1.0, 2013-08-27</h5>
<ul>
<li>Documentation of 
<a href=\"modelica://QuasiStationaryFundamentalWave.UsersGuide.Concept\">phasor concept</a></li>
<li>Connections.branch between electric and magnetic quasi stationary connectors to handle open circuit and motor operation of machines</li>
<li>Saliency effects are properly considered</li>
<li>Electromagnetic coupling with Analog domain is implemented fully quasi stationary with v = 0 at the electric connectors -- this may have to be changed in the future</li>
<li>Implemented machine types</li>
<ul>
  <li>Asynchronous induction machine with squirrel cage</li>
  <li>Permanent magnet synchronous machine with optional damper cage</li>
  <li>Electrical excited synchronous machine with optional damper cage (may be removed in first release)</li>
  <li>Synchronous reluctance machine with optional damper cage (may be removed in first release)</li>
</ul>
</ul>

<h5>Version 0.2.0, 2013-09-01</h5>
<ul>
<li>Implemented asynchronous induction machine with slip ring rotor including example</li>
<li>Implemented magnetic crossing</li>
</ul>

</html>"));
    end ReleaseNotes;

    class References "References"
      extends Modelica.Icons.References;
      annotation (Documentation(info="<html>

</html>"));
    end References;
    annotation (Documentation(info="<html>
<p>
This library on quasi stationary fundamental wave models for the application in three phase phase machines is currently under development.
</p>

</html>"));
  end UsersGuide;


  package Examples "Examples"
  extends Modelica.Icons.ExamplesPackage;

    package Components
    "Examples for testing quasi stationary fundamental wave components"
      extends Modelica.Icons.ExamplesPackage;

      model ThreePhaseInductance "Three phase inductance"
      import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        constant Integer m = 3 "Number of phases";
        parameter Modelica.SIunits.Frequency f = 1 "Supply frequency";
        parameter Modelica.SIunits.Voltage VRMS = 100 "RMS supply voltage";
        parameter Modelica.SIunits.Resistance R = 1E-5 "Resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load inductance";
        parameter Real effectiveTurns = 5 "Effective number of turns";
        // Symmetrical multi phase magnetic reluctance
        final parameter Modelica.SIunits.Reluctance R_m = effectiveTurns^2/L
        "Equivalent magnetic reluctance of the positive symmetrical component";
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_e
          annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_m
          annotation (Placement(transformation(extent={{-70,-90},{-50,-70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star star_e(m=m)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,40})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star star_m(m=m)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSource_e(
          m=m,
          f=f,
        phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
        V=fill(VRMS, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,70})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSource_m(
          m=m,
          f=f,
        phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
        V=fill(VRMS, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m, R_ref=
            fill(R*m/3, m))
        annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m, R_ref=
            fill(R*m/3, m))
        annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor inductor_e(m=m, L=
              fill(L, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,70})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
                                                                          converter_m(
          m=m,
          effectiveTurns=effectiveTurns)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_m(R_m(d=R_m,
              q=R_m))
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,-30})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_m
          annotation (Placement(transformation(extent={{10,-90},{30,-70}})));
      equation
        connect(star_e.pin_n, ground_e.pin)
                                          annotation (Line(
            points={{-60,30},{-60,30}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_e.plug_p, voltageSource_e.plug_n)
                                                     annotation (Line(
            points={{-60,50},{-60,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.plug_n, inductor_e.plug_n)
                                                        annotation (Line(
            points={{-60,60},{-1.33227e-15,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_m.port_p, reluctance_m.port_p)
                                                       annotation (Line(
            points={{20,-20},{60,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_n, reluctance_m.port_n)
                                                       annotation (Line(
            points={{20,-40},{60,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_n, groundM_m.port_p)
                                                    annotation (Line(
            points={{20,-40},{20,-70}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(voltageSource_m.plug_n, star_m.plug_p)
                                                     annotation (Line(
            points={{-60,-40},{-60,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_m.pin_n, ground_m.pin)
                                          annotation (Line(
            points={{-60,-70},{-60,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_m.plug_n, converter_m.plug_n)
                                                          annotation (Line(
            points={{-60,-40},{-4.44089e-16,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.plug_p, resistor_e.plug_p)
                                                      annotation (Line(
          points={{-60,80},{-40,80}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(resistor_e.plug_n, inductor_e.plug_p)
                                                annotation (Line(
          points={{-20,80},{2.66454e-15,80}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(voltageSource_m.plug_p, resistor_m.plug_p)
                                                       annotation (Line(
          points={{-60,-20},{-40,-20}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(resistor_m.plug_n, converter_m.plug_p)
                                                   annotation (Line(
          points={{-20,-20},{-4.44089e-16,-20}},
          color={85,170,255},
          smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),graphics),
          experiment(StopTime=100, Interval=0.01));
      end ThreePhaseInductance;

      model MultiPhaseInductance "Multi phase inductance"
      import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Integer m = 3 "Number of phases";
        parameter Modelica.SIunits.Frequency f = 1 "Supply frequency";
        parameter Modelica.SIunits.Voltage VRMS = 100 "RMS supply voltage";
        parameter Modelica.SIunits.Resistance R = 1E-5 "Resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load inductance";
        parameter Real effectiveTurns = 5 "Effective number of turns";
        // Symmetrical multi phase magnetic reluctance
        final parameter Modelica.SIunits.Reluctance R_m = m*effectiveTurns^2/2/L
        "Equivalent magnetic reluctance";
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_e
          annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_m
          annotation (Placement(transformation(extent={{-70,-90},{-50,-70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star star_e(m=m)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,40})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star star_m(m=m)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSource_e(
          m=m,
          f=f,
        phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
        V=fill(VRMS, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,70})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSource_m(
          m=m,
          f=f,
        phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
        V=fill(VRMS, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m, R_ref=
            fill(R*m/3, m))
        annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m, R_ref=
            fill(R*m/3, m))
        annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor inductor_e(m=m, L=
              fill(L, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,70})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
                                                                          converter_m(
          m=m, effectiveTurns=effectiveTurns)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_m(R_m(d=R_m,
              q=R_m))
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,-30})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_m
          annotation (Placement(transformation(extent={{10,-90},{30,-70}})));
      equation
        connect(star_e.pin_n, ground_e.pin)
                                          annotation (Line(
            points={{-60,30},{-60,30}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_e.plug_p, voltageSource_e.plug_n)
                                                     annotation (Line(
            points={{-60,50},{-60,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.plug_n, inductor_e.plug_n)
                                                        annotation (Line(
            points={{-60,60},{-1.33227e-15,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_m.port_p, reluctance_m.port_p)
                                                       annotation (Line(
            points={{20,-20},{60,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_n, reluctance_m.port_n)
                                                       annotation (Line(
            points={{20,-40},{60,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_n, groundM_m.port_p)
                                                    annotation (Line(
            points={{20,-40},{20,-70}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(voltageSource_m.plug_n, star_m.plug_p)
                                                     annotation (Line(
            points={{-60,-40},{-60,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_m.pin_n, ground_m.pin)
                                          annotation (Line(
            points={{-60,-70},{-60,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_m.plug_n, converter_m.plug_n)
                                                          annotation (Line(
            points={{-60,-40},{-4.44089e-16,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.plug_p, resistor_e.plug_p)
                                                      annotation (Line(
          points={{-60,80},{-40,80}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(resistor_e.plug_n, inductor_e.plug_p)
                                                annotation (Line(
          points={{-20,80},{2.66454e-15,80}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(voltageSource_m.plug_p, resistor_m.plug_p)
                                                       annotation (Line(
          points={{-60,-20},{-40,-20}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(resistor_m.plug_n, converter_m.plug_p)
                                                   annotation (Line(
          points={{-20,-20},{-4.44089e-16,-20}},
          color={85,170,255},
          smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),graphics),
          experiment(StopTime=100, Interval=0.01));
      end MultiPhaseInductance;

      model EddyCurrentLosses
      "Comparison of equivalent circuits of eddy current loss models"
      import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        constant Integer m=3 "Number of phases";
        // ## Original value R = 0.1 Ohm
        parameter Modelica.SIunits.Resistance R=0.1 "Resistance";
        parameter Modelica.SIunits.Conductance Gc=0.0001 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";
        output Modelica.SIunits.Power lossPower_e=sum(loss_e.conductor.LossPower);
        output Modelica.SIunits.Power lossPower_m=loss_m.lossPower;
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_e
          annotation (Placement(transformation(extent={{-90,0},{-70,20}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_m
          annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_e(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_m(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_e(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_m(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m, R_ref=
            fill(R*m/3, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,70})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m, R_ref=
            fill(R*m/3, m))               annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,-20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_e(m=m)
          annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_m(m=m)
          annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Conductor loss_e(
          m=m, G_ref=fill(Gc*3/m, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,60})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_e(
          effectiveTurns=N)
          annotation (Placement(transformation(extent={{20,50},{40,70}})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_m(
          effectiveTurns=N)
          annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
        QuasiStationaryFundamentalWave.Components.EddyCurrent loss_m(G=m*N^2*Gc/2)
                         annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={60,-20})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_e(
            R_m(d=R_m, q=R_m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,60})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_m(
            R_m(d=R_m, q=R_m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,-30})));
        QuasiStationaryFundamentalWave.Components.Ground mground_e
          annotation (Placement(transformation(extent={{30,0},{50,20}})));
        QuasiStationaryFundamentalWave.Components.Ground mground_m
          annotation (Placement(transformation(extent={{30,-90},{50,-70}})));
      equation
        connect(sineVoltage_e.plug_n, converter_e.plug_n) annotation (Line(
            points={{-80,50},{20,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_e.plug_n, star_e.plug_p) annotation (Line(
            points={{-80,50},{-80,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_e.port_p, reluctance_e.port_p) annotation (Line(
            points={{40,70},{80,70}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_e.port_n, reluctance_e.port_n) annotation (Line(
            points={{40,50},{80,50}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_e.port_n, mground_e.port_p) annotation (Line(
            points={{40,50},{40,20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(resistor_e.plug_p, sineVoltage_e.plug_p)
                                                       annotation (Line(
            points={{-70,70},{-80,70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(loss_e.plug_n, sineVoltage_e.plug_n) annotation (Line(
            points={{-1.33227e-15,50},{-80,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(loss_e.plug_p, converter_e.plug_p) annotation (Line(
            points={{2.66454e-15,70},{20,70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_e.plug_n, powerb_e.currentP)
                                                    annotation (Line(
            points={{-50,70},{-40,70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_e.currentN, converter_e.plug_p) annotation (Line(
            points={{-20,70},{20,70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_e.pin_n, ground_e.pin) annotation (Line(
            points={{-80,20},{-80,20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_e.currentP, powerb_e.voltageP) annotation (Line(
            points={{-40,70},{-40,80},{-30,80}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_e.voltageN, sineVoltage_e.plug_n) annotation (Line(
            points={{-30,60},{-30,50},{-80,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_m.plug_n,star_m. plug_p) annotation (Line(
            points={{-80,-40},{-80,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_m.plug_n,converter_m. plug_n) annotation (Line(
            points={{-80,-40},{20,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_m.port_n,reluctance_m. port_n) annotation (Line(
            points={{40,-40},{80,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_p,loss_m. port_p) annotation (Line(
            points={{40,-20},{50,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(loss_m.port_n,reluctance_m. port_p) annotation (Line(
            points={{70,-20},{80,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_m.port_n,mground_m. port_p) annotation (Line(
            points={{40,-40},{40,-70}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(sineVoltage_m.plug_p,resistor_m. plug_p)
                                                       annotation (Line(
            points={{-80,-20},{-70,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_m.plug_n,powerb_m. currentP)
                                                    annotation (Line(
            points={{-50,-20},{-40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_m.currentN,converter_m. plug_p) annotation (Line(
            points={{-20,-20},{20,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_m.pin_n,ground_m. pin) annotation (Line(
            points={{-80,-70},{-80,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_m.currentP,powerb_m. voltageP) annotation (Line(
            points={{-40,-20},{-40,-10},{-30,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_m.voltageN,sineVoltage_m. plug_n) annotation (Line(
            points={{-30,-30},{-30,-40},{-80,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (experiment(StopTime=40, Interval=0.01), Documentation(info="<html>
<p>
In this example the eddy current losses are implemented in two different ways. Compare the loss dissipation <code>powerb_e.power</code> and <code>powerb_m.power</code> of the two models indicated by power meters.</p>
</html>"),Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
                preserveAspectRatio=false), graphics),
          Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
      end EddyCurrentLosses;
    end Components;

    package ToBeRemovedLater
    "This package will be removed in the final version"
      extends Modelica.Icons.ExamplesPackage;
      model Example1
        extends Modelica.Icons.Example;
        Modelica.Magnetic.FundamentalWave.Components.Ground ground_t
          annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
        QuasiStationaryFundamentalWave.Components.Ground ground_f
          annotation (Placement(transformation(extent={{-30,-76},{-10,-56}})));
        Modelica.Magnetic.FundamentalWave.Sources.SignalFlux constantFlux_t
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,50})));
        Modelica.ComplexBlocks.Sources.ComplexRotatingPhasor complexRotatingPhasor(
          magnitude=sqrt(2),
          w=2*Modelica.Constants.pi,
          phi0=0)
          annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
        QuasiStationaryFundamentalWave.Sources.ConstantFlux constantFlux_f(f=1, Phi=
              Complex(re=sqrt(2), im=0))           annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,-30})));
        Modelica.Magnetic.FundamentalWave.Components.EddyCurrent eddyCurrent_t(G=1)
          annotation (Placement(transformation(extent={{0,50},{20,70}})));
        QuasiStationaryFundamentalWave.Components.EddyCurrent eddyCurrent_f(G=1)
          annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
      equation
        connect(constantFlux_f.port_n, eddyCurrent_f.port_p) annotation (Line(
            points={{-20,-20},{0,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(eddyCurrent_f.port_n, constantFlux_f.port_p) annotation (Line(
            points={{20,-20},{20,-40},{-20,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantFlux_f.port_p, ground_f.port_p) annotation (Line(
            points={{-20,-40},{-20,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantFlux_t.port_p, ground_t.port_p) annotation (Line(
            points={{-20,40},{-20,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(constantFlux_t.port_n, eddyCurrent_t.port_p) annotation (Line(
            points={{-20,60},{0,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(eddyCurrent_t.port_n, constantFlux_t.port_p) annotation (Line(
            points={{20,60},{20,40},{-20,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(complexRotatingPhasor.y, constantFlux_t.Phi) annotation (Line(
            points={{-39,50},{-30,50}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example1;

      model Example2
        extends Modelica.Icons.Example;
        Modelica.Magnetic.FundamentalWave.Components.Ground ground_t
          annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
        QuasiStationaryFundamentalWave.Components.Ground ground_f
          annotation (Placement(transformation(extent={{-30,-76},{-10,-56}})));
        Modelica.Magnetic.FundamentalWave.Sources.SignalFlux constantFlux_t
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,50})));
        Modelica.ComplexBlocks.Sources.ComplexRotatingPhasor complexRotatingPhasor(
          magnitude=sqrt(2),
          w=2*Modelica.Constants.pi,
          phi0=0)
          annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
        QuasiStationaryFundamentalWave.Sources.ConstantFlux constantFlux_f(f=1, Phi=
              Complex(re=sqrt(2), im=0))           annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,-30})));
        Modelica.Magnetic.FundamentalWave.Components.EddyCurrent eddyCurrent_t(G=1)
          annotation (Placement(transformation(extent={{0,50},{20,70}})));
        QuasiStationaryFundamentalWave.Components.EddyCurrent eddyCurrent_f(G=1)
          annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.Reluctance reluctance_t(R_m(d=1,
              q=1)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,50})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_f(R_m(d=1, q=1))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={40,-30})));
      equation
        connect(constantFlux_f.port_n, eddyCurrent_f.port_p) annotation (Line(
            points={{-20,-20},{0,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantFlux_f.port_p, ground_f.port_p) annotation (Line(
            points={{-20,-40},{-20,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantFlux_t.port_p, ground_t.port_p) annotation (Line(
            points={{-20,40},{-20,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(constantFlux_t.port_n, eddyCurrent_t.port_p) annotation (Line(
            points={{-20,60},{0,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(complexRotatingPhasor.y, constantFlux_t.Phi) annotation (Line(
            points={{-39,50},{-30,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(eddyCurrent_f.port_n, reluctance_f.port_p) annotation (Line(
            points={{20,-20},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantFlux_f.port_p, reluctance_f.port_n) annotation (Line(
            points={{-20,-40},{40,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(eddyCurrent_t.port_n, reluctance_t.port_p) annotation (Line(
            points={{20,60},{40,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(constantFlux_t.port_p, reluctance_t.port_n) annotation (Line(
            points={{-20,40},{40,40}},
            color={255,128,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example2;

      model Example3
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        constant Integer m=1 "Number of phases";
        // ## Original value R = 0.1 Ohm
        parameter Modelica.SIunits.Resistance R=10 "Resistance";
        parameter Modelica.SIunits.Conductance Gc=1 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";
        Modelica.Electrical.Analog.Basic.Ground ground_t
          annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_f
          annotation (Placement(transformation(extent={{-98,-90},{-78,-70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star_t(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_f(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,-60})));
        Modelica.Electrical.MultiPhase.Sources.CosineVoltage
                                                           sineVoltage_t(
          m=m,
          V=fill(1, m),
          freqHz=fill(1, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) +
              fill(pi/2, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_f(
          m=m,
          f=1,
          V=fill(1/sqrt(2), m),
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) +
              fill(Modelica.Constants.pi/2, m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-88,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_t(m=m, R=fill(R*
              m/3, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-68,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_f(m=m, R_ref=
              fill(R*m/3, m))             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-68,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerb_t(m=m)
          annotation (Placement(transformation(extent={{-48,50},{-28,70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_f(m=m)
          annotation (Placement(transformation(extent={{-48,-30},{-28,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_t(
          m=m,
          effectiveTurns=fill(N, m),
          orientation=Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(extent={{42,40},{62,60}})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_f(
          effectiveTurns=N, m=m)
          annotation (Placement(transformation(extent={{42,-40},{62,-20}})));
        Modelica.Magnetic.FundamentalWave.Components.Ground groundM_t
          annotation (Placement(transformation(extent={{52,0},{72,20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_f
          annotation (Placement(transformation(extent={{52,-76},{72,-56}})));
        Modelica.Magnetic.FundamentalWave.Components.Reluctance reluctance_t(R_m(d=1,
              q=1), V_m(re(start=0.05)))
                    annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={92,50})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_f(R_m(d=1, q=1))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={92,-30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor
          currentSensor_f(m=m)
          annotation (Placement(transformation(extent={{-18,-10},{2,-30}})));
        Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor_t(m=m)
          annotation (Placement(transformation(extent={{-18,70},{2,50}})));
        Modelica.Electrical.MultiPhase.Sensors.CurrentQuasiRMSSensor
          currentSensorRMS_t(m=m)
          annotation (Placement(transformation(extent={{12,70},{32,50}})));
        Modelica.ComplexBlocks.ComplexMath.ComplexToPolar currentSensorRMS_f[m]
          annotation (Placement(transformation(extent={{28,0},{48,20}})));
        Modelica.ComplexBlocks.ComplexMath.Gain gain[m](each k=Complex(sqrt(2), 0))
          annotation (Placement(transformation(extent={{-2,0},{18,20}})));
      equation
        connect(sineVoltage_t.plug_n, star_t.plug_p) annotation (Line(
            points={{-88,40},{-88,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star_t.pin_n, ground_t.p) annotation (Line(
            points={{-88,10},{-88,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, converter_t.plug_n) annotation (Line(
            points={{-88,40},{42,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_p, resistor_t.plug_p)
                                                       annotation (Line(
            points={{-88,60},{-78,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor_t.plug_n, powerb_t.pc)
                                              annotation (Line(
            points={{-58,60},{-48,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.pc, powerb_t.pv) annotation (Line(
            points={{-48,60},{-48,70},{-38,70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nv, sineVoltage_t.plug_n) annotation (Line(
            points={{-38,50},{-38,40},{-88,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(converter_t.port_n, reluctance_t.port_n) annotation (Line(
            points={{62,40},{92,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_t.port_n, groundM_t.port_p) annotation (Line(
            points={{62,40},{62,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,star_f. plug_p) annotation (Line(
            points={{-88,-40},{-88,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,converter_f. plug_n) annotation (Line(
            points={{-88,-40},{42,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_p,resistor_f. plug_p)
                                                       annotation (Line(
            points={{-88,-20},{-78,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_f.plug_n,powerb_f. currentP)
                                                    annotation (Line(
            points={{-58,-20},{-48,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_f.pin_n,ground_f. pin) annotation (Line(
            points={{-88,-70},{-88,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentP,powerb_f. voltageP) annotation (Line(
            points={{-48,-20},{-48,-10},{-38,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.voltageN,sineVoltage_f. plug_n) annotation (Line(
            points={{-38,-30},{-38,-40},{-88,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_f.port_n, reluctance_f.port_n) annotation (Line(
            points={{62,-40},{92,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, groundM_f.port_p) annotation (Line(
            points={{62,-40},{62,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_t.port_p, reluctance_t.port_p) annotation (Line(
            points={{62,60},{92,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_f.port_p, reluctance_f.port_p) annotation (Line(
            points={{62,-20},{92,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(powerb_f.currentN, currentSensor_f.plug_p) annotation (Line(
            points={{-28,-20},{-18,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensor_f.plug_n, converter_f.plug_p) annotation (Line(
            points={{2,-20},{42,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_t.nc, currentSensor_t.plug_p) annotation (Line(
            points={{-28,60},{-18,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor_t.plug_n, currentSensorRMS_t.plug_p) annotation (Line(
            points={{2,60},{12,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensorRMS_t.plug_n, converter_t.plug_p) annotation (Line(
            points={{32,60},{42,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(gain.y, currentSensorRMS_f.u) annotation (Line(
            points={{19,10},{26,10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensor_f.y, gain.u) annotation (Line(
            points={{-8,-9},{-8,10},{-4,10}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),   graphics),
          Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
      end Example3;

      model Example3a
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        constant Integer m=3 "Number of phases";
        // ## Original value R = 0.1 Ohm
        parameter Modelica.SIunits.Resistance R=1E-5 "Resistance";
        parameter Modelica.SIunits.Resistance Rq=1E4
        "Resistance for not causing problems with symmetrical components";
        parameter Modelica.SIunits.Conductance Gc=1 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";
        Modelica.Electrical.Analog.Basic.Ground ground_t
          annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_f
          annotation (Placement(transformation(extent={{-100,-90},{-80,-70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star_t(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-90,20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_f(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-90,-60})));
        Modelica.Electrical.MultiPhase.Sources.CosineCurrent
                                                           sineVoltage_t(
          m=m,
          freqHz=fill(1, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) +
              fill(pi/2, m),
          I=fill(1, m))      annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-90,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.CurrentSource
          sineVoltage_f(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) +
              fill(Modelica.Constants.pi/2, m),
          I=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-90,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_t(m=m, R=fill(R*
              m/3, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-70,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_f(m=m, R_ref=
              fill(R*m/3, m))             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-70,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerb_t(m=m)
          annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_f(m=m)
          annotation (Placement(transformation(extent={{-50,-30},{-30,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_t(
          m=m,
          effectiveTurns=fill(N, m),
          orientation=Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(extent={{40,40},{60,60}})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_f(
          effectiveTurns=N, m=m)
          annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
        Modelica.Magnetic.FundamentalWave.Components.Ground groundM_t
          annotation (Placement(transformation(extent={{50,0},{70,20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_f
          annotation (Placement(transformation(extent={{50,-76},{70,-56}})));
        Modelica.Magnetic.FundamentalWave.Components.Reluctance reluctance_t(R_m(d=1,
              q=1), V_m(re(start=0.05)))
                    annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={90,50})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_f(R_m(d=1, q=1))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={90,-30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor
          currentSensor_f(m=m)
          annotation (Placement(transformation(extent={{-20,-10},{0,-30}})));
        Modelica.Electrical.MultiPhase.Sensors.CurrentSensor currentSensor_t(m=m)
          annotation (Placement(transformation(extent={{-20,70},{0,50}})));
        Modelica.Electrical.MultiPhase.Sensors.CurrentQuasiRMSSensor
          currentSensorRMS_t(m=m)
          annotation (Placement(transformation(extent={{10,70},{30,50}})));
        Modelica.ComplexBlocks.ComplexMath.ComplexToPolar currentSensorRMS_f[3]
          annotation (Placement(transformation(extent={{0,0},{20,20}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistorq_f(
                                                                                 m=m, R_ref=
              fill(Rq*m/3, m))            annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_qt(
                                                                 m=m, R=fill(Rq
              *m/3, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,50})));
      equation
        connect(sineVoltage_t.plug_n, star_t.plug_p) annotation (Line(
            points={{-90,40},{-90,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star_t.pin_n, ground_t.p) annotation (Line(
            points={{-90,10},{-90,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, converter_t.plug_n) annotation (Line(
            points={{-90,40},{40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_p, resistor_t.plug_p)
                                                       annotation (Line(
            points={{-90,60},{-80,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor_t.plug_n, powerb_t.pc)
                                              annotation (Line(
            points={{-60,60},{-50,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.pc, powerb_t.pv) annotation (Line(
            points={{-50,60},{-50,70},{-40,70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nv, sineVoltage_t.plug_n) annotation (Line(
            points={{-40,50},{-40,40},{-90,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(converter_t.port_n, reluctance_t.port_n) annotation (Line(
            points={{60,40},{90,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_t.port_n, groundM_t.port_p) annotation (Line(
            points={{60,40},{60,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,star_f. plug_p) annotation (Line(
            points={{-90,-40},{-90,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,converter_f. plug_n) annotation (Line(
            points={{-90,-40},{40,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_p,resistor_f. plug_p)
                                                       annotation (Line(
            points={{-90,-20},{-80,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_f.plug_n,powerb_f. currentP)
                                                    annotation (Line(
            points={{-60,-20},{-50,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_f.pin_n,ground_f. pin) annotation (Line(
            points={{-90,-70},{-90,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentP,powerb_f. voltageP) annotation (Line(
            points={{-50,-20},{-50,-10},{-40,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.voltageN,sineVoltage_f. plug_n) annotation (Line(
            points={{-40,-30},{-40,-40},{-90,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_f.port_n, reluctance_f.port_n) annotation (Line(
            points={{60,-40},{90,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, groundM_f.port_p) annotation (Line(
            points={{60,-40},{60,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_t.port_p, reluctance_t.port_p) annotation (Line(
            points={{60,60},{90,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_f.port_p, reluctance_f.port_p) annotation (Line(
            points={{60,-20},{90,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(powerb_f.currentN, currentSensor_f.plug_p) annotation (Line(
            points={{-30,-20},{-20,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensor_f.plug_n, converter_f.plug_p) annotation (Line(
            points={{0,-20},{40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_t.nc, currentSensor_t.plug_p) annotation (Line(
            points={{-30,60},{-20,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor_t.plug_n, currentSensorRMS_t.plug_p) annotation (Line(
            points={{0,60},{10,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensorRMS_t.plug_n, converter_t.plug_p) annotation (Line(
            points={{30,60},{40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor_f.y, currentSensorRMS_f.u) annotation (Line(
            points={{-10,-9},{-10,10},{-2,10}},
            color={85,170,255},
            smooth=Smooth.None));
      connect(resistor_qt.plug_p, resistor_t.plug_p) annotation (Line(
          points={{-80,60},{-80,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(resistor_qt.plug_n, star_t.plug_p) annotation (Line(
          points={{-80,40},{-90,40},{-90,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(resistorq_f.plug_n, star_f.plug_p) annotation (Line(
          points={{-80,-40},{-90,-40},{-90,-50}},
          color={85,170,255},
          smooth=Smooth.None));
        connect(resistorq_f.plug_p, resistor_f.plug_p) annotation (Line(
            points={{-80,-20},{-80,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),   graphics),
          Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
      end Example3a;

      model Example4
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        constant Integer m=3 "Number of phases";
        parameter Modelica.SIunits.Resistance R=10 "Resistance";
        parameter Modelica.SIunits.Conductance Gc=1 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";
        Modelica.Electrical.Analog.Basic.Ground ground_t
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_f
          annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star_t(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_f(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-60})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_t(
          m=m,
          V=fill(1, m),
          freqHz=fill(1, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_f(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_t(m=m, R=fill(R*m/3, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_f(m=m, R_ref=
              fill(R*m/3, m))             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerb_t(m=m)
          annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_f(m=m)
          annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_t(
          m=m,
          effectiveTurns=fill(N, m),
          orientation=Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(extent={{0,40},{20,60}})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_f(           final effectiveTurns=N, m=m)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        Modelica.Magnetic.FundamentalWave.Components.Ground groundM_t
          annotation (Placement(transformation(extent={{10,0},{30,20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_f
          annotation (Placement(transformation(extent={{10,-76},{30,-56}})));
        Modelica.Magnetic.FundamentalWave.Components.EddyCurrent eddyCurrent_t(G=1)
          annotation (Placement(transformation(extent={{40,50},{60,70}})));
        QuasiStationaryFundamentalWave.Components.EddyCurrent eddyCurrent_f(G=1)
          annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.Reluctance reluctance_t(R_m(d=1,
              q=1)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,50})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_f(R_m(d=1, q=1))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,-30})));
      equation
        connect(eddyCurrent_t.port_n, reluctance_t.port_p) annotation (Line(
            points={{60,60},{80,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, star_t.plug_p) annotation (Line(
            points={{-80,40},{-80,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star_t.pin_n, ground_t.p) annotation (Line(
            points={{-80,10},{-80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, converter_t.plug_n) annotation (Line(
            points={{-80,40},{0,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_p, resistor_t.plug_p)
                                                       annotation (Line(
            points={{-80,60},{-70,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor_t.plug_n, powerb_t.pc)
                                              annotation (Line(
            points={{-50,60},{-40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.pc, powerb_t.pv) annotation (Line(
            points={{-40,60},{-40,70},{-30,70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nc, converter_t.plug_p) annotation (Line(
            points={{-20,60},{0,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nv, sineVoltage_t.plug_n) annotation (Line(
            points={{-30,50},{-30,40},{-80,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(converter_t.port_n, reluctance_t.port_n) annotation (Line(
            points={{20,40},{80,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_t.port_n, groundM_t.port_p) annotation (Line(
            points={{20,40},{20,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(eddyCurrent_f.port_n, reluctance_f.port_p) annotation (Line(
            points={{60,-20},{80,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,star_f. plug_p) annotation (Line(
            points={{-80,-40},{-80,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,converter_f. plug_n) annotation (Line(
            points={{-80,-40},{0,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_p,resistor_f. plug_p)
                                                       annotation (Line(
            points={{-80,-20},{-70,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_f.plug_n,powerb_f. currentP)
                                                    annotation (Line(
            points={{-50,-20},{-40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentN,converter_f. plug_p) annotation (Line(
            points={{-20,-20},{0,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_f.pin_n,ground_f. pin) annotation (Line(
            points={{-80,-70},{-80,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentP,powerb_f. voltageP) annotation (Line(
            points={{-40,-20},{-40,-10},{-30,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.voltageN,sineVoltage_f. plug_n) annotation (Line(
            points={{-30,-30},{-30,-40},{-80,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_f.port_p, eddyCurrent_f.port_p) annotation (Line(
            points={{20,-20},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, reluctance_f.port_n) annotation (Line(
            points={{20,-40},{80,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, groundM_f.port_p) annotation (Line(
            points={{20,-40},{20,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_t.port_p, eddyCurrent_t.port_p) annotation (Line(
            points={{20,60},{40,60}},
            color={255,128,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),   graphics),
          Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
      end Example4;

      model Example_AirGap1
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Frequency f = 1 "Supply frequency";
        Sources.ConstantMagneticPotentialDifference
          constantMagneticPotentialDifference(f=f, V_m=Complex(re=0, im=1))
                                                   annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-40,0})));
        QuasiStationaryFundamentalWave.Components.Ground grounds
          annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundr
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.RotorSaliencyAirGap
          rotorSaliencyAirGap(p=1, L0(d=1, q=1))
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixeds
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2
              *Modelica.Constants.pi*f/3)
          annotation (Placement(transformation(extent={{-20,30},{0,50}})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance(R_m(d=1,
              q=1))
          annotation (Placement(transformation(extent={{20,0},{40,20}})));
      equation
        connect(rotorSaliencyAirGap.port_rn, groundr.port_p) annotation (Line(
            points={{10,-10},{40,-10},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.support, fixeds.flange)
                                                           annotation (Line(
            points={{0,-10},{0,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, grounds.port_p)
          annotation (Line(
            points={{-40,-10},{-40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, rotorSaliencyAirGap.port_sp)
          annotation (Line(
            points={{-40,-10},{-10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_n, rotorSaliencyAirGap.port_sn)
          annotation (Line(
            points={{-40,10},{-10,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantSpeed.flange, rotorSaliencyAirGap.flange_a) annotation (Line(
            points={{0,40},{4.44089e-16,40},{4.44089e-16,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.port_rp, reluctance.port_p) annotation (
            Line(
            points={{10,10},{20,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(reluctance.port_n, rotorSaliencyAirGap.port_rn) annotation (
            Line(
            points={{40,10},{40,-10},{10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example_AirGap1;

      model Example_AirGap2
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Frequency fs = 1 "Stator supply frequency";
        parameter Modelica.SIunits.Frequency fr = 2/3 "Rotor supply frequency";
        Sources.ConstantMagneticPotentialDifference
          constantMagneticPotentialDifference(     V_m=Complex(re=0, im=1), f=fs)
                                                   annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-40,0})));
        QuasiStationaryFundamentalWave.Components.Ground grounds
          annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundr
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.RotorSaliencyAirGap
          rotorSaliencyAirGap(p=1, L0(d=1, q=1))
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixeds
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2
              *Modelica.Constants.pi*(fs - fr))
          annotation (Placement(transformation(extent={{-20,30},{0,50}})));
        QuasiStationaryFundamentalWave.Sources.ConstantFlux  reluctance(Phi=Complex(
              re=1, im=1), f=fr)
          annotation (Placement(transformation(extent={{20,0},{40,20}})));
      equation
        connect(rotorSaliencyAirGap.port_rn, groundr.port_p) annotation (Line(
            points={{10,-10},{40,-10},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.support, fixeds.flange)
                                                           annotation (Line(
            points={{0,-10},{0,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, grounds.port_p)
          annotation (Line(
            points={{-40,-10},{-40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, rotorSaliencyAirGap.port_sp)
          annotation (Line(
            points={{-40,-10},{-10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_n, rotorSaliencyAirGap.port_sn)
          annotation (Line(
            points={{-40,10},{-10,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantSpeed.flange, rotorSaliencyAirGap.flange_a) annotation (Line(
            points={{0,40},{4.44089e-16,40},{4.44089e-16,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.port_rp, reluctance.port_p) annotation (Line(
            points={{10,10},{20,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(reluctance.port_n, rotorSaliencyAirGap.port_rn) annotation (Line(
            points={{40,10},{40,-10},{10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example_AirGap2;

      model Example_AirGap3
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Frequency fs = 0 "Stator supply frequency";
        parameter Modelica.SIunits.Frequency fr = 0 "Rotor supply frequency";
        Modelica.SIunits.ComplexCurrent is[3]={Complex(0,0) for k in 1:3};
        Sources.ConstantMagneticPotentialDifference
          constantMagneticPotentialDifference(     V_m=Complex(re=1, im=0), f=fs)
                                                   annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-40,0})));
        QuasiStationaryFundamentalWave.Components.Ground grounds
          annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundr
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.RotorSaliencyAirGap
          rotorSaliencyAirGap(p=1, L0(d=1, q=1))
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.PermanentMagnet
                                                                                permanentMagnet(
            is=is,
            permanentMagnetLossParameters(IRef=1, wRef=0.10471975511966))
          annotation (Placement(transformation(extent={{20,0},{40,20}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixeds
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixedr
          annotation (Placement(transformation(extent={{-10,40},{10,20}})));
      equation
        connect(rotorSaliencyAirGap.port_rn, groundr.port_p) annotation (Line(
            points={{10,-10},{40,-10},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.port_rp, permanentMagnet.port_p)
                                                                annotation (Line(
            points={{10,10},{20,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(permanentMagnet.port_n, rotorSaliencyAirGap.port_rn)
                                                                annotation (Line(
            points={{40,10},{40,-10},{10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.support, fixeds.flange)
                                                           annotation (Line(
            points={{4.44089e-16,-10},{4.44089e-16,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, grounds.port_p)
          annotation (Line(
            points={{-40,-10},{-40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, rotorSaliencyAirGap.port_sp)
          annotation (Line(
            points={{-40,-10},{-10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_n, rotorSaliencyAirGap.port_sn)
          annotation (Line(
            points={{-40,10},{-10,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(permanentMagnet.support, fixeds.flange) annotation (Line(
            points={{30,-4.44089e-16},{30,-20},{4.44089e-16,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.flange_a, permanentMagnet.flange) annotation (
            Line(
            points={{6.66134e-16,10},{0,10},{0,20},{30,20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(fixedr.flange, rotorSaliencyAirGap.flange_a) annotation (Line(
            points={{0,30},{0,10}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example_AirGap3;

      model Example_AirGap4
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Frequency fs = 1 "Stator supply frequency";
        parameter Modelica.SIunits.Frequency fr = 1e-5 "Rotor supply frequency";
        Modelica.SIunits.ComplexCurrent is[3]={Complex(0,0) for k in 1:3};
        Sources.ConstantMagneticPotentialDifference
          constantMagneticPotentialDifference(     V_m=Complex(re=1, im=0), f=fs)
                                                   annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-40,0})));
        QuasiStationaryFundamentalWave.Components.Ground grounds
          annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundr
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.RotorSaliencyAirGap
          rotorSaliencyAirGap(p=1, L0(d=1, q=1))
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.PermanentMagnet
                                                                                permanentMagnet(
            is=is,
            permanentMagnetLossParameters(IRef=1, wRef=0.10471975511966))
          annotation (Placement(transformation(extent={{20,0},{40,20}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixeds
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2
              *Modelica.Constants.pi*(fs - fr))
          annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
      equation
        connect(rotorSaliencyAirGap.port_rn, groundr.port_p) annotation (Line(
            points={{10,-10},{40,-10},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.port_rp, permanentMagnet.port_p)
                                                                annotation (Line(
            points={{10,10},{20,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(permanentMagnet.port_n, rotorSaliencyAirGap.port_rn)
                                                                annotation (Line(
            points={{40,10},{40,-10},{10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.support, fixeds.flange)
                                                           annotation (Line(
            points={{4.44089e-16,-10},{4.44089e-16,-16},{0,-16},{0,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, grounds.port_p)
          annotation (Line(
            points={{-40,-10},{-40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, rotorSaliencyAirGap.port_sp)
          annotation (Line(
            points={{-40,-10},{-10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_n, rotorSaliencyAirGap.port_sn)
          annotation (Line(
            points={{-40,10},{-10,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(permanentMagnet.support, fixeds.flange) annotation (Line(
            points={{30,-4.44089e-16},{30,-20},{0,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(permanentMagnet.flange, rotorSaliencyAirGap.flange_a) annotation (
            Line(
            points={{30,20},{6.66134e-16,20},{6.66134e-16,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantSpeed.flange, rotorSaliencyAirGap.flange_a) annotation (Line(
            points={{-20,50},{0,50},{0,10},{6.66134e-16,10}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example_AirGap4;

      model Example_AirGap5
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        parameter Modelica.SIunits.Frequency fs = 1 "Stator supply frequency";
        parameter Modelica.SIunits.Frequency fr = 0.1 "Rotor supply frequency";
        Modelica.SIunits.ComplexCurrent is[3]={Complex(0,0) for k in 1:3};
        Sources.ConstantMagneticPotentialDifference
          constantMagneticPotentialDifference(     V_m=Complex(re=1, im=0), f=fs)
                                                   annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-40,0})));
        QuasiStationaryFundamentalWave.Components.Ground grounds
          annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundr
          annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.RotorSaliencyAirGap
          rotorSaliencyAirGap(p=1, L0(d=1, q=1))
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.QuasiStionaryAnalogWinding
                                                                                permanentMagnet(
          RRef=1,
          TRef=293.15,
          alpha20(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
          TOperational=293.15)
          annotation (Placement(transformation(extent={{60,-10},{40,10}})));
        Modelica.Mechanics.Rotational.Components.Fixed fixeds
          annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=2
              *Modelica.Constants.pi*(fs - fr))
          annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
        Modelica.Electrical.Analog.Sources.ConstantCurrent constantCurrent
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={80,0})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{70,-40},{90,-20}})));
      equation
        connect(rotorSaliencyAirGap.port_rn, groundr.port_p) annotation (Line(
            points={{10,-10},{40,-10},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.port_rp, permanentMagnet.port_p)
                                                                annotation (Line(
            points={{10,10},{40,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(permanentMagnet.port_n, rotorSaliencyAirGap.port_rn)
                                                                annotation (Line(
            points={{40,-10},{10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(rotorSaliencyAirGap.support, fixeds.flange)
                                                           annotation (Line(
            points={{4.44089e-16,-10},{4.44089e-16,-16},{0,-16},{0,-20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, grounds.port_p)
          annotation (Line(
            points={{-40,-10},{-40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_p, rotorSaliencyAirGap.port_sp)
          annotation (Line(
            points={{-40,-10},{-10,-10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantMagneticPotentialDifference.port_n, rotorSaliencyAirGap.port_sn)
          annotation (Line(
            points={{-40,10},{-10,10}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(constantSpeed.flange, rotorSaliencyAirGap.flange_a) annotation (Line(
            points={{-20,50},{0,50},{0,10},{6.66134e-16,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(permanentMagnet.pin_p, constantCurrent.n) annotation (Line(
            points={{60,10},{80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(permanentMagnet.pin_n, constantCurrent.p) annotation (Line(
            points={{60,-10},{80,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantCurrent.p, ground.p) annotation (Line(
            points={{80,-10},{80,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end Example_AirGap5;

      model TestQS "Series resonance circuit"
        extends Modelica.Icons.Example;
        Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource voltageSource1(
          f=1,
          V=1,
          phi=0)
          annotation (Placement(transformation(
              origin={-20,0},
              extent={{-10,10},{10,-10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground1
          annotation (Placement(transformation(extent={{-30,-40},{-10,-20}}, rotation=
                 0)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor(R_ref=1)
          annotation (Placement(transformation(extent={{-8,10},{12,30}},  rotation=0)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource voltageSource2(
          f=1,
          V=1,
          phi=0)
          annotation (Placement(transformation(
              origin={20,0},
              extent={{-10,10},{10,-10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground2
          annotation (Placement(transformation(extent={{10,-40},{30,-20}},   rotation=
                 0)));
      initial equation
        voltageSource1.pin_p.reference.gamma=0;
        voltageSource2.pin_p.reference.gamma=0;

      equation
        connect(ground1.pin, voltageSource1.pin_n)
                                                 annotation (Line(points={{-20,-20},{-20,
                -15},{-20,-10}},           color={85,170,255}));
        connect(ground2.pin, voltageSource2.pin_n)
                                                 annotation (Line(points={{20,-20},{20,
                -15},{20,-10}},            color={85,170,255}));
        connect(voltageSource1.pin_p, resistor.pin_p) annotation (Line(
            points={{-20,10},{-20,20},{-8,20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor.pin_n, voltageSource2.pin_p) annotation (Line(
            points={{12,20},{20,20},{20,10}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (
          experiment(StopTime=1.0, Interval=0.001),
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end TestQS;
    end ToBeRemovedLater;

    package BasicMachines "Examples of basic machines"
    extends Modelica.Icons.ExamplesPackage;
      model AIMC_Mains
      "Asynchronous induction machine with squirrel cage operated at mains"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of phases";
        parameter Modelica.SIunits.Voltage VsNominal=100
        "Nominal RMS voltage per phase";
        parameter Modelica.SIunits.Frequency fNominal=50 "Nominal frequency";
        parameter Modelica.SIunits.Time tOn=0.1 "Start time of machine";
        parameter Modelica.SIunits.Torque T_Load=161.4 "Nominal load torque";
        parameter Modelica.SIunits.AngularVelocity w_Load(displayUnit="1/min")=
             1440.45*2*Modelica.Constants.pi/60 "Nominal load speed";
        parameter Modelica.SIunits.Inertia J_Load=0.5 "Load inertia";
        parameter Integer p=2 "Number of pole pairs";
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSourceQS(
          m=m,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          f=fNominal,
          V=fill(VsNominal, m))
          annotation (Placement(transformation(
              origin={-60,40},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(m=m)
          annotation (Placement(transformation(
              origin={-80,10},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-100,10})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor powerSensorQS(m=m)
          annotation (Placement(transformation(extent={{-60,80},{-40,100}},
                                                                          rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor currentSensorQS(m=m)
          annotation (Placement(transformation(extent={{-30,80},{-10,100}},
                                                                          rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Ideal.IdealClosingSwitch
                                                                                idealCloserQS(
          final m=m,
          Ron=fill(1e-5*m/3, m),
          Goff=fill(1e-5*3/m, m))
                              annotation (Placement(transformation(
              origin={-60,70},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        Modelica.Blocks.Sources.BooleanStep booleanStepQS[m](each startTime=tOn,
            each startValue=false)
                              annotation (Placement(transformation(extent={{-100,60},{
                  -80,80}},      rotation=0)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              transformation(
              origin={-100,-80},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(extent={{-60,-90},{-80,-70}},rotation=0)));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          freqHz=fill(fNominal, m),
          V=fill(sqrt(2.0)*VsNominal, m))     annotation (Placement(
              transformation(
              origin={-60,-60},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.MultiPhase.Ideal.IdealClosingSwitch idealCloser(
          final m=m,
          Ron=fill(1e-5*m/3, m),
          Goff=fill(1e-5*3/m, m))
                              annotation (Placement(transformation(
              origin={-60,-30},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime=tOn, each
            startValue=false) annotation (Placement(transformation(extent={{-100,
                -40},{-80,-20}}, rotation=0)));
        Modelica.Electrical.MultiPhase.Sensors.CurrentQuasiRMSSensor
                                                                   currentRMSsensor(final m=m)
                            annotation (Placement(transformation(
              origin={-20,-10},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Machines.Utilities.TerminalBox terminalBoxM(m=m,
            terminalConnection="Y") annotation (Placement(transformation(extent={{20,-60},
                {40,-40}},            rotation=0)));
        Modelica.Magnetic.FundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
          aimc(
          Jr=aimcData.Jr,
          Js=aimcData.Js,
          p=aimcData.p,
          fsNominal=aimcData.fsNominal,
          TsRef=aimcData.TsRef,
          alpha20s(displayUnit="1/K") = aimcData.alpha20s,
          frictionParameters=aimcData.frictionParameters,
          statorCoreParameters=aimcData.statorCoreParameters,
          strayLoadParameters=aimcData.strayLoadParameters,
          alpha20r(displayUnit="1/K") = aimcData.alpha20r,
          phiMechanical(fixed=true),
          wMechanical(fixed=true),
          Rs=aimcData.Rs*m/3,
          Lssigma=aimcData.Lssigma*m/3,
          Lszero=aimcData.Lszero*m/3,
          Lm=aimcData.Lm*m/3,
          Lrsigma=aimcData.Lrsigma*m/3,
          Rr=aimcData.Rr*m/3,
          TrRef=aimcData.TrRef,
          m=m,
          TsOperational=293.15,
          TrOperational=293.15) annotation (Placement(transformation(extent={{20,-80},
                {40,-60}},        rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=J_Load)
          annotation (Placement(transformation(extent={{50,-80},{70,-60}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticLoadTorque(
          w_nominal=w_Load,
          tau_nominal=-T_Load,
          TorqueDirection=false,
          useSupport=false) annotation (Placement(transformation(extent={{100,-80},
                {80,-60}},   rotation=0)));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerSensor(final m=m)
          annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertiaQS(J=J_Load)
          annotation (Placement(transformation(extent={{50,20},{70,40}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque
          quadraticLoadTorqueQS(
          w_nominal=w_Load,
          tau_nominal=-T_Load,
          TorqueDirection=false,
          useSupport=false) annotation (Placement(transformation(extent={{100,20},
                {80,40}},    rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.AIM_SquirrelCageData
          aimcData
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        QuasiStationaryFundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
          aimcQS(
          Jr=aimcData.Jr,
          Js=aimcData.Js,
          p=aimcData.p,
          fsNominal=aimcData.fsNominal,
          TsRef=aimcData.TsRef,
          alpha20s(displayUnit="1/K") = aimcData.alpha20s,
          frictionParameters=aimcData.frictionParameters,
          statorCoreParameters=aimcData.statorCoreParameters,
          strayLoadParameters=aimcData.strayLoadParameters,
          alpha20r(displayUnit="1/K") = aimcData.alpha20r,
          Rs=aimcData.Rs*m/3,
          Lssigma=aimcData.Lssigma*m/3,
          Lm=aimcData.Lm*m/3,
          Lrsigma=aimcData.Lrsigma*m/3,
          Rr=aimcData.Rr*m/3,
          TrRef=aimcData.TrRef,
          m=m,
          wMechanical(fixed=true),
        TsOperational=293.15,
        gammar(fixed=true, start=pi/2),
        gamma(fixed=true, start=-pi/2),
        TrOperational=293.15)   annotation (Placement(transformation(extent={{20,20},
                {40,40}},         rotation=0)));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.SymmetricalComponents
          symmetricalComponents(m=m) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-20,60})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=0,
              origin={0,0})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={0,30})));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(m=m, terminalConnection="Y")
                                    annotation (Placement(transformation(extent={{20,40},
                {40,60}},              rotation=0)));
      initial equation
        aimc.is[1:m-1]=zeros(m-1);
        aimc.ir[1:m-1]=zeros(m-1);

      equation
        connect(groundQS.pin, starQS.pin_n)
                                        annotation (Line(points={{-90,10},{-92,10},{-90,
                10}},
              color={85,170,255}));
        connect(starQS.plug_p, voltageSourceQS.plug_n)
                                                   annotation (Line(points={{-70,10},{
                -60,10},{-60,30}},
                            color={85,170,255}));
        connect(powerSensorQS.currentN, currentSensorQS.plug_p)
          annotation (Line(points={{-40,90},{-30,90}}, color={85,170,255}));
        connect(powerSensorQS.voltageP, powerSensorQS.currentP)
                                                            annotation (Line(points={{-50,100},
              {-60,100},{-60,90}},           color={85,170,255}));
        connect(powerSensorQS.voltageN, starQS.plug_p)
                                                   annotation (Line(points={{-50,80},
              {-50,10},{-70,10}},                    color={85,170,255}));
        connect(booleanStepQS.y, idealCloserQS.control)
                                                    annotation (Line(
            points={{-79,70},{-67,70}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p)
          annotation (Line(points={{-80,-80},{-90,-80}},
                                                       color={0,0,255}));
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(points={{-60,-70},
              {-60,-80}},               color={0,0,255}));
        connect(aimc.flange, loadInertia.flange_a)   annotation (Line(points={{40,-70},
              {40,-70},{50,-70}},           color={0,0,0}));
        connect(loadInertia.flange_b, quadraticLoadTorque.flange)
          annotation (Line(points={{70,-70},{80,-70}}, color={0,0,0}));
        connect(terminalBoxM.plug_sn, aimc.plug_sn)
          annotation (Line(points={{24,-60},{24,-60}}, color={0,0,255}));
        connect(terminalBoxM.plug_sp, aimc.plug_sp)
          annotation (Line(points={{36,-60},{36,-60}},
                                                     color={0,0,255}));
        connect(booleanStep.y, idealCloser.control) annotation (Line(
            points={{-79,-30},{-67,-30}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(idealCloser.plug_p, sineVoltage.plug_p) annotation (Line(
            points={{-60,-40},{-60,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(idealCloserQS.plug_p, voltageSourceQS.plug_p) annotation (Line(
            points={{-60,60},{-60,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(idealCloserQS.plug_n, powerSensorQS.currentP) annotation (Line(
            points={{-60,80},{-60,90}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(idealCloser.plug_n, powerSensor.pc) annotation (Line(
            points={{-60,-20},{-60,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerSensor.nc, currentRMSsensor.plug_p) annotation (Line(
            points={{-40,-10},{-30,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentRMSsensor.plug_n, terminalBoxM.plugSupply) annotation (Line(
            points={{-10,-10},{30,-10},{30,-58}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(loadInertiaQS.flange_b, quadraticLoadTorqueQS.flange)
          annotation (Line(points={{70,30},{80,30}},   color={0,0,0}));
        connect(powerSensor.pv, powerSensor.pc) annotation (Line(
            points={{-50,0},{-60,0},{-60,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerSensor.nv, star.plug_p) annotation (Line(
            points={{-50,-20},{-50,-80},{-60,-80}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(aimcQS.flange, loadInertiaQS.flange_a)  annotation (Line(
            points={{40,30},{50,30}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(currentSensorQS.y, symmetricalComponents.u) annotation (Line(
            points={{-20,79},{-20,72}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sn, aimcQS.plug_sn) annotation (Line(
            points={{24,40},{24,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sp, aimcQS.plug_sp) annotation (Line(
            points={{36,40},{36,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
            points={{0,40},{0,42},{21,42}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensorQS.plug_n, terminalBoxQS.plugSupply) annotation (Line(
            points={{-10,90},{30,90},{30,42}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.pin_n, groundMachineQS.pin) annotation (Line(
            points={{0,20},{0,10}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment,
          __Dymola_experimentSetupOutput);
      end AIMC_Mains;

      model AIMS_Start
      "Starting of asynchronous induction machine with slip rings"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of phases";
        parameter Modelica.SIunits.Voltage VsNominal=100
        "Nominal RMS voltage per phase";
        parameter Modelica.SIunits.Frequency fNominal=50 "Nominal frequency";
        parameter Modelica.SIunits.Time tOn=0.1 "Start time of machine";
        parameter Modelica.SIunits.Resistance RStart=0.16/aimsData.turnsRatio^2
        "Starting resistance";
        parameter Modelica.SIunits.Time tRheostat=1.0
        "Time of shortening the rheostat";
        parameter Modelica.SIunits.Torque T_Load=161.4 "Nominal load torque";
        parameter Modelica.SIunits.AngularVelocity w_Load(displayUnit="1/min")=
             Modelica.SIunits.Conversions.from_rpm(1440.45)
        "Nominal load speed";
        parameter Modelica.SIunits.Inertia J_Load=0.29 "Load inertia";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              transformation(
              origin={-90,-80},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(extent={{10,-10},{-10,10}},  rotation=0,
              origin={-70,-80})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          freqHz=fill(fNominal, m),
          V=fill(sqrt(2.0)*VsNominal, m))
                                    annotation (Placement(transformation(
              origin={-60,-50},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.MultiPhase.Ideal.IdealClosingSwitch idealCloser(
          final m=m,
          Ron=fill(1e-5*m/3, m),
          Goff=fill(1e-5*3/m, m))
                              annotation (Placement(transformation(
              origin={-60,-20},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime=tOn)
          annotation (Placement(transformation(extent={{-100,-30},{-80,-10}},
                rotation=0)));
        Modelica.Electrical.MultiPhase.Sensors.CurrentQuasiRMSSensor currentRMSsensor(m=m)
                            annotation (Placement(transformation(
              origin={-20,-10},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Machines.Utilities.TerminalBox terminalBoxM(m=m,
          terminalConnection="Y")   annotation (Placement(transformation(extent={{20,-60},
                  {40,-40}},          rotation=0)));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(m=m, terminalConnection="Y")
                                    annotation (Placement(transformation(extent={{20,40},
                  {40,60}},            rotation=0)));
        Modelica.Magnetic.FundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing
          aims(
          Jr=aimsData.Jr,
          Js=aimsData.Js,
          p=aimsData.p,
          fsNominal=aimsData.fsNominal,
          TsRef=aimsData.TsRef,
          alpha20s(displayUnit="1/K") = aimsData.alpha20s,
          frictionParameters=aimsData.frictionParameters,
          statorCoreParameters=aimsData.statorCoreParameters,
          strayLoadParameters=aimsData.strayLoadParameters,
          phiMechanical(fixed=true),
          wMechanical(fixed=true),
          TrRef=aimsData.TrRef,
          alpha20r(displayUnit="1/K") = aimsData.alpha20r,
          useTurnsRatio=aimsData.useTurnsRatio,
          VsNominal=aimsData.VsNominal,
          VrLockedRotor=aimsData.VrLockedRotor,
          rotorCoreParameters=aimsData.rotorCoreParameters,
          TurnsRatio=aimsData.turnsRatio,
          mr=m,
          m=m,
          Rs=aimsData.Rs*m/3,
          Lssigma=aimsData.Lssigma*m/3,
          Lszero=aimsData.Lszero*m/3,
          Lm=aimsData.Lm*m/3,
          Lrsigma=aimsData.Lrsigma*m/3,
          Lrzero=aimsData.Lrzero*m/3,
          Rr=aimsData.Rr*m/3,
          TsOperational=293.15,
          TrOperational=293.15) annotation (Placement(transformation(extent={{20,-80},
                  {40,-60}},      rotation=0)));
        QuasiStationaryFundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing
          aimsQS(
          p=aimsData.p,
          fsNominal=aimsData.fsNominal,
          TsRef=aimsData.TsRef,
          alpha20s(displayUnit="1/K") = aimsData.alpha20s,
          Jr=aimsData.Jr,
          Js=aimsData.Js,
          frictionParameters=aimsData.frictionParameters,
          statorCoreParameters=aimsData.statorCoreParameters,
          strayLoadParameters=aimsData.strayLoadParameters,
          TrRef=aimsData.TrRef,
          alpha20r(displayUnit="1/K") = aimsData.alpha20r,
          useTurnsRatio=aimsData.useTurnsRatio,
          VsNominal=aimsData.VsNominal,
          VrLockedRotor=aimsData.VrLockedRotor,
          rotorCoreParameters=aimsData.rotorCoreParameters,
          mr=m,
          m=m,
          Rs=aimsData.Rs*m/3,
          Lssigma=aimsData.Lssigma*m/3,
          Lm=aimsData.Lm*m/3,
          Lrsigma=aimsData.Lrsigma*m/3,
          Rr=aimsData.Rr*m/3,
        gammar(fixed=true, start=pi/2),
        gamma(fixed=true, start=-pi/2),
        wMechanical(fixed=true),
        TsOperational=566.3,
        TrOperational=566.3,
        TurnsRatio=aimsData.turnsRatio)
                               annotation (Placement(transformation(extent={{20,20},{40,
                  40}},           rotation=0)));
        Modelica.Electrical.Machines.Utilities.SwitchedRheostat rheostatM(
          tStart=tRheostat,
          m=m,
          RStart=RStart*m/3)
               annotation (Placement(transformation(extent={{0,-80},{20,-60}},
                rotation=0)));
        QuasiStationaryFundamentalWave.Utilities.SwitchedRheostat rheostatE(
          tStart=tRheostat,
          m=m,
          RStart=RStart*m/3)
               annotation (Placement(transformation(extent={{0,20},{20,40}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=J_Load)
          annotation (Placement(transformation(extent={{50,-80},{70,-60}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertiaE(J=J_Load)
          annotation (Placement(transformation(extent={{50,20},{70,40}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticLoadTorque(
          tau_nominal=-T_Load,
          TorqueDirection=false,
          useSupport=false,
          w_nominal=w_Load) annotation (Placement(transformation(extent={{100,-80},{80,
                  -60}},     rotation=0)));
        Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque
          quadraticLoadTorqueE(
          tau_nominal=-T_Load,
          TorqueDirection=false,
          useSupport=false,
          w_nominal=w_Load) annotation (Placement(transformation(extent={{100,20},{80,
                  40}},      rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.AIM_SlipRingData
                                                                                   aimsData
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSourceQS(
          m=m,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          f=fNominal,
          V=fill(VsNominal, m))
          annotation (Placement(transformation(
              origin={-60,40},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(m=m)
          annotation (Placement(transformation(
              origin={-70,10},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-90,10})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor powerSensorQS(m=m)
          annotation (Placement(transformation(extent={{-60,80},{-40,100}},
                                                                          rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor currentSensorQS(m=m)
          annotation (Placement(transformation(extent={{-30,80},{-10,100}},
                                                                          rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Ideal.IdealClosingSwitch
                                                                                idealCloserQS(
          final m=m,
          Ron=fill(1e-5*m/3, m),
          Goff=fill(1e-5*3/m, m))
                              annotation (Placement(transformation(
              origin={-60,70},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        Modelica.Blocks.Sources.BooleanStep booleanStepQS[m](each startTime=tOn,
            each startValue=false)
                              annotation (Placement(transformation(extent={{-100,60},{
                  -80,80}},      rotation=0)));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.SymmetricalComponents
          symmetricalComponents(m=m) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-20,60})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerSensor(m=m)
          annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-10,30})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=0,
              origin={-10,0})));
      initial equation
        aims.is[1:m-1]=zeros(m-1);
        aims.ir[1:m]=zeros(m);

      equation
        connect(star.pin_n, ground.p)
          annotation (Line(points={{-80,-80},{-80,-80}},
                                                       color={0,0,255}));
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(points={{-60,-60},{-60,
                -80}},                  color={0,0,255}));
        connect(loadInertiaE.flange_b, quadraticLoadTorqueE.flange)
          annotation (Line(points={{70,30},{74,30},{80,30}},
                                                       color={0,0,0}));
        connect(aimsQS.flange, loadInertiaE.flange_a)
          annotation (Line(points={{40,30},{44,30},{50,30}},
                                                       color={0,0,0}));
        connect(booleanStep.y, idealCloser.control)
          annotation (Line(points={{-79,-20},{-79,-20},{-67,-20}},
                                                      color={255,0,255}));
        connect(terminalBoxQS.plug_sn, aimsQS.plug_sn)
          annotation (Line(points={{24,40},{24,40}},   color={0,0,255}));
        connect(terminalBoxQS.plug_sp, aimsQS.plug_sp)
          annotation (Line(points={{36,40},{36,40}}, color={0,0,255}));
        connect(loadInertia.flange_b, quadraticLoadTorque.flange)   annotation (
           Line(points={{70,-70},{70,-70},{80,-70}}, color={0,0,0}));
        connect(aims.flange, loadInertia.flange_a)   annotation (Line(points={{40,-70},
                {40,-70},{50,-70}},         color={0,0,0}));
        connect(terminalBoxM.plug_sp, aims.plug_sp)
          annotation (Line(points={{36,-60},{36,-60}},
                                                     color={0,0,255}));
        connect(terminalBoxM.plug_sn, aims.plug_sn)
          annotation (Line(points={{24,-60},{24,-60}}, color={0,0,255}));
        connect(currentRMSsensor.plug_n, terminalBoxM.plugSupply)  annotation (
            Line(
            points={{-10,-10},{30,-10},{30,-58}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rheostatM.plug_p, aims.plug_rp)  annotation (Line(
            points={{20,-64},{20,-64}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(rheostatM.plug_n, aims.plug_rn)  annotation (Line(
            points={{20,-76},{20,-76}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(idealCloser.plug_p, sineVoltage.plug_p) annotation (Line(
            points={{-60,-30},{-60,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(aimsQS.plug_rp, rheostatE.plug_p) annotation (Line(
            points={{20,36},{20,36}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(rheostatE.plug_n, aimsQS.plug_rn) annotation (Line(
            points={{20,24},{20,24}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(groundQS.pin,starQS. pin_n)
                                        annotation (Line(points={{-80,10},{-80,10}},
              color={85,170,255}));
        connect(starQS.plug_p,voltageSourceQS. plug_n)
                                                   annotation (Line(points={{-60,10},{
                -60,10},{-60,30}},
                            color={85,170,255}));
        connect(powerSensorQS.currentN,currentSensorQS. plug_p)
          annotation (Line(points={{-40,90},{-40,90},{-30,90}},
                                                       color={85,170,255}));
        connect(powerSensorQS.voltageP,powerSensorQS. currentP)
                                                            annotation (Line(points={{-50,100},
                {-60,100},{-60,90}},         color={85,170,255}));
        connect(powerSensorQS.voltageN,starQS. plug_p)
                                                   annotation (Line(points={{-50,80},{
                -50,10},{-60,10}},                   color={85,170,255}));
        connect(booleanStepQS.y,idealCloserQS. control)
                                                    annotation (Line(
            points={{-79,70},{-67,70}},
            color={255,0,255},
            smooth=Smooth.None));
        connect(idealCloserQS.plug_p,voltageSourceQS. plug_p) annotation (Line(
            points={{-60,60},{-60,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(idealCloserQS.plug_n,powerSensorQS. currentP) annotation (Line(
            points={{-60,80},{-60,90}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensorQS.y,symmetricalComponents. u) annotation (Line(
            points={{-20,79},{-20,72}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(idealCloser.plug_n, powerSensor.pc) annotation (Line(
            points={{-60,-10},{-60,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerSensor.pc, powerSensor.pv) annotation (Line(
            points={{-60,-10},{-60,0},{-50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerSensor.nv, star.plug_p) annotation (Line(
            points={{-50,-20},{-50,-80},{-60,-80}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerSensor.nc, currentRMSsensor.plug_p) annotation (Line(
            points={{-40,-10},{-30,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensorQS.plug_n, terminalBoxQS.plugSupply)
                                                                 annotation (Line(
            points={{-10,90},{30,90},{30,42}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.pin_n, groundMachineQS.pin)
                                                  annotation (Line(
            points={{-10,20},{-10,10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
            points={{-10,40},{-10,42},{21,42}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (experiment(
            StopTime=1.5,
            Interval=0.001,
            Tolerance=1e-05), Documentation(info="<HTML>
<h4>Starting of an asynchronous induction machine with slipring rotor resistance starting</h4>
<p>
At start time <code>tOn</code> three phase voltage is supplied to the
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing\">asynchronous induction machine with sliprings</a>.
The machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed,
using a starting resistance. At time tRheostat external rotor resistance is shortened, finally reaching nominal speed.</p>

<p>
Simulate for 1.5 seconds and plot (versus time):
</p>

<ul>
<li><code>currentRMSsensorM|E.I</code>: equivalent RMS stator current</li>
<li><code>aimsM/E.wMechanical</code>: machine speed</li>
<li><code>aimsM|E.tauElectrical</code>: machine torque</li>
</ul>
</HTML>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end AIMS_Start;

      model SMPM_Mains "Permanent magnet synchronous machine operated at mains"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of phases";
        parameter Modelica.SIunits.Frequency f = 50 "Supply frequency";
        // parameter Modelica.SIunits.Frequency fslip = 1e-3 "Supply frequency";
        parameter Modelica.SIunits.Voltage V = 112.3 "Suppy voltage";
        parameter Modelica.SIunits.Torque T_Load=181.4 "Nominal load torque";
        parameter Modelica.SIunits.Time tStep=0.5 "Time of load torque step";
        parameter Modelica.SIunits.Inertia J_Load=0.29 "Load inertia";
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          voltageSource(
          m=m,
          f=f,
          V=fill(V, m),
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(
              origin={-60,0},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star(m=m)
          annotation (Placement(transformation(
              origin={-60,-30},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground
          annotation (Placement(transformation(extent={{-70,-70},{-50,-50}}, rotation=
                 0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerSensor(m=m)
          annotation (Placement(transformation(extent={{-60,20},{-40,40}},rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor
          currentSensor(m=m)
          annotation (Placement(transformation(extent={{-20,20},{0,40}},  rotation=0)));
        QuasiStationaryFundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
          smpmQS(
          m=m,
          Jr=smpmData.Jr,
          p=smpmData.p,
          fsNominal=smpmData.fsNominal,
          Rs=smpmData.Rs,
          TsRef=smpmData.TsRef,
          Lssigma=smpmData.Lssigma,
          frictionParameters=smpmData.frictionParameters,
          statorCoreParameters=smpmData.statorCoreParameters,
          strayLoadParameters=smpmData.strayLoadParameters,
          Lmd=smpmData.Lmd,
          Lmq=smpmData.Lmq,
          useDamperCage=smpmData.useDamperCage,
          Lrsigmad=smpmData.Lrsigmad,
          Lrsigmaq=smpmData.Lrsigmaq,
          Rrd=smpmData.Rrd,
          Rrq=smpmData.Rrq,
          TrRef=smpmData.TrRef,
          VsOpenCircuit=smpmData.VsOpenCircuit,
          permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
        gammar(start=pi/2, fixed=true),
        gamma(start=-pi/2, fixed=true),
        TsOperational=293.15,
        alpha20s=smpmData.alpha20s,
        wMechanical(fixed=true, start=2*pi*smpmData.fsNominal/smpmData.p),
        alpha20r=smpmData.alpha20r,
        TrOperational=293.15)
               annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertiaM(J=J_Load)
          annotation (Placement(transformation(extent={{20,-20},{40,0}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStepM(
          startTime=tStep,
          stepTorque=-T_Load,
          useSupport=false,
          offsetTorque=0) annotation (Placement(transformation(extent={{70,-20},{50,0}},
                             rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
          smpmData(useDamperCage=true)
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=0,
              origin={-30,-38})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-30,-10})));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(m=m, terminalConnection="Y")
                                    annotation (Placement(transformation(extent={{-10,0},
                  {10,20}},            rotation=0)));
      equation
        connect(ground.pin,star. pin_n) annotation (Line(points={{-60,-50},{-60,-40}},
              color={85,170,255}));
        connect(star.plug_p,voltageSource. plug_n) annotation (Line(points={{-60,-20},
                {-60,-10}}, color={85,170,255}));
        connect(voltageSource.plug_p,powerSensor. currentP) annotation (Line(points={{-60,10},
                {-60,20},{-60,30}},           color={85,170,255}));
        connect(powerSensor.currentN,currentSensor. plug_p)
          annotation (Line(points={{-40,30},{-20,30}}, color={85,170,255}));
        connect(powerSensor.voltageP,powerSensor. currentP) annotation (Line(points={{-50,40},
                {-60,40},{-60,30}},          color={85,170,255}));
        connect(powerSensor.voltageN,star. plug_p) annotation (Line(points={{-50,20},{
                -50,-20},{-60,-20}},                 color={85,170,255}));
        connect(loadInertiaM.flange_b,torqueStepM. flange)
          annotation (Line(points={{40,-10},{50,-10}}, color={0,0,0}));
        connect(smpmQS.flange, loadInertiaM.flange_a)
                                                    annotation (Line(
            points={{10,-10},{20,-10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sn, smpmQS.plug_sn)
                                                     annotation (Line(
            points={{-6,0},{-6,0}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sp, smpmQS.plug_sp)
                                                     annotation (Line(
            points={{6,0},{6,0}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
            points={{-30,-1.77636e-015},{-30,2},{-9,2}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensor.plug_n, terminalBoxQS.plugSupply) annotation (Line(
            points={{0,30},{0,2}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.pin_n, groundMachineQS.pin) annotation (Line(
            points={{-30,-20},{-30,-28}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end SMPM_Mains;

      model SMPM_OpenCircuit
      "Test example: PermanentMagnetSynchronousInductionMachine with inverter"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of phases";
        QuasiStationaryFundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
          smpmQS(
          p=smpmData.p,
          fsNominal=smpmData.fsNominal,
          Rs=smpmData.Rs,
          TsRef=smpmData.TsRef,
          Lssigma=smpmData.Lssigma,
          Jr=smpmData.Jr,
          Js=smpmData.Js,
          frictionParameters=smpmData.frictionParameters,
          statorCoreParameters=smpmData.statorCoreParameters,
          strayLoadParameters=smpmData.strayLoadParameters,
          VsOpenCircuit=smpmData.VsOpenCircuit,
          Lmd=smpmData.Lmd,
          Lmq=smpmData.Lmq,
          useDamperCage=smpmData.useDamperCage,
          Lrsigmad=smpmData.Lrsigmad,
          Lrsigmaq=smpmData.Lrsigmaq,
          Rrd=smpmData.Rrd,
          Rrq=smpmData.Rrq,
          TrRef=smpmData.TrRef,
          permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
          phiMechanical(start=0),
          m=m,
          TsOperational=293.15,
          alpha20s=smpmData.alpha20s,
          alpha20r=smpmData.alpha20r,
          TrOperational=293.15)
          annotation (Placement(transformation(extent={{-10,20},{10,40}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(useSupport=
              false, w_fixed(displayUnit="rad/s") = 2*pi*smpmData.fsNominal/
            smpmData.p,
        phi(start=0, fixed=true))
                      annotation (Placement(transformation(extent={{80,-10},{60,10}},
                         rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
          smpmData(useDamperCage=false)
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(m=m)
                                                              annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-20,50})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundQS
                                                                   annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PotentialSensor potentialSensorQS(m=m)
          annotation (Placement(transformation(extent={{10,40},{30,60}})));
        Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
          smpm(
          p=smpmData.p,
          fsNominal=smpmData.fsNominal,
          Rs=smpmData.Rs,
          TsRef=smpmData.TsRef,
          Lssigma=smpmData.Lssigma,
          Jr=smpmData.Jr,
          Js=smpmData.Js,
          frictionParameters=smpmData.frictionParameters,
          statorCoreParameters=smpmData.statorCoreParameters,
          strayLoadParameters=smpmData.strayLoadParameters,
          VsOpenCircuit=smpmData.VsOpenCircuit,
          Lmd=smpmData.Lmd,
          Lmq=smpmData.Lmq,
          useDamperCage=smpmData.useDamperCage,
          Lrsigmad=smpmData.Lrsigmad,
          Lrsigmaq=smpmData.Lrsigmaq,
          Rrd=smpmData.Rrd,
          Rrq=smpmData.Rrq,
          TrRef=smpmData.TrRef,
          permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
          m=m,
          TsOperational=293.15,
          alpha20s=smpmData.alpha20s,
          alpha20r=smpmData.alpha20r,
          TrOperational=293.15)
          annotation (Placement(transformation(extent={{-10,-50},{10,-30}},
                rotation=0)));
        Modelica.Electrical.MultiPhase.Basic.Star star(m=m)
                                               annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={-20,-20})));
        Modelica.Electrical.Analog.Basic.Ground ground
                                               annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PotentialSensor potentialSensor(m=m)
          annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistorQS(m=m,
            R_ref=fill(1e6, m))
          annotation (Placement(transformation(extent={{-10,50},{10,70}})));
      equation
        connect(starQS.plug_p, smpmQS.plug_sn)
                                           annotation (Line(
            points={{-10,50},{-10,40},{-6,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(groundQS.pin, starQS.pin_n)
                                        annotation (Line(
            points={{-40,50},{-30,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(potentialSensorQS.plug_p, smpmQS.plug_sp)
                                                      annotation (Line(
            points={{10,50},{10,40},{6,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(constantSpeed.flange, smpm.flange)  annotation (Line(
            points={{60,0},{50,0},{50,-40},{10,-40}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(constantSpeed.flange, smpmQS.flange)
                                                   annotation (Line(
            points={{60,0},{50,0},{50,30},{10,30}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(ground.p, star.pin_n)   annotation (Line(
            points={{-40,-20},{-30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.plug_p, smpm.plug_sn)   annotation (Line(
            points={{-10,-20},{-10,-30},{-6,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(potentialSensor.plug_p, smpm.plug_sp)   annotation (Line(
            points={{10,-20},{10,-30},{6,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistorQS.plug_p, smpmQS.plug_sn)
                                               annotation (Line(
            points={{-10,60},{-10,40},{-6,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistorQS.plug_n, smpmQS.plug_sp)
                                               annotation (Line(
            points={{10,60},{10,40},{6,40}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (
          experiment(StopTime=0.1, Interval=0.001),
          Documentation(info="<html>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                  100,100}}),
                          graphics),
          __Dymola_experimentSetupOutput);
      end SMPM_OpenCircuit;

      model SMPM_Inverter
      "Test example: PermanentMagnetSynchronousInductionMachine with inverter"
        import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Integer m=3 "Number of phases";
        parameter Modelica.SIunits.Voltage VNominal=100
        "Nominal RMS voltage per phase";
        parameter Modelica.SIunits.Frequency fNominal=50 "Nominal frequency";
        parameter Modelica.SIunits.Frequency f=50 "Actual frequency";
        parameter Modelica.SIunits.Time tRamp=1 "Frequency ramp";
        parameter Modelica.SIunits.Torque TLoad=181.4 "Nominal load torque";
        parameter Modelica.SIunits.Time tStep=1.2 "Time of load torque step";
        parameter Modelica.SIunits.Inertia JLoad=0.29
        "Load's moment of inertia";
        Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
          smpm(
          p=smpmData.p,
          fsNominal=smpmData.fsNominal,
          Rs=smpmData.Rs,
          TsRef=smpmData.TsRef,
          Lszero=smpmData.Lszero,
          Lssigma=smpmData.Lssigma,
          Jr=smpmData.Jr,
          Js=smpmData.Js,
          frictionParameters=smpmData.frictionParameters,
          phiMechanical(fixed=true),
          wMechanical(fixed=true),
          statorCoreParameters=smpmData.statorCoreParameters,
          strayLoadParameters=smpmData.strayLoadParameters,
          VsOpenCircuit=smpmData.VsOpenCircuit,
          Lmd=smpmData.Lmd,
          Lmq=smpmData.Lmq,
          useDamperCage=smpmData.useDamperCage,
          Lrsigmad=smpmData.Lrsigmad,
          Lrsigmaq=smpmData.Lrsigmaq,
          Rrd=smpmData.Rrd,
          Rrq=smpmData.Rrq,
          TrRef=smpmData.TrRef,
          permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
          ir(fixed=true),
          m=m,
          TsOperational=293.15,
          alpha20s=smpmData.alpha20s,
          alpha20r=smpmData.alpha20r,
          TrOperational=293.15)
          annotation (Placement(transformation(extent={{20,-90},{40,-70}},
                rotation=0)));
        Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor
          annotation (Placement(transformation(
              origin={20,-50},
              extent={{-10,10},{10,-10}},
              rotation=0)));
        Modelica.Blocks.Sources.Ramp ramp(height=f, duration=tRamp)
          annotation (Placement(transformation(extent={{-100,-20},{-80,0}},
                rotation=0)));
        Modelica.Electrical.Machines.Utilities.VfController vfController(
          final m=m,
          VNominal=VNominal,
          fNominal=fNominal,
          BasePhase=+Modelica.Constants.pi/2)
          annotation (Placement(transformation(extent={{-40,-20},{-20,0}},
                rotation=0)));
        Modelica.Electrical.MultiPhase.Sources.SignalVoltage signalVoltage(final m=
              m)
          annotation (Placement(transformation(
              origin={-10,-50},
              extent={{10,-10},{-10,10}},
              rotation=0)));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
          annotation (Placement(transformation(extent={{-30,-60},{-50,-40}},
                rotation=0)));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(
              origin={-70,-50},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
                                                          J=JLoad)
          annotation (Placement(transformation(extent={{50,-90},{70,-70}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorqueStep(
                                                                startTime=tStep,
            stepTorque=-TLoad,
          useSupport=false,
          offsetTorque=0)
                      annotation (Placement(transformation(extent={{100,-90},{80,-70}},
                         rotation=0)));
        Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
            terminalConnection="Y")        annotation (Placement(transformation(
                extent={{20,-70},{40,-50}}, rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
          smpmData
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertiaQS(J=JLoad)
          annotation (Placement(transformation(extent={{50,10},{70,30}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorqueStepQS(
          startTime=tStep,
          stepTorque=-TLoad,
          useSupport=false,
          offsetTorque=0)
                      annotation (Placement(transformation(extent={{100,10},{80,30}},
                         rotation=0)));
        QuasiStationaryFundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet
          smpmQS(
          p=smpmData.p,
          fsNominal=smpmData.fsNominal,
          Rs=smpmData.Rs,
          TsRef=smpmData.TsRef,
          Lssigma=smpmData.Lssigma,
          Jr=smpmData.Jr,
          Js=smpmData.Js,
          frictionParameters=smpmData.frictionParameters,
          statorCoreParameters=smpmData.statorCoreParameters,
          strayLoadParameters=smpmData.strayLoadParameters,
          VsOpenCircuit=smpmData.VsOpenCircuit,
          Lmd=smpmData.Lmd,
          Lmq=smpmData.Lmq,
          useDamperCage=smpmData.useDamperCage,
          Lrsigmad=smpmData.Lrsigmad,
          Lrsigmaq=smpmData.Lrsigmaq,
          Rrd=smpmData.Rrd,
          Rrq=smpmData.Rrq,
          TrRef=smpmData.TrRef,
          permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
          m=m,
        wMechanical(fixed=true, start=0),
        gammar(start=pi/2, fixed=true),
        gamma(start=-pi/2, fixed=true),
        TsOperational=293.15,
        alpha20s=smpmData.alpha20s,
        alpha20r=smpmData.alpha20r,
        TrOperational=293.15)
          annotation (Placement(transformation(extent={{20,10},{40,30}},
                rotation=0)));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(terminalConnection="Y", m=m)
                                           annotation (Placement(transformation(
                extent={{20,30},{40,50}},   rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,30})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-30,30})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor currentSensorQS(m=m)
          annotation (Placement(transformation(extent={{-10,10},{10,-10}},rotation=0,
              origin={20,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VariableVoltageSource
                                                                                     signalVoltageQS(final m=m)
          annotation (Placement(transformation(
              origin={-10,50},
              extent={{10,-10},{-10,10}},
              rotation=0)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundQS
          annotation (Placement(transformation(
              origin={-70,50},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(final m=m)
          annotation (Placement(transformation(extent={{-30,40},{-50,60}},
                rotation=0)));
        QuasiStationaryFundamentalWave.Utilities.VfController vfControllerQS(
          final m=m,
          VNominal=VNominal,
          fNominal=fNominal,
          BasePhase=+Modelica.Constants.pi/2)
          annotation (Placement(transformation(extent={{-40,80},{-20,100}},
                rotation=0)));
      initial equation
        smpm.is[1:2]=zeros(2);
      //conditional damper cage currents are defined as fixed start values

      equation
        connect(signalVoltage.plug_n, star.plug_p)
          annotation (Line(points={{-20,-50},{-20,-50},{-30,-50}},
              color={0,0,255}));
        connect(star.pin_n, ground.p)
          annotation (Line(points={{-50,-50},{-54,-50},{-60,-50}},
                                                       color={0,0,255}));
        connect(ramp.y, vfController.u)
          annotation (Line(points={{-79,-10},{-70,-10},{-42,-10}},
                                                       color={0,0,255}));
        connect(loadInertia.flange_b, loadTorqueStep.flange)
          annotation (Line(points={{70,-80},{80,-80}}, color={0,0,0}));
        connect(signalVoltage.plug_p, currentQuasiRMSSensor.plug_p)
                                                                 annotation (Line(
              points={{0,-50},{0,-50},{10,-50}},                          color={
                0,0,255}));
        connect(terminalBox.plugSupply, currentQuasiRMSSensor.plug_n)
                                                                   annotation (Line(
            points={{30,-68},{30,-50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(terminalBox.plug_sn, smpm.plug_sn)   annotation (Line(
            points={{24,-70},{24,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(terminalBox.plug_sp, smpm.plug_sp)   annotation (Line(
            points={{36,-70},{36,-70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(smpm.flange, loadInertia.flange_a) annotation (Line(
            points={{40,-80},{50,-80}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(vfController.y, signalVoltage.v) annotation (Line(
            points={{-19,-10},{-10,-10},{-10,-43}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(loadInertiaQS.flange_b, loadTorqueStepQS.flange)
          annotation (Line(points={{70,20},{80,20}},   color={0,0,0}));
        connect(smpmQS.flange, loadInertiaQS.flange_a) annotation (Line(
            points={{40,20},{50,20}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sn, smpmQS.plug_sn) annotation (Line(
            points={{24,30},{24,30}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sp, smpmQS.plug_sp) annotation (Line(
            points={{36,30},{36,30}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
            points={{10,30},{10,32},{21,32}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(groundMachineQS.pin, starMachineQS.pin_n) annotation (Line(
            points={{-20,30},{-10,30}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(currentSensorQS.plug_n, terminalBoxQS.plugSupply) annotation (Line(
            points={{30,50},{30,32}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(signalVoltageQS.plug_p, currentSensorQS.plug_p) annotation (Line(
            points={{0,50},{10,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(groundQS.pin, starQS.pin_n) annotation (Line(
            points={{-60,50},{-50,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starQS.plug_p, signalVoltageQS.plug_n) annotation (Line(
            points={{-30,50},{-20,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(vfControllerQS.y, signalVoltageQS.V) annotation (Line(
            points={{-19,90},{-6,90},{-6,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(ramp.y, vfControllerQS.u) annotation (Line(
            points={{-79,-10},{-70,-10},{-70,90},{-42,90}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(ramp.y, signalVoltageQS.f) annotation (Line(
            points={{-79,-10},{-70,-10},{-70,70},{-14,70},{-14,60}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          experiment(StopTime=1.5, Interval=0.001),
          Documentation(info="<HTML>
<b>Test example: Permanent magnet synchronous induction machine fed by an ideal inverter</b><br>
An ideal frequency inverter is modeled by using a VfController and a three-phase SignalVoltage.<br>
Frequency is raised by a ramp, causing the permanent magnet synchronous induction machine to start,
and accelerating inertias.<br>At time tStep a load step is applied.<br>
Simulate for 1.5 seconds and plot (versus time):
<ul>
<li>currentQuasiRMSSensor.I: stator current RMS</li>
<li>smpm.wMechanical: motor's speed</li>
<li>smpm.tauElectrical: motor's torque</li>
<li>rotorDisplacementAngle.rotorDisplacementAngle: rotor displacement angle</li>
</ul>
Default machine parameters of model <i>SM_PermanentMagnet</i> are used.
<p>
<b>In practice it is nearly impossible to drive a PMSMD without current controller.</b>
</p>
</HTML>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end SMPM_Inverter;

      model SMEE_Generator
      "Electrical excited synchronous machine operating as generator"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of stator phases";
        parameter Modelica.SIunits.Voltage VsNominal=100
        "Nominal RMS voltage per phase";
        parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
        parameter Modelica.SIunits.AngularVelocity w=
            Modelica.SIunits.Conversions.from_rpm(1499) "Nominal speed";
        parameter Modelica.SIunits.Current Ie=19 "Excitation current";
        parameter Modelica.SIunits.Current Ie0=10 "Initial excitation current";
        parameter Modelica.SIunits.Angle gamma0(displayUnit="deg") = 0
        "Initial rotor displacement angle";
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(extent={{-50,-20},{-70,0}},  rotation=0)));
        Modelica.Electrical.Analog.Basic.Ground grounde
                                                       annotation (Placement(
              transformation(
              origin={-90,-10},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          final V=fill(VsNominal*sqrt(2), m),
          final freqHz=fill(fsNominal, m)) annotation (Placement(transformation(
                extent={{-20,-20},{-40,0}},  rotation=0)));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor         electricalPowerSensor(m=m)
                                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-20})));
        Modelica.Electrical.Machines.Utilities.TerminalBox terminalBoxM(
            terminalConnection="Y", m=m)
                                    annotation (Placement(transformation(extent={{-10,-50},
                  {10,-30}},          rotation=0)));
        Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
          smee(
          phiMechanical(start=-(Modelica.Constants.pi + gamma0)/smee.p, fixed=true),
          Jr=0.29,
          Js=0.29,
          p=2,
          fsNominal=smeeData.fsNominal,
          Rs=smeeData.Rs,
          TsRef=smeeData.TsRef,
          alpha20s(displayUnit="1/K") = smeeData.alpha20s,
          Lssigma=smeeData.Lssigma,
          Lmd=smeeData.Lmd,
          Lmq=smeeData.Lmq,
          Lrsigmad=smeeData.Lrsigmad,
          Lrsigmaq=smeeData.Lrsigmaq,
          Rrd=smeeData.Rrd,
          Rrq=smeeData.Rrq,
          TrRef=smeeData.TrRef,
          alpha20r(displayUnit="1/K") = smeeData.alpha20r,
          VsNominal=smeeData.VsNominal,
          IeOpenCircuit=smeeData.IeOpenCircuit,
          Re=smeeData.Re,
          TeRef=smeeData.TeRef,
          alpha20e(displayUnit="1/K") = smeeData.alpha20e,
          sigmae=smeeData.sigmae,
          statorCoreParameters(VRef=100),
          strayLoadParameters(IRef=100),
          brushParameters(ILinear=0.01),
          ir(fixed=true),
          useDamperCage=false,
          m=m,
        TsOperational=293.15,
        frictionParameters(PRef=0),
        TrOperational=293.15,
        TeOperational=293.15)                 annotation (Placement(
              transformation(extent={{-10,-70},{10,-50}}, rotation=0)));
        QuasiStationaryFundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited
          smeeQS(
          p=2,
          fsNominal=smeeData.fsNominal,
          Rs=smeeData.Rs,
          TsRef=smeeData.TsRef,
          alpha20s(displayUnit="1/K") = smeeData.alpha20s,
          Lssigma=smeeData.Lssigma,
          Jr=0.29,
          Js=0.29,
          frictionParameters(PRef=0),
          statorCoreParameters(PRef=0, VRef=100),
          strayLoadParameters(PRef=0, IRef=100),
          Lmd=smeeData.Lmd,
          Lmq=smeeData.Lmq,
          Lrsigmad=smeeData.Lrsigmad,
          Rrd=smeeData.Rrd,
          Rrq=smeeData.Rrq,
          alpha20r(displayUnit="1/K") = smeeData.alpha20r,
          VsNominal=smeeData.VsNominal,
          IeOpenCircuit=smeeData.IeOpenCircuit,
          Re=smeeData.Re,
          TeRef=smeeData.TeRef,
          alpha20e(displayUnit="1/K") = smeeData.alpha20e,
          brushParameters(V=0, ILinear=0.01),
          Lrsigmaq=smeeData.Lrsigmaq,
          TrRef=smeeData.TrRef,
          useDamperCage=false,
          m=m,
        TsOperational=293.15,
        gammar(fixed=true, start=pi/2),
        gamma(fixed=true, start=-pi/2),
        TrOperational=293.15,
        TeOperational=293.15)   annotation (Placement(transformation(extent={{-10,30},
                  {10,50}},       rotation=0)));
        Modelica.Electrical.Analog.Basic.Ground groundr annotation (Placement(
              transformation(
              origin={-50,-70},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground groundrQS
                                                        annotation (Placement(
              transformation(
              origin={-50,30},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.Analog.Sources.RampCurrent rampCurrent(
          duration=0.1,
          I=Ie - Ie0,
          offset=Ie0) annotation (Placement(transformation(
              origin={-30,-60},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Electrical.Analog.Sources.RampCurrent rampCurrentQS(
          duration=0.1,
          I=Ie - Ie0,
          offset=Ie0) annotation (Placement(transformation(
              origin={-28,40},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor mechanicalPowerSensor
                                 annotation (Placement(transformation(extent={{20,-70},
                  {40,-50}},         rotation=0)));
        Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor mechanicalPowerSensorQS
                                 annotation (Placement(transformation(extent={{20,30},
                  {40,50}},          rotation=0)));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(final
            w_fixed=w, useSupport=false)       annotation (Placement(
              transformation(extent={{70,-70},{50,-50}},  rotation=0)));
        Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeedQS(final
            w_fixed=w, useSupport=false)       annotation (Placement(
              transformation(extent={{70,30},{50,50}},    rotation=0)));
        parameter Modelica.Electrical.Machines.Utilities.SynchronousMachineData
                                                                                smeeData(
          SNominal=30e3,
          VsNominal=100,
          fsNominal=50,
          IeOpenCircuit=10,
          x0=0.1,
          xd=1.6,
          xdTransient=0.1375,
          xdSubtransient=0.121428571,
          xqSubtransient=0.148387097,
          Ta=0.014171268,
          Td0Transient=0.261177343,
          Td0Subtransient=0.006963029,
          Tq0Subtransient=0.123345081,
          alpha20s(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
          alpha20r(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
          alpha20e(displayUnit="1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero,
          xq=1.1,
          TsSpecification=293.15,
          TsRef=293.15,
          TrSpecification=293.15,
          TrRef=293.15,
          TeSpecification=293.15,
          TeRef=293.15)
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource voltageSourceQS(
          m=m,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(VsNominal, m),
          f=fsNominal)
          annotation (Placement(transformation(
              origin={-30,90},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(m=m)
          annotation (Placement(transformation(
              origin={-60,90},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundeQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-90,90})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor powerSensorQS(m=m)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,
              origin={0,80})));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(m=m, terminalConnection="Y")
                                    annotation (Placement(transformation(extent={{-10,50},
                {10,70}},              rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={-20,60})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-50,60})));
      initial equation
        smee.is[1:2] = zeros(2);

      equation
        connect(star.pin_n, grounde.p)
          annotation (Line(points={{-70,-10},{-80,-10}},
                                                       color={0,0,255}));
        connect(star.plug_p, sineVoltage.plug_n)
          annotation (Line(points={{-50,-10},{-40,-10}},
                                                       color={0,0,255}));
        connect(smeeQS.flange, mechanicalPowerSensorQS.flange_a)
          annotation (Line(points={{10,40},{20,40}},   color={0,0,0}));
        connect(mechanicalPowerSensorQS.flange_b, constantSpeedQS.flange)
          annotation (Line(points={{40,40},{50,40}},   color={0,0,0}));
        connect(rampCurrentQS.p, groundrQS.p)
          annotation (Line(points={{-28,30},{-40,30}},   color={0,0,255}));
        connect(rampCurrentQS.p, smeeQS.pin_en)
                                              annotation (Line(points={{-28,30},{-20,30},
                {-20,34},{-10,34}},            color={0,0,255}));
        connect(rampCurrentQS.n, smeeQS.pin_ep)
                                              annotation (Line(points={{-28,50},{-20,50},
                {-20,46},{-10,46}},            color={0,0,255}));
        connect(smee.flange, mechanicalPowerSensor.flange_a)
          annotation (Line(points={{10,-60},{20,-60}}, color={0,0,0}));
        connect(mechanicalPowerSensor.flange_b, constantSpeed.flange)
          annotation (Line(points={{40,-60},{50,-60}}, color={0,0,0}));
        connect(rampCurrent.p, groundr.p)  annotation (Line(points={{-30,-70},{-35,-70},
                {-40,-70}},                    color={0,0,255}));
        connect(rampCurrent.p, smee.pin_en)   annotation (Line(points={{-30,-70},{-20,
                -70},{-20,-66},{-10,-66}},      color={0,0,255}));
        connect(rampCurrent.n, smee.pin_ep)   annotation (Line(points={{-30,-50},{-20,
                -50},{-20,-54},{-10,-54}},      color={0,0,255}));
        connect(smee.plug_sn, terminalBoxM.plug_sn)
          annotation (Line(points={{-6,-50},{-6,-50}}, color={0,0,255}));
        connect(smee.plug_sp, terminalBoxM.plug_sp)
          annotation (Line(points={{6,-50},{6,-50}}, color={0,0,255}));
        connect(groundeQS.pin, starQS.pin_n)
                                        annotation (Line(points={{-80,90},{-80,90},{-70,
                90}},
              color={85,170,255}));
        connect(starQS.plug_p, voltageSourceQS.plug_n)
                                                   annotation (Line(points={{-50,90},{
                -50,90},{-40,90}},
                            color={85,170,255}));
      connect(voltageSourceQS.plug_p, powerSensorQS.currentP)
                                                            annotation (Line(points={{-20,90},
                {-20,90},{0,90}},             color={85,170,255}));
      connect(powerSensorQS.voltageP, powerSensorQS.currentP)
                                                            annotation (Line(points={{10,80},
                {10,90},{0,90}},             color={85,170,255}));
      connect(powerSensorQS.voltageN, starQS.plug_p)
                                                   annotation (Line(points={{-10,80},
              {-10,80},{-42,80},{-50,80},{-50,90}},  color={85,170,255}));
        connect(sineVoltage.plug_p, electricalPowerSensor.pc) annotation (Line(
            points={{-20,-10},{0,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(electricalPowerSensor.pc, electricalPowerSensor.pv) annotation (Line(
            points={{0,-10},{10,-10},{10,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(electricalPowerSensor.nv, star.plug_p) annotation (Line(
            points={{-10,-20},{-50,-20},{-50,-10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(electricalPowerSensor.nc, terminalBoxM.plugSupply) annotation (Line(
            points={{0,-30},{0,-48}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sn, smeeQS.plug_sn) annotation (Line(
            points={{-6,50},{-6,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(terminalBoxQS.plug_sp, smeeQS.plug_sp) annotation (Line(
            points={{6,50},{6,50}},
            color={85,170,255},
            smooth=Smooth.None));
      connect(powerSensorQS.currentN, terminalBoxQS.plugSupply) annotation (Line(
            points={{0,70},{0,52}},
            color={85,170,255},
            smooth=Smooth.None));
      connect(starMachineQS.pin_n,groundMachineQS. pin) annotation (Line(
          points={{-30,60},{-40,60}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
          points={{-10,60},{-10,52},{-9,52}},
          color={85,170,255},
          smooth=Smooth.None));
        annotation (experiment(
            StopTime=30,
            Interval=0.005,
            Tolerance=1e-06), Documentation(info="<HTML>
<h4>Electrical excited synchronous induction machine as generator</h4>
<p>
An
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited\">electrically excited synchronous generator</a> is connected to the grid and driven with constant speed.
Since speed is slightly smaller than synchronous speed corresponding to mains frequency,
rotor angle is very slowly increased. This allows to see several characteristics dependent on rotor angle.
</p>

<p>
Simulate for 30 seconds and plot (versus <code>rotorAngleM.rotorDisplacementAngle</code>):
</p>

<ul>
<li><code>speedM|E.tauElectrical</code>: machine torque</li>
<li><code>mechanicalPowerSensorM|E.P</code>: mechanical power</li>
</ul>
</HTML>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                          graphics));
      end SMEE_Generator;

      model SMR_Inverter
      "Starting of synchronous reluctance machine with inverter"
        import Modelica.Constants.pi;
        extends Modelica.Icons.Example;
        parameter Integer m=3 "Number of stator phases";
        parameter Modelica.SIunits.Voltage VsNominal=100
        "Nominal RMS voltage per phase";
        parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
        parameter Modelica.SIunits.Frequency fKnee=50
        "Knee frequency of V/f curve";
        parameter Modelica.SIunits.Time tRamp=1 "Frequency ramp";
        parameter Modelica.SIunits.Torque T_Load=46 "Nominal load torque";
        parameter Modelica.SIunits.Time tStep=2 "Time of load torque step";
        parameter Modelica.SIunits.Inertia J_Load=0.29 "Load inertia";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
              transformation(
              origin={-70,-50},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) annotation (
            Placement(transformation(extent={{-30,-60},{-50,-40}},rotation=0)));
        Modelica.Electrical.MultiPhase.Sources.SignalVoltage signalVoltage(
            final m=m) annotation (Placement(transformation(
              origin={-10,-50},
              extent={{10,-10},{-10,10}},
              rotation=0)));
        Modelica.Blocks.Sources.Ramp ramp(height=fKnee, duration=tRamp,
          startTime=0.1)
          annotation (Placement(transformation(extent={{-100,-30},{-80,-10}},
                rotation=0)));
        Modelica.Electrical.Machines.Utilities.VfController vfController(
          final m=m,
          VNominal=VsNominal,
          fNominal=fsNominal) annotation (Placement(transformation(extent={{-40,-30},{
                  -20,-10}},     rotation=0)));
        Modelica.Electrical.MultiPhase.Sensors.CurrentQuasiRMSSensor currentRMSsensorM(m=m)
                            annotation (Placement(transformation(
              origin={20,-50},
              extent={{-10,10},{10,-10}},
              rotation=0)));
        Modelica.Electrical.Machines.Utilities.TerminalBox terminalBoxM(
            terminalConnection="Y", m=m)
                                    annotation (Placement(transformation(extent={{20,-70},
                  {40,-50}},          rotation=0)));
        Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor
          smr(
          Jr=smrData.Jr,
          Js=smrData.Js,
          p=smrData.p,
          fsNominal=smrData.fsNominal,
          Rs=smrData.Rs,
          TsRef=smrData.TsRef,
          alpha20s(displayUnit="1/K") = smrData.alpha20s,
          Lssigma=smrData.Lssigma,
          Lszero=smrData.Lszero,
          frictionParameters=smrData.frictionParameters,
          statorCoreParameters=smrData.statorCoreParameters,
          strayLoadParameters=smrData.strayLoadParameters,
          phiMechanical(fixed=true),
          wMechanical(fixed=true),
          Lmd=smrData.Lmd,
          Lmq=smrData.Lmq,
          useDamperCage=smrData.useDamperCage,
          Lrsigmad=smrData.Lrsigmad,
          Lrsigmaq=smrData.Lrsigmaq,
          Rrd=smrData.Rrd,
          Rrq=smrData.Rrq,
          TrRef=smrData.TrRef,
          alpha20r(displayUnit="1/K") = smrData.alpha20r,
          ir(fixed=true),
          m=m,
          TsOperational=293.15,
          TrOperational=293.15)                           annotation (Placement(
              transformation(extent={{20,-90},{40,-70}},  rotation=0)));
        QuasiStationaryFundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor
          smrQS(
          p=smrData.p,
          fsNominal=smrData.fsNominal,
          Rs=smrData.Rs,
          TsRef=smrData.TsRef,
          alpha20s(displayUnit="1/K") = smrData.alpha20s,
          Lssigma=smrData.Lssigma,
          Jr=smrData.Jr,
          Js=smrData.Js,
          frictionParameters=smrData.frictionParameters,
          statorCoreParameters=smrData.statorCoreParameters,
          strayLoadParameters=smrData.strayLoadParameters,
          Lmd=smrData.Lmd,
          Lmq=smrData.Lmq,
          useDamperCage=smrData.useDamperCage,
          Lrsigmad=smrData.Lrsigmad,
          Lrsigmaq=smrData.Lrsigmaq,
          Rrd=smrData.Rrd,
          Rrq=smrData.Rrq,
          TrRef=smrData.TrRef,
          alpha20r(displayUnit="1/K") = smrData.alpha20r,
        TsOperational=293.15,
        gammar(fixed=true, start=pi/2),
        gamma(fixed=true, start=-pi/2),
        wMechanical(fixed=true, start=0),
        TrOperational=293.15)                             annotation (Placement(
              transformation(extent={{20,10},{40,30}},    rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=J_Load)
          annotation (Placement(transformation(extent={{50,-90},{70,-70}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia loadInertiaE(J=J_Load)
          annotation (Placement(transformation(extent={{50,10},{70,30}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStep(
          startTime=tStep,
          stepTorque=-T_Load,
          useSupport=false,
          offsetTorque=0) annotation (Placement(transformation(extent={{100,-90},{80,-70}},
                             rotation=0)));
        Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStepE(
          startTime=tStep,
          stepTorque=-T_Load,
          useSupport=false,
          offsetTorque=0) annotation (Placement(transformation(extent={{100,10},{80,30}},
                             rotation=0)));
        parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_ReluctanceRotorData
          smrData
          annotation (Placement(transformation(extent={{80,80},{100,100}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VariableVoltageSource
          voltageSourceQS(
          m=m)
          annotation (Placement(transformation(
              origin={-10,50},
              extent={{-10,10},{10,-10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starQS(m=m)
          annotation (Placement(transformation(
              origin={-40,50},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundeQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=270,
              origin={-70,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.CurrentSensor
          powerSensor(m=m)
          annotation (Placement(transformation(extent={{-10,10},{10,-10}},rotation=0,
              origin={20,50})));
        QuasiStationaryFundamentalWave.Utilities.VfController vfControllerQS(
          VNominal=VsNominal,
          m=m,
          fNominal=fsNominal)
          annotation (Placement(transformation(extent={{-40,80},{-20,100}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground groundMachineQS
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},   rotation=0,
              origin={0,-10})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star starMachineQS(m=
              QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(
               m)) annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={0,20})));
        QuasiStationaryFundamentalWave.MoveTo_Modelica.QuasiStationary_MultiPhase.TerminalBox
          terminalBoxQS(m=m, terminalConnection="Y")
                                    annotation (Placement(transformation(extent={{20,30},
                  {40,50}},            rotation=0)));
      initial equation
        smr.is[1:2] = zeros(2);

      equation
        connect(signalVoltage.plug_n, star.plug_p) annotation (Line(points={{-20,-50},
                {-20,-50},{-30,-50}},                       color={0,0,255}));
        connect(star.pin_n, ground.p)
          annotation (Line(points={{-50,-50},{-54,-50},{-60,-50}},
                                                       color={0,0,255}));
        connect(smrQS.flange, loadInertiaE.flange_a)
          annotation (Line(points={{40,20},{46,20},{50,20}},
                                                       color={0,0,0}));
        connect(vfController.y, signalVoltage.v)
          annotation (Line(points={{-19,-20},{-10,-20},{-10,-43}},
                                                      color={0,0,255}));
        connect(loadInertiaE.flange_b, torqueStepE.flange)
          annotation (Line(points={{70,20},{76,20},{80,20}},
                                                       color={0,0,0}));
        connect(smr.flange, loadInertia.flange_a)   annotation (Line(points={{40,-80},
                {40,-80},{50,-80}},         color={0,0,0}));
        connect(loadInertia.flange_b, torqueStep.flange)
          annotation (Line(points={{70,-80},{76,-80},{80,-80}},
                                                       color={0,0,0}));
        connect(terminalBoxM.plug_sp, smr.plug_sp)
          annotation (Line(points={{36,-70},{36,-70}},
                                                     color={0,0,255}));
        connect(terminalBoxM.plug_sn, smr.plug_sn)
          annotation (Line(points={{24,-70},{24,-70}}, color={0,0,255}));
        connect(currentRMSsensorM.plug_n, terminalBoxM.plugSupply) annotation (
            Line(points={{30,-50},{30,-68}},                        color={0,0,
                255}));
        connect(groundeQS.pin,starQS. pin_n)
                                        annotation (Line(points={{-60,50},{-60,50},{-50,
                50}},
              color={85,170,255}));
        connect(starQS.plug_p,voltageSourceQS. plug_n)
                                                   annotation (Line(points={{-30,50},{
                -30,50},{-20,50}},
                            color={85,170,255}));
        connect(voltageSourceQS.plug_p, powerSensor.plug_p) annotation (Line(
            points={{0,50},{10,50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(vfControllerQS.y, voltageSourceQS.V) annotation (Line(
            points={{-19,90},{-6,90},{-6,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(signalVoltage.plug_p, currentRMSsensorM.plug_p) annotation (Line(
            points={{0,-50},{10,-50}},
            color={0,0,255},
            smooth=Smooth.None));
      connect(terminalBoxQS.plug_sn, smrQS.plug_sn) annotation (Line(
          points={{24,30},{24,30}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(terminalBoxQS.plug_sp, smrQS.plug_sp) annotation (Line(
          points={{36,30},{36,30}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(starMachineQS.plug_p, terminalBoxQS.starpoint) annotation (Line(
          points={{1.77636e-015,30},{1.77636e-015,32},{21,32}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(starMachineQS.pin_n, groundMachineQS.pin) annotation (Line(
          points={{0,10},{0,0}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(powerSensor.plug_n, terminalBoxQS.plugSupply) annotation (Line(
          points={{30,50},{30,32}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(ramp.y, vfController.u) annotation (Line(
          points={{-79,-20},{-42,-20}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ramp.y, vfControllerQS.u) annotation (Line(
          points={{-79,-20},{-70,-20},{-70,10},{-80,10},{-80,90},{-42,90}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ramp.y, voltageSourceQS.f) annotation (Line(
          points={{-79,-20},{-70,-20},{-70,10},{-80,10},{-80,70},{-14,70},{-14,60}},
          color={0,0,127},
          smooth=Smooth.None));
        annotation (experiment(
            StopTime=3,
            Interval=0.001,
            Tolerance=1e-06), Documentation(info="<HTML>
<h4>Synchronous induction machine with reluctance rotor fed by an ideal inverter</h4>
<p>
An ideal frequency inverter is modeled by using a
<a href=\"modelica://Modelica.Electrical.Machines.Utilities.VfController\">VfController</a>
and a three-phase <a href=\"modelica://Modelica.Electrical.MultiPhase.Sources.SignalVoltage\">SignalVoltage</a>.
Frequency is raised by a ramp, causing the
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor\">reluctance machine</a> to start,
and accelerating inertias. At time <code>tStep</code> a load step is applied.
</p>

<p>
Simulate for 1.5 seconds and plot (versus time):
</p>

<ul>
<li><code>currentRMSsensorM|E.I</code>: equivalent RMS stator current</li>
<li><code>smrM|E.wMechanical</code>: machine speed</li>
<li><code>smrM|E.tauElectrical</code>: machine torque</li>
<li><code>rotorAngleM|R.rotorDisplacementAngle</code>: rotor displacement angle</li>
</ul>
</HTML>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics),
          __Dymola_experimentSetupOutput);
      end SMR_Inverter;
    end BasicMachines;
  end Examples;


  package Components "Basic fundamental wave components"
    extends Modelica.Icons.Package;
    model Ground "Magnetic ground"
      QuasiStationaryFundamentalWave.Interfaces.PositiveMagneticPort port_p
      "Complex magnetic port"
        annotation (Placement(transformation(extent={{-10,90},{10,110}}, rotation=0)));
    equation
      Connections.potentialRoot(port_p.reference, 254);
      if Connections.isRoot(port_p.reference) then
        port_p.reference.gamma = 0;
      end if;
      port_p.V_m = Complex(0,0);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics={
            Line(points={{0,100},{0,50}}, color={255,170,85}),
            Line(points={{-60,50},{60,50}}, color={255,170,85}),
            Line(points={{-40,30},{40,30}}, color={255,170,85}),
            Line(points={{-20,10},{20,10}}, color={255,170,85})}),
            Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Line(points={{0,100},{0,50}}, color={255,128,0}),
            Line(points={{-60,50},{60,50}}, color={255,128,0}),
            Line(points={{-40,30},{40,30}}, color={255,128,0}),
            Line(points={{-20,10},{20,10}}, color={255,128,0}),
            Text(
              extent={{-144,-19},{156,-59}},
              textString="%name",
              lineColor={0,0,255})}),
      Documentation(info="<html>

<p>
Grounding of the complex magnetic potential. Each magnetic circuit has to be grounded at least one point of the circuit.
</p>

</html>"));
    end Ground;

    model Reluctance "Salient reluctance"
      import Modelica.Constants.pi;
      extends
      QuasiStationaryFundamentalWave.Interfaces.PartialTwoPortElementary;
      parameter Modelica.Magnetic.FundamentalWave.Types.SalientReluctance R_m(
        d(start=1),
        q(start=1)) "Magnetic reluctance in d=re and q=im axis";
    equation
      (pi/2) * V_m.re = R_m.d * Phi.re;
      (pi/2) * V_m.im = R_m.q * Phi.im;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-70,30},{70,-30}},
              lineColor={255,170,85},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-96,0},{-70,0}}, color={255,170,85}),
            Line(points={{70,0},{96,0}}, color={255,170,85}),
            Text(
              extent={{0,60},{0,100}},
              lineColor={0,0,255},
              textString="%name"),
            Text(
              extent={{0,-70},{0,-110}},
              lineColor={0,0,0},
              textString="R_m.d=%R_m.d, R_m.q=%R_m.q")}),
                                                       Documentation(info="<html>
<p>
The salient reluctance models the relationship between the complex magnetic potential difference
<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/V_m.png\"> and the complex magnetic flux <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Phi.png\">,
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/reluctance.png\">
</p>

<p>which can also be expressed in terms complex phasors:</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/reluctance_alt.png\">
</p>
</html>"));
    end Reluctance;

    model EddyCurrent
    "Constant loss model under sinusoidal magnetic conditions"
      import Modelica.Constants.pi;
      constant Complex j = Complex(0,1);
      extends
      QuasiStationaryFundamentalWave.Interfaces.PartialTwoPortElementary;
      parameter Modelica.SIunits.Conductance G(min=0)
      "Eqivalent symmetric loss conductance";
      extends
      Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPort(
         final T = 273.15);
      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma)
      "Angular velocity";
    equation
      lossPower = (pi/2)*Modelica.ComplexMath.imag(omega*V_m*Modelica.ComplexMath.conj(Phi));
      // Alternative calculaton of loss power
      // lossPower = -(pi/2)*Modelica.ComplexMath.real(j*omega*V_m*Modelica.ComplexMath.conj(Phi));
      if G>0 then
        (pi/2)*V_m = j*omega*G*Phi;
      else
        V_m = Complex(0,0);
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
                -100},{100,100}}),      graphics={
            Rectangle(
              extent={{-70,30},{70,-30}},
              lineColor={255,170,85},
              fillColor={255,170,85},
              fillPattern=FillPattern.Solid),
            Line(points={{-96,0},{-70,0}}, color={255,170,85}),
            Line(points={{70,0},{96,0}}, color={255,170,85}),
            Text(
              extent={{0,60},{0,100}},
              lineColor={0,0,255},
              textString="%name"),
            Text(
              extent={{0,-40},{0,-80}},
              lineColor={0,0,0},
              textString="G=%G")}),                    Documentation(info="<html>
<p>
The eddy current loss model with respect to fundamental wave effects is designed in accordance to
<a href=\"modelica://Modelica.Magnetic.FluxTubes.Basic.EddyCurrent\">FluxTubes.Basic.EddyCurrent</a>.
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/eddycurrent.png\">.
</p>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\">Fig. 1: equivalent models of eddy current losses</caption>
  <tr>
    <td>
      <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/eddycurrent_electric.png\">
    </td>
  </tr>
</table>

<p>Due to the nature of eddy current losses, which can be represented by symmetric
conductors in an equivalent electric circuit (Fig. 1), the respective
number of phases <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/m.png\"> has to be taken into account.
Assume that the <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/m.png\"> conductances
of the equivalent circuit are <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/Gc.png\">,
the conductance for the eddy current loss model is determined by</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/GGc.png\">
</p>

<p>
where <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/N.png\"> is the number of turns of the symmetric electro magnetic coupling.
</p>

<p>For such an <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/m.png\"> phase system
the relationship between the voltage and current <a href=\"http://www.haumer.at/refimg/SpacePhasors.pdf\">space phasors</a>
and the magnetic flux and magnetic potential difference phasor is
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/vPhi\">,<br>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/iV_m.png\">,
</p>

<p>
where <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/v_k\">
and <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/i_k\">
are the phase voltages and currents, respectively.
</p>

<p>
The dissipated loss power
</p>
<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/lossPower.png\">
</p>
<p>
can be determined for the <a href=\"http://www.haumer.at/refimg/SpacePhasors.pdf\">space phasor</a>
relationship of the voltage and current space phasor.
</p>
<h4>See also</h4>

<p><a href=\"modelica://Modelica.Magnetic.FluxTubes.Basic.EddyCurrent\">FluxTubes.Basic.EddyCurrent</a></p>

</html>"),
        Diagram(graphics));
    end EddyCurrent;

    model MultiPhaseElectroMagneticConverter
    "Multi phase electro magnetic converter"
      import Modelica.Constants.pi;
      constant Complex j = Complex(0,1);
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
        plug_p(
        final m=m) "Positive plug"
        annotation (Placement(transformation(
            origin={-100,100},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
        plug_n(
        final m=m) "Negative plug"
        annotation (Placement(transformation(
            origin={-100,-100},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      QuasiStationaryFundamentalWave.Interfaces.PositiveMagneticPort port_p
      "Positive complex magnetic port"
        annotation (Placement(transformation(extent={{90,90},{110,110}}, rotation=0)));
      QuasiStationaryFundamentalWave.Interfaces.NegativeMagneticPort port_n
      "Negative complex magnetic port"
        annotation (Placement(transformation(extent={{90,-110},{110,-90}}, rotation=
               0)));
      parameter Integer m = 3 "Number of phases";
      parameter Real effectiveTurns "Effective number of turns";
      // IMPORTANT NOTE
      // This parameter may be removed in the final version of the library
      // for consistency reasons with resepect to the single phase
      // electromagnetic converter, where the orientation shall NOT be
      // implmented in the final version
      final parameter Modelica.SIunits.Angle orientation = 0
      "Orientation of the first winding axis";
      // Local electric multi phase quantities
      Modelica.SIunits.ComplexVoltage v[m] "Voltage drop";
      Modelica.SIunits.ComplexCurrent i[m] "Current";
      // Local electromagnetic fundamental wave quantities
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma);
      // A technical solution with a rotator cannot be applied to the equations below
      final parameter Complex N=
        effectiveTurns*Modelica.ComplexMath.exp(Complex(0,orientation))
      "Complex effective number of turns";
      Modelica.SIunits.ComplexVoltage vSymmetricalComponent[m] = QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.symmetricTransformationMatrix(m)*v
      "Symmetrical components of voltages";
      Modelica.SIunits.ComplexCurrent iSymmetricalComponent[m] = QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.symmetricTransformationMatrix(m)*i
      "Symmetrical components of currents";
      // NOTE
      // Assert of asymmetric component iSymmetricalComponent[1] <> 0 and
      // iSymmetricalComponent[3] <> 0 have to be included in the future!
  protected
      final parameter Integer indexNonPos[:]=
        QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.indexNonPositiveSequence(m)
      "Indices of all non positive seqeuence componentes";
      final parameter Integer indexPos[:]=
        QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.indexPositiveSequence(m)
      "Indices of all positive seqeuence componentes";
    equation
      // Magnetic flux and flux balance of the magnetic ports
      port_p.Phi = Phi;
      port_p.Phi + port_n.Phi = Complex(0,0);
      // Magnetic potential difference of the magnetic ports
      port_p.V_m - port_n.V_m = V_m;
      // Voltage drop between the electrical plugs
      v = plug_p.pin.v - plug_n.pin.v;
      // Current and current balance of the electric plugs
      i = plug_p.pin.i;
      plug_p.pin.i + plug_n.pin.i = {Complex(0,0) for k in 1:m};
      V_m.re = sqrt(2) * (2.0/pi) * sum(Modelica.ComplexMath.real(N*iSymmetricalComponent[indexPos]))*m/2;
      V_m.im = sqrt(2) * (2.0/pi) * sum(Modelica.ComplexMath.imag(N*iSymmetricalComponent[indexPos]))*m/2;
      for k in 1:size(indexNonPos,1) loop
        iSymmetricalComponent[indexNonPos[k]] = Complex(0,0);
      end for;
      for k in 2:size(indexPos,1) loop
        vSymmetricalComponent[indexPos[1]] = vSymmetricalComponent[indexPos[k]];
      end for;
      // Induced voltages from complex magnetic flux, number of turns
      // and angles of orientation of winding
      -sqrt(2) * Complex(sum(Modelica.ComplexMath.real(vSymmetricalComponent[indexPos])),
                         sum(Modelica.ComplexMath.imag(vSymmetricalComponent[indexPos])))
               = Modelica.ComplexMath.conj(N)*j*omega*Phi;
      // Potential roots are not used; instead the reference angle is handled
      // by means of Connections.branch beteen eletric plug_p and magnetic port_p
      // It has to be checked whether this Modelica compliant
      //   Connections.potentialRoot(plug_p.reference);
      //   Connections.potentialRoot(port_p.reference);
      Connections.branch(port_p.reference, port_n.reference);
      port_p.reference.gamma = port_n.reference.gamma;
      Connections.branch(plug_p.reference, plug_n.reference);
      plug_p.reference.gamma = plug_n.reference.gamma;
      Connections.branch(plug_p.reference, port_p.reference);
      plug_p.reference.gamma = port_p.reference.gamma;
      annotation (         Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-60,60},{58,0}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-58,0},{60,-60}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-60,60},{0,-60}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{100,-100},{94,-100},{84,-98},{76,-94},{64,-86},{50,-72},
                  {42,-58},{36,-40},{30,-18},{30,0},{30,18},{34,36},{46,66},{62,
                  84},{78,96},{90,100},{100,100}}, color={255,170,85}),
            Line(points={{0,60},{-100,60},{-100,100}}, color={85,170,255}),
            Line(points={{0,-60},{-100,-60},{-100,-98}}, color={85,170,255}),
            Text(
              extent={{0,160},{0,120}},
              lineColor={0,0,255},
              fillColor={255,128,0},
              fillPattern=FillPattern.Solid,
              textString="%name")}),
        Documentation(info="<html>

<p>
Each phase <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/k.png\"> of an <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/m.png\"> phase winding has an effective number of turns, <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/effectiveTurns_k.png\"> and an respective winging angle <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/orientation_k.png\"> and a phase current <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/i_k.png\">.
</p>

<p>
The total complex magnetic potential difference of the mutli phase winding is determined by:
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/multiphaseconverter_vm.png\">
</p>

<p>
In this equation each contribution of a winding magneto motive force on the total complex magnetic potential difference is aligned with the respective orientation of the winding.
</p>

<p>
The voltages <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/v_k.png\"> induced in each winding depend on the cosinus between the orientation of the winding and the angle of the complex magnetic flux. Additionally, the magnitudes of the induced voltages are propotional to the respective number of turns. This relationship can be modeled by means of</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/multiphaseconverter_phi.png\">
</p>

<p>for <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/k_in_1_m.png\"> and is also illustrated by the following figure:</p>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig:</b> Orientation of winding and location of complex magnetic flux</caption>
  <tr>
    <td>
      <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/coupling.png\">
    </td>
  </tr>
</table>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.SinglePhaseElectroMagneticConverter\">SinglePhaseElectroMagneticConverter</a>
<p>
</p>
</html>"));
    end MultiPhaseElectroMagneticConverter;

    model QuasiStionaryAnalogElectroMagneticConverter
    "Electro magnetic converter to only (!) quasi stationary analog, neglecting induced voltage"

      // Note: It has not whether the transient voltage induction and the
      //   leakage induction shall be considered in this model or not.
      //   This model is required for electrical excited synchronous machines (SMEE)

      import Modelica.Constants.pi;
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive pin"
        annotation (Placement(transformation(
            origin={-100,100},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative pin"
        annotation (Placement(transformation(
            origin={-100,-100},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Interfaces.PositiveMagneticPort port_p "Positive complex magnetic port"
        annotation (Placement(transformation(extent={{90,90},{110,110}},
              rotation=0)));
      Interfaces.NegativeMagneticPort port_n "Negative complex magnetic port"
        annotation (Placement(transformation(extent={{90,-110},{110,-90}},
              rotation=0)));
      parameter Real effectiveTurns "Effective number of turns"
        annotation (Evaluate=true);
      // Local electric single phase quantities
      Modelica.SIunits.Voltage v "Voltage drop";
      Modelica.SIunits.Current i "Current";
      // Local electromagnetic fundamental wave quantities
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
      Modelica.SIunits.Angle gamma "Angle of V_m fixed reference frame";
      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma);
    equation
      // Magnetic flux and flux balance of the magnetic ports
      port_p.Phi = Phi;
      port_p.Phi + port_n.Phi = Complex(0, 0);
      // Magnetic potential difference of the magnetic ports
      port_p.V_m - port_n.V_m = V_m;
      // Voltage drop between the electrical pins
      v = pin_p.v - pin_n.v;
      // Current and current balance of the electric pins
      i = pin_p.i;
      pin_p.i + pin_n.i = 0;
      // Complex magnetic potential difference is determined from currents, number
      // of turns and angles of orientation of the magnetic potential difference
      V_m = (2/pi)*effectiveTurns*i * Modelica.ComplexMath.fromPolar(1,-gamma);
      // Induced voltages is zero due to quasi stationary electric analog circuit
      v = 0;
      Connections.branch(port_p.reference, port_n.reference);
      port_p.reference.gamma = port_n.reference.gamma;
      // Reference angle to magnetic potential fixed frame
      gamma = port_p.reference.gamma;
      annotation (
        defaultComponentName="converter",
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-60,60},{58,0}},
              lineColor={0,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-58,0},{60,-60}},
              lineColor={0,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-60,60},{0,-60}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{100,-100},{94,-100},{84,-98},{76,-94},{64,-86},{50,-72},
                  {42,-58},{36,-40},{30,-18},{30,0},{30,18},{34,36},{46,66},{62,
                  84},{78,96},{90,100},{100,100}}, color={255,128,0}),
            Line(points={{0,60},{-100,60},{-100,100}}, color={0,0,255}),
            Line(points={{0,-60},{-100,-60},{-100,-98}}, color={0,0,255}),
            Text(
              extent={{0,160},{0,120}},
              lineColor={0,0,255},
              fillColor={255,128,0},
              fillPattern=FillPattern.Solid,
              textString="%name")}),
        Documentation(info="<html>
<p>
The single phase winding has an effective number of turns, <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/effectiveTurns.png\"> and a respective orientation of the winding, <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/orientation.png\">. The current in winding is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/i.png\">.
</p>

<p>
The total complex magnetic potential difference of the single phase winding is determined by:
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/Components/singlephaseconverter_vm.png\">
</p>

<p>
In this equation the magneto motive force is aligned with the orientation of the winding.
</p>

<p>
The voltage <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/v.png\"> induced in the winding depends on the cosine between the orientation of the winding and the angle of the complex magnetic flux. Additionally, the magnitudes of the induced voltages are proportional to the respective number of turns. This relationship can be modeled by means of</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/Components/singlephaseconverter_phi.png\">
</p>

<p>The single phase electromagnetic converter is a special case of the
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics));
    end QuasiStionaryAnalogElectroMagneticConverter;

    model Idle "Idle running branch"
      extends
      QuasiStationaryFundamentalWave.Interfaces.PartialTwoPortElementary;
    equation
      Phi = Complex(0,0);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics={
            Text(
              extent={{0,60},{0,100}},
              lineColor={0,0,255},
              textString="%name"),
            Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-100,0},{-40,0}}, color={255,170,85}),
            Line(points={{40,0},{100,0}}, color={255,170,85})}),
        Documentation(info="<html>
<p>
This is a simple idle running branch.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.Short\">Short</a>
</p>

</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Line(points={{-100,0},{-60,0}}, color={255,128,0}),
            Line(points={{60,0},{100,0}}, color={255,128,0}),
            Line(points={{-60,0},{-40,2},{-18,6},{0,14},{12,26}}, color={255,
                  128,0}),
            Line(points={{60,0},{40,-2},{18,-6},{0,-14},{-12,-26}}, color={255,
                  128,0})}));
    end Idle;

    model Short "Short connection"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
    equation
      connect(port_p, port_n) annotation (Line(points={{-100,5.55112e-16},{-1,
              5.55112e-16},{-1,5.55112e-16},{100,5.55112e-16}}, color={255,128,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{100,100}}), graphics={
            Text(
              extent={{0,60},{0,100}},
              lineColor={0,0,255},
              textString="%name"),
            Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-100,0},{100,0}}, color={255,170,85})}),
        Documentation(info="<html>
<p>
This is a simple short cut branch.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.Idle\">Idle</a>
</p>

</html>"));
    end Short;

    model Crossing "Crossing of connections"

    Interfaces.PositiveMagneticPort port_p1
      annotation (Placement(transformation(extent={{-110,90},{-90,110}})));
    Interfaces.NegativeMagneticPort port_n1
      annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
    Interfaces.PositiveMagneticPort port_p2
      annotation (Placement(transformation(extent={{90,-110},{110,-90}})));
    Interfaces.NegativeMagneticPort port_n2
      annotation (Placement(transformation(extent={{90,90},{110,110}})));
    equation
    connect(port_p1, port_p2) annotation (Line(
        points={{-100,100},{-100,20},{0,20},{0,-20},{100,-20},{100,-100}},
        color={255,170,85},
        smooth=Smooth.None));
    connect(port_n2, port_n1) annotation (Line(
        points={{100,100},{100,0},{-100,0},{-100,-100}},
        color={255,170,85},
        smooth=Smooth.None));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),        graphics={
            Text(
              extent={{0,60},{0,100}},
              lineColor={0,0,255},
              textString="%name"),
            Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
          Line(
            points={{-100,100},{-100,40},{100,-40},{100,-100}},
            color={255,170,85},
            smooth=Smooth.None),
          Line(
            points={{100,100},{100,40},{-100,-40},{-100,-100}},
            color={255,170,85},
            smooth=Smooth.None)}),
        Documentation(info="<html>
<p>
This is a simple short cut branch.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.Idle\">Idle</a>
</p>

</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}),
                        graphics));
    end Crossing;
    annotation (DymolaStoredErrors, Documentation(info="<html>
<p>Basic components of the FundamentalWave library for modeling magnetic circuits. Machine specific components are
located at <a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components\">Machines.Components</a>.</p>
</html>"));
  end Components;


  package BasicMachines "Basic quasi stationary machine models"
    extends Modelica.Icons.Package;
    package AsynchronousInductionMachines
    "Quasi stationary asynchronous induction machines"
      extends Modelica.Icons.Package;
      model AIM_SquirrelCage
      "Asynchronous induction machine with squirrel cage"
        // Removed form extension of FUNDAMENTAL WAVE model: is(start=zeros(m)) ##
        extends
        QuasiStationaryFundamentalWave.Interfaces.PartialBasicInductionMachine(
          Rs(start=0.03),
          Lssigma(start=3*(1 - sqrt(1 - 0.0667))/(2*pi*fsNominal)),
          final L0(d=2.0*Lm/m/effectiveStatorTurns^2, q=2.0*Lm/m/
                effectiveStatorTurns^2),
          redeclare final
          Modelica.Electrical.Machines.Thermal.AsynchronousInductionMachines.ThermalAmbientAIMC
            thermalAmbient(final Tr=TrOperational),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC
            thermalPort,
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMC
            internalThermalPort,
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.PowerBalanceAIMC
            powerBalance(final lossPowerRotorWinding=sum(rotorCage.resistor.resistor.LossPower),
              final lossPowerRotorCore=0));
        parameter Modelica.SIunits.Inductance Lm(start=3*sqrt(1 - 0.0667)/(2*pi
              *fsNominal)) "Stator main field inductance"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lrsigma(start=3*(1 - sqrt(1 -
              0.0667))/(2*pi*fsNominal))
        "Rotor leakage inductance of equivalent m phase winding w.r.t. stator side"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Resistance Rr(start=0.04)
        "Rotor resistance of equivalent m phase winding w.r.t. stator side"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Temperature TrRef(start=293.15)
        "Reference temperature of rotor resistance"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20r(start=0)
        "Temperature coefficient of rotor resistance at 20 degC"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Temperature TrOperational(start=293.15)
        "Operational temperature of rotor resistance"   annotation (Dialog(
              group="Operational temperatures", enable=not useThermalPort));
        output Modelica.SIunits.ComplexCurrent ir[m] = rotorCage.winding.plug_p.pin.i
        "Rotor cage currents";
        Components.SymmetricMultiPhaseCageWinding
          rotorCage(
          final Lsigma=Lrsigma,
          final effectiveTurns=effectiveStatorTurns,
          final useHeatPort=true,
          final RRef=Rr,
          final TRef=TrRef,
          final TOperational=TrRef,
          final m=m,
          final alpha20=alpha20r)
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (Placement(transformation(extent={{-10,-40},{10,-20}},
                rotation=0)));
      equation
        connect(rotorCage.heatPortWinding, internalThermalPort.heatPortRotorWinding)
          annotation (Line(
            points={{-6.10623e-16,-40},{-40,-40},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(airGap.port_rn, rotorCage.port_p) annotation (Line(
            points={{-10,-10},{-10,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(airGap.port_rp, rotorCage.port_n) annotation (Line(
            points={{10,-10},{10,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        annotation (
          defaultComponentName="aimc",
          Documentation(info="<html>
<p>
Resistances and stray inductances of the machine refer to an <code>m</code> phase stator. The symmetry of the stator and rotor is assumed. The machine models take the following loss effects into account:
</p>

<ul>
<li>heat losses in the temperature dependent stator winding resistances</li>
<li>heat losses in the temperature dependent cage resistances</li>
<li>friction losses</li>
<li>core losses (only eddy current losses, no hysteresis losses)</li>
<li>stray load losses</li>
</ul>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing\">AIM_SlipRing</a>,
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end AIM_SquirrelCage;

      model AIM_SlipRing "Asynchronous induction machine with slip ring rotor"
        parameter Integer mr(min=3) = m "Number of rotor phases";
        extends
        QuasiStationaryFundamentalWave.Interfaces.PartialBasicInductionMachine(
          Rs(start=0.03),
          Lssigma(start=3*(1 - sqrt(1 - 0.0667))/(2*pi*fsNominal)),
          final L0(d=2.0*Lm/m/effectiveStatorTurns^2, q=2.0*Lm/m/
                effectiveStatorTurns^2),
          redeclare final
          Modelica.Electrical.Machines.Thermal.AsynchronousInductionMachines.ThermalAmbientAIMS
            thermalAmbient(final Tr=TrOperational, final mr=mr),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMS
            thermalPort(final mr=mr),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortAIMS
            internalThermalPort(final mr=mr),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.PowerBalanceAIMS
            powerBalance(final lossPowerRotorWinding = sum(rotor.resistor.resistor.LossPower),
              final lossPowerRotorCore = rotor.core.lossPower,
              final lossPowerBrush=0,
              final powerRotor=QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.activePower(vr, ir)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
                                                               plug_rn(final m=
              mr) "Negative plug of rotor" annotation (Placement(transformation(
                extent={{-110,-50},{-90,-70}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
                                                               plug_rp(final m=
              mr) "Positive plug of rotor" annotation (Placement(transformation(
                extent={{-110,70},{-90,50}}, rotation=0)));
        parameter Modelica.SIunits.Inductance Lm(start=3*sqrt(1 - 0.0667)/(2*pi
              *fsNominal)) "Stator main field inductance"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lrsigma(start=3*(1 - sqrt(1 -
              0.0667))/(2*pi*fsNominal))
        "Rotor leakage inductance w.r.t. rotor side"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Resistance Rr(start=0.04)
        "Rotor resistance per phase w.r.t. rotor side"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Temperature TrRef(start=293.15)
        "Reference temperature of rotor resistance"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20r(start=0)
        "Temperature coefficient of rotor resistance at 20 degC"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Temperature TrOperational(start=293.15)
        "Operational temperature of rotor resistance"   annotation (Dialog(
              group="Operational temperatures", enable=not useThermalPort));
        parameter Boolean useTurnsRatio(start=true)
        "Use TurnsRatio or calculate from locked-rotor voltage?";
        parameter Real TurnsRatio(final min=Modelica.Constants.small, start=1)
        "Effective number of stator turns / effective number of rotor turns"
          annotation (Dialog(enable=useTurnsRatio));
        parameter Modelica.SIunits.Voltage VsNominal(start=100)
        "Nominal stator voltage per phase"
          annotation (Dialog(enable=not useTurnsRatio));
        parameter Modelica.SIunits.Voltage VrLockedRotor(start=100*(2*pi*
              fsNominal*Lm)/sqrt(Rs^2 + (2*pi*fsNominal*(Lm + Lssigma))^2))
        "Locked rotor voltage per phase"
          annotation (Dialog(enable=not useTurnsRatio));
        parameter Modelica.Electrical.Machines.Losses.CoreParameters
          rotorCoreParameters(
          final m=3,
          PRef=0,
          VRef(start=1) = 1,
          wRef(start=1) = 1)
        "Rotor core losses, all quantities refer to rotor side"
          annotation (Dialog(tab="Losses"));
        output Modelica.SIunits.ComplexVoltage vr[mr]=plug_rp.pin.v - plug_rn.pin.v
        "Rotor voltages";
        output Modelica.SIunits.ComplexCurrent ir[mr]=plug_rp.pin.i
        "Rotor currents";
    protected
        final parameter Real internalTurnsRatio=if useTurnsRatio then
            TurnsRatio else VsNominal/VrLockedRotor*(2*pi*fsNominal*Lm)/sqrt(Rs
            ^2 + (2*pi*fsNominal*(Lm + Lssigma))^2);
    public
        Components.SymmetricMultiPhaseWinding
          rotor(
          final Lsigma=Lrsigma,
          final effectiveTurns=effectiveStatorTurns/internalTurnsRatio,
          final useHeatPort=true,
          final RRef=Rr,
          final TRef=TrRef,
          final TOperational=TrOperational,
          final GcRef=rotorCoreParameters.GcRef,
          final m=mr,
          final alpha20=alpha20r)
        "Symmetric rotor winding including resistances, zero and stray inductances and zero core losses"
          annotation (Placement(transformation(
              origin={0,-40},
              extent={{-10,-10},{10,10}},
              rotation=90)));
      equation
        connect(rotor.plug_n, plug_rn) annotation (Line(points={{10,-50},{10,-60},{-100,
                -60}}, color={85,170,255}));
        connect(rotor.heatPortCore, internalThermalPort.heatPortRotorCore)
          annotation (Line(
            points={{10,-36},{20,-36},{20,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(rotor.heatPortWinding, internalThermalPort.heatPortRotorWinding)
          annotation (Line(
            points={{10,-44},{20,-44},{20,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(plug_rp, rotor.plug_p) annotation (Line(
            points={{-100,60},{-80,60},{-80,-50},{-10,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(airGap.port_rn, rotor.port_p) annotation (Line(
            points={{-10,-10},{-10,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(airGap.port_rp, rotor.port_n) annotation (Line(
            points={{10,-10},{10,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        annotation (
          defaultComponentName="aims",
          Icon(graphics={Line(points={{-100,50},{-100,20},{-60,20}}, color={85,170,255}),
                           Line(points={{-100,-50},{-100,-20},{-60,-20}}, color={85,170,
                    255})}),
          Documentation(info="<html>
<p>
Resistances and stray inductances of the machine always refer to either stator or rotor. The symmetry of the stator and rotor is assumed. The number of stator and rotor phases may be different. The machine models take the following loss effects into account:
</p>

<ul>
<li>heat losses in the temperature dependent stator winding resistances</li>
<li>heat losses in the temperature dependent rotor winding resistances</li>
<li>friction losses</li>
<li>stator and rotor core losses (only eddy current losses, no hysteresis losses)</li>
<li>stray load losses</li>
</ul>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage\">AIM_SquirrelCage</a>,
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),            graphics));
      end AIM_SlipRing;
    end AsynchronousInductionMachines;

    package SynchronousInductionMachines
    "Quasi stationary synchronous induction machines"
    extends Modelica.Icons.Package;
      model SM_PermanentMagnet
      "Permanent magnet synchronous machine with optional damper cage"
        extends
        QuasiStationaryFundamentalWave.Interfaces.PartialBasicInductionMachine(
          Rs(start=0.03),
          Lssigma(start=0.1/(2*pi*fsNominal)),
          final L0(d=2.0*Lmd/m/effectiveStatorTurns^2, q=2.0*Lmq/m/
                effectiveStatorTurns^2),
          redeclare final
          Modelica.Electrical.Machines.Thermal.SynchronousInductionMachines.ThermalAmbientSMPM
            thermalAmbient(
              final useDamperCage=useDamperCage,
              final Tr=TrOperational,
              final Tpm=TpmOperational),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMPM
            thermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMPM
            internalThermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.PowerBalanceSMPM
            powerBalance(
            final lossPowerRotorWinding=damperCageLossPower,
            final lossPowerRotorCore=0,
            final lossPowerPermanentMagnet=permanentMagnet.lossPower));
        parameter Modelica.SIunits.Inductance Lmd(start=0.3/(2*pi*fsNominal))
        "Stator main field inductance, d-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq(start=0.3/(2*pi*fsNominal))
        "Stator main field inductance, q-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        // Rotor cage parameters
        parameter Boolean useDamperCage(start=true)
        "Enable/disable damper cage"
           annotation (Dialog(tab=
                "Nominal resistances and inductances", group="Damper cage"));
        parameter Modelica.SIunits.Inductance Lrsigmad(start=0.05/(2*pi*
              fsNominal))
        "Rotor leakage inductance, d-axis, w.r.t. stator side"
            annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq=Lrsigmad
        "Rotor leakage inductance, q-axis, w.r.t. stator side"   annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd(start=0.04)
        "Rotor resistance, d-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq=Rrd
        "Rotor resistance , q-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Temperature TrRef(start=293.15)
        "Reference temperature of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20r(start=0)
        "Temperature coefficient of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="Damper cage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Voltage VsOpenCircuit(start=112.3)
        "Open circuit RMS voltage per phase @ fsNominal";
        final parameter Modelica.SIunits.Temperature TpmOperational=293.15
        "Operational temperature of permanent magnet"
           annotation(Dialog(group="Operational temperatures"));
        parameter Modelica.SIunits.Temperature TrOperational(start=293.15)
        "Operational temperature of (optional) damper cage"   annotation (
            Dialog(group="Operational temperatures",
                   enable=not useThermalPort and useDamperCage));
        parameter
        Modelica.Electrical.Machines.Losses.PermanentMagnetLossParameters
          permanentMagnetLossParameters(IRef(start=100), wRef(start=2*pi*
                fsNominal/p)) "Permanent magnet loss losses"
          annotation (Dialog(tab="Losses"));
      // Removed in QS implementation
      //   Modelica.Blocks.Interfaces.Output ir[2](
      //     start=zeros(2),
      //     each final quantity="ElectricCurrent", each final unit="A") if useDamperCage
      //     "Damper cage currents" annotation(Dialog(showStartAttribute=true));
        QuasiStationaryFundamentalWave.Components.Short    short if not
          useDamperCage
        "Magnetic connection in case the damper cage is not present"
          annotation (Placement(transformation(
              origin={10,-40},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        Components.SaliencyCageWinding
          rotorCage(
          final RRef(d=Rrd, q=Rrq),
          final Lsigma(d=Lrsigmad, q=Lrsigmaq),
          final effectiveTurns=sqrt(3.0/2.0)*effectiveStatorTurns,
          final useHeatPort=true,
          final TRef=TrRef,
          final alpha20=alpha20r,
          final TOperational=TrOperational) if useDamperCage
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={20,-40})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.PermanentMagnet
                                                                                   permanentMagnet(
          final V_m=Complex(V_mPM, 0),
          final m=m,
          final permanentMagnetLossParameters=permanentMagnetLossParameters,
          final useHeatPort=true,
          final is=is) "Magnetic potential difference of permanent magnet"
                                                              annotation (
            Placement(transformation(
              origin={-10,-40},
              extent={{-10,-10},{10,10}},
              rotation=270)));
    protected
        final parameter Modelica.SIunits.MagneticPotentialDifference V_mPM=
           (2/pi)*sqrt(2)*(m/2)*VsOpenCircuit/effectiveStatorTurns/
           (Lmd/effectiveStatorTurns^2*2*pi*fsNominal)
        "Equivalent excitation magnetic potential difference";
        Modelica.Blocks.Interfaces.RealOutput damperCageLossPower(
          final quantity="Power", final unit="W") "Damper losses";
      equation
        connect(damperCageLossPower, rotorCage.lossPower);
        if not useDamperCage then
          damperCageLossPower=0;
        end if;
        connect(permanentMagnet.port_p, airGap.port_rn) annotation (Line(
            points={{-10,-30},{-10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(permanentMagnet.support, airGap.support) annotation (Line(
            points={{-20,-40},{-50,-40},{-50,0},{-10,0}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(permanentMagnet.heatPort, internalThermalPort.heatPortPermanentMagnet)
          annotation (Line(
            points={{-20,-30},{-40,-30},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(permanentMagnet.flange, inertiaRotor.flange_b) annotation (Line(
            points={{0,-40},{0,-20},{90,-20},{90,-1.33227e-015}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(airGap.port_rp, rotorCage.port_n) annotation (Line(
            points={{10,-10},{10,-30},{20,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_n, airGap.port_rp) annotation (Line(
            points={{10,-30},{10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_p, permanentMagnet.port_n) annotation (Line(
            points={{10,-50},{-10,-50}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.port_p, permanentMagnet.port_n) annotation (Line(
            points={{20,-50},{-10,-50}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.heatPortWinding, internalThermalPort.heatPortRotorWinding)
          annotation (Line(
            points={{30,-40},{40,-40},{40,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        annotation (
          defaultComponentName="smpm",
          Icon(graphics={
              Rectangle(
                extent={{-130,10},{-100,-10}},
                lineColor={0,0,0},
                fillColor={0,255,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,10},{-70,-10}},
                lineColor={0,0,0},
                fillColor={255,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(extent={{-134,34},{-66,-34}}, lineColor={85,170,255})}),
          Documentation(info="<html>
<p>
Resistances and stray inductances of the machine refer to an <code>m</code> phase stator. The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. The machine models take the following loss effects into account:
</p>

<ul>
<li>heat losses in the temperature dependent stator winding resistances</li>
<li>optional, when enabled: heat losses in the temperature dependent damper cage resistances</li>
<li>friction losses</li>
<li>core losses (only eddy current losses, no hysteresis losses)</li>
<li>stray load losses</li>
<li>permanent magnet losses</li>
</ul>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited\">SM_ElectricalExcited</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor\">SM_ReluctanceRotor</a>,
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),            graphics));
      end SM_PermanentMagnet;

      model SM_ElectricalExcited
      "Electrical excited synchronous machine with optional damper cage"
        extends
        QuasiStationaryFundamentalWave.Interfaces.PartialBasicInductionMachine(
          Rs(start=0.03),
          Lssigma(start=0.1/(2*pi*fsNominal)),
          final L0(d=2.0*Lmd/m/effectiveStatorTurns^2, q=2.0*Lmq/m/
                effectiveStatorTurns^2),
          redeclare final
          Modelica.Electrical.Machines.Thermal.SynchronousInductionMachines.ThermalAmbientSMEE
            thermalAmbient(
            final useDamperCage=useDamperCage,
            final Te=TeOperational,
            final Tr=TrOperational),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMEE
            thermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMEE
            internalThermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.PowerBalanceSMEE
            powerBalance(
            final lossPowerRotorWinding=damperCageLossPower,
            final powerExcitation=0,
            final lossPowerExcitation=excitation.resistor.LossPower,
            final lossPowerBrush=brush.lossPower,
            final lossPowerRotorCore=0));
        parameter Modelica.SIunits.Inductance Lmd(start=1.5/(2*pi*fsNominal))
        "Stator main field inductance, d-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq(start=1.5/(2*pi*fsNominal))
        "Stator main field inductance, q-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        // Rotor cage parameters
        parameter Boolean useDamperCage(start=true)
        "Enable/disable damper cage"   annotation (Dialog(tab=
                "Nominal resistances and inductances", group="DamperCage"));
        parameter Modelica.SIunits.Inductance Lrsigmad(start=0.05/(2*pi*
              fsNominal))
        "Rotor leakage inductance, d-axis, w.r.t. stator side"   annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq=Lrsigmad
        "Rotor leakage inductance, q-axis, w.r.t. stator side"   annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd(start=0.04)
        "Rotor resistance, d-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq=Rrd
        "Rotor resistance , q-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Temperature TrRef(start=293.15)
        "Reference temperature of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20r(start=0)
        "Temperature coefficient of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        // Operational temperature
        parameter Modelica.SIunits.Temperature TrOperational(start=293.15)
        "Operational temperature of (optional) damper cage"   annotation (
            Dialog(group="Operational temperatures", enable=not useThermalPort
                 and useDamperCage));
        parameter Modelica.SIunits.Temperature TeOperational(start=293.15)
        "Operational excitation temperature"   annotation (Dialog(group=
                "Operational temperatures", enable=not useThermalPort));
        // Excitation parameters
        parameter Modelica.SIunits.Voltage VsNominal(start=100)
        "Nominal stator voltage"   annotation (Dialog(tab="Excitation"));
        parameter Modelica.SIunits.Current IeOpenCircuit(start=10)
        "Open circuit excitation current @ nominal voltage and frequency"
          annotation (Dialog(tab="Excitation"));
        parameter Modelica.SIunits.Resistance Re(start=2.5)
        "Warm excitation resistance"   annotation (Dialog(tab="Excitation"));
        parameter Modelica.SIunits.Temperature TeRef(start=293.15)
        "Reference temperture of excitation resistance"
          annotation (Dialog(tab="Excitation"));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20e(start=0) "Temperature coefficient of excitation resistance"
          annotation (Dialog(tab="Excitation"));
        parameter Modelica.Electrical.Machines.Losses.BrushParameters
          brushParameters "Brush losses" annotation (Dialog(tab="Losses"));
        output Modelica.SIunits.Voltage ve=pin_ep.v - pin_en.v
        "Excitation voltage";
        output Modelica.SIunits.Current ie=pin_ep.i "Excitation current";
      // Re-insert in final version ##
      //   Modelica.ComplexBlocks.Interfaces.ComplexOutput ir[2](
      //      each final quantity="ElectricCurrent", each final unit="A") if useDamperCage
      //     "Damper cage currents" annotation(Dialog(showStartAttribute=true));
        QuasiStationaryFundamentalWave.Components.Short short if not useDamperCage
        "Magnetic connection in case the damper cage is not present"
          annotation (Placement(transformation(
              origin={10,-40},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        QuasiStationaryFundamentalWave.BasicMachines.Components.SaliencyCageWinding
                                                                                    rotorCage(
          final Lsigma(d=Lrsigmad, q=Lrsigmaq),
          final effectiveTurns=sqrt(3.0/2.0)*effectiveStatorTurns,
          final useHeatPort=true,
          final TRef=TrRef,
          final TOperational=TrOperational,
          final RRef(d=Rrd, q=Rrq),
          final alpha20=alpha20r) if useDamperCage
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={20,-40})));
        QuasiStationaryFundamentalWave.BasicMachines.Components.QuasiStionaryAnalogWinding
          excitation(
          final RRef=Re,
          final TRef=TeRef,
          final effectiveTurns=effectiveStatorTurns*turnsRatio*m/2,
          final useHeatPort=true,
          final TOperational=TeOperational,
          final alpha20=alpha20e)
        "Excitation winding including resistance and stray inductance"
          annotation (Placement(transformation(extent={{-30,-50},{-10,-30}},
                rotation=0)));
    protected
        final parameter Real turnsRatio=sqrt(2)*VsNominal/(2*pi*fsNominal*Lmd*IeOpenCircuit)
        "Stator current / excitation current";
        Modelica.Blocks.Interfaces.RealOutput damperCageLossPower(
          final quantity="Power", final unit="W") "Damper losses";
    public
        Modelica.Electrical.Machines.Losses.DCMachines.Brush brush(final
            brushParameters=brushParameters, final useHeatPort=true)
              annotation (Placement(
              transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-80,40})));
        Modelica.Electrical.Analog.Interfaces.PositivePin pin_ep
        "Positive pin of excitation"   annotation (Placement(transformation(
                extent={{-110,70},{-90,50}}, rotation=0)));
        Modelica.Electrical.Analog.Interfaces.NegativePin pin_en
        "Negative pin of excitation"   annotation (Placement(transformation(
                extent={{-90,-50},{-110,-70}}, rotation=0)));
      equation
        connect(damperCageLossPower, rotorCage.lossPower);
        if not useDamperCage then
          damperCageLossPower=0;
        end if;
        connect(airGap.port_rn, excitation.port_p) annotation (Line(
            points={{-10,-10},{-10,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(excitation.heatPortWinding, internalThermalPort.heatPortExcitation)
          annotation (Line(
            points={{-20,-50},{-20,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(airGap.port_rp, rotorCage.port_n) annotation (Line(
            points={{10,-10},{10,-30},{20,-30}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_n, airGap.port_rp) annotation (Line(
            points={{10,-30},{10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.port_p, excitation.port_n) annotation (Line(
            points={{20,-50},{-10,-50}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_p, excitation.port_n) annotation (Line(
            points={{10,-50},{-10,-50}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.heatPortWinding, internalThermalPort.heatPortRotorWinding)
          annotation (Line(
            points={{30,-40},{40,-40},{40,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(pin_ep,brush. p) annotation (Line(
            points={{-100,60},{-80,60},{-80,50}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(brush.heatPort, internalThermalPort.heatPortBrush) annotation (
            Line(
            points={{-70,50},{-40,50},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(excitation.pin_n,pin_en)  annotation (Line(
            points={{-30,-50},{-80,-50},{-80,-60},{-100,-60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(excitation.pin_p,brush. n) annotation (Line(
            points={{-30,-30},{-80,-30},{-80,30}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          defaultComponentName="smee",
          Icon(graphics={
              Ellipse(extent={{-134,34},{-66,-34}}, lineColor={85,170,255}),
              Line(points={{-100,50},{-100,20},{-130,20},{-130,-4}}, color={0,0,
                    255}),
              Line(points={{-130,-4},{-129,1},{-125,5},{-120,6},{-115,5},{-111,
                    1},{-110,-4}}, color={0,0,255}),
              Line(points={{-110,-4},{-109,1},{-105,5},{-100,6},{-95,5},{-91,1},
                    {-90,-4}}, color={0,0,255}),
              Line(points={{-90,-4},{-89,1},{-85,5},{-80,6},{-75,5},{-71,1},{-70,
                    -4}}, color={0,0,255}),
              Line(points={{-100,-50},{-100,-20},{-70,-20},{-70,-2}}, color={0,
                    0,255})}),
          Documentation(info="<html>
<p>
The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. The machine models take the following loss effects into account:
</p>

<ul>
<li>heat losses in the temperature dependent stator winding resistances</li>
<li>heat losses in the temperature dependent excitation winding resistance</li>
<li>optional, when enabled: heat losses in the temperature dependent damper cage resistances</li>
<li>brush losses in the excitation circuit</li>
<li>friction losses</li>
<li>core losses (only eddy current losses, no hysteresis losses)</li>
<li>stray load losses</li>
</ul>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet\">SM_PermanentMagnet</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor\">SM_ReluctanceRotor</a>,
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}),            graphics));
      end SM_ElectricalExcited;

      model SM_ReluctanceRotor "Reluctance machine with optional damper cage"
        extends
        QuasiStationaryFundamentalWave.Interfaces.PartialBasicInductionMachine(
          Rs(start=0.03),
          Lssigma(start=0.1/(2*pi*fsNominal)),
          final L0(d=2.0*Lmd/m/effectiveStatorTurns^2, q=2.0*Lmq/m/effectiveStatorTurns^2),
          redeclare final
          Modelica.Electrical.Machines.Thermal.SynchronousInductionMachines.ThermalAmbientSMR
            thermalAmbient(final useDamperCage=useDamperCage, final Tr=
                TrOperational),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMR
            thermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.ThermalPortSMR
            internalThermalPort(final useDamperCage=useDamperCage),
          redeclare final
          Modelica.Electrical.Machines.Interfaces.InductionMachines.PowerBalanceSMR
            powerBalance(final lossPowerRotorWinding=damperCageLossPower,
              final lossPowerRotorCore=0));
        parameter Modelica.SIunits.Temperature TrOperational(start=293.15)
        "Operational temperature of (optional) damper cage"   annotation (
            Dialog(group="Operational temperatures", enable=not useThermalPort
                 and useDamperCage));
        parameter Modelica.SIunits.Inductance Lmd(start=2.9/(2*pi*fsNominal))
        "Stator main field inductance, d-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq(start=0.9/(2*pi*fsNominal))
        "Stator main field inductance, q-axis"
          annotation (Dialog(tab="Nominal resistances and inductances"));
        // Rotor cage parameters
        parameter Boolean useDamperCage(start=true)
        "Enable/disable damper cage"   annotation (Dialog(tab=
                "Nominal resistances and inductances", group="DamperCage"));
        parameter Modelica.SIunits.Inductance Lrsigmad(start=0.05/(2*pi*
              fsNominal))
        "Rotor leakage inductance, d-axis, w.r.t. stator side"   annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq=Lrsigmad
        "Rotor leakage inductance, q-axis, w.r.t. stator side"   annotation (
            Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd(start=0.04)
        "Rotor resistance, d-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq=Rrd
        "Rotor resistance , q-axis, w.r.t. stator side"   annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter Modelica.SIunits.Temperature TrRef(start=293.15)
        "Reference temperature of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20r(start=0)
        "Temperature coefficient of damper resistances in d- and q-axis"
          annotation (Dialog(
            tab="Nominal resistances and inductances",
            group="DamperCage",
            enable=useDamperCage));
      // Re-implement in final version ##
      //   Modelica.ComplexBlocks.Interfaces.ComplexOutput ir[2](
      //     each final quantity="ComplexCurrent", each final unit="A") if useDamperCage
      //     "Damper cage currents" annotation(Dialog(showStartAttribute=true));
        QuasiStationaryFundamentalWave.Components.Short    short if not
          useDamperCage
        "Magnetic connection in case the damper cage is not present"
          annotation (Placement(transformation(
              origin={10,-40},
              extent={{10,10},{-10,-10}},
              rotation=270)));
        Components.SaliencyCageWinding
          rotorCage(
          final RRef(d=Rrd, q=Rrq),
          final Lsigma(d=Lrsigmad, q=Lrsigmaq),
          final effectiveTurns=sqrt(3.0/2.0)*effectiveStatorTurns,
          final useHeatPort=true,
          final TRef=TrRef,
          final alpha20=alpha20r,
          final TOperational=TrOperational) if useDamperCage
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={20,-40})));
    protected
        Modelica.Blocks.Interfaces.RealOutput damperCageLossPower(
          final quantity="Power", final unit="W") "Damper losses";
      equation
        connect(damperCageLossPower, rotorCage.lossPower);
        if not useDamperCage then
          damperCageLossPower=0;
        end if;
        connect(rotorCage.port_n, airGap.port_rp) annotation (Line(
            points={{20,-30},{10,-30},{10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_n, airGap.port_rp) annotation (Line(
            points={{10,-30},{10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.port_p, airGap.port_rn) annotation (Line(
            points={{20,-50},{-10,-50},{-10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(short.port_p, airGap.port_rn) annotation (Line(
            points={{10,-50},{-10,-50},{-10,-10}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(rotorCage.heatPortWinding, internalThermalPort.heatPortRotorWinding)
          annotation (Line(
            points={{30,-40},{40,-40},{40,-80},{-40,-80},{-40,-90}},
            color={191,0,0},
            smooth=Smooth.None));
        annotation (
          defaultComponentName="smr",
          Icon(graphics={
              Rectangle(extent={{-130,10},{-100,-10}}, lineColor={0,0,0}),
              Rectangle(extent={{-100,10},{-70,-10}}, lineColor={0,0,0}),
              Ellipse(extent={{-134,34},{-66,-34}}, lineColor={85,170,255})}),
          Documentation(info="<html>
<p>
The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. The machine models take the following loss effects into account:
</p>

<ul>
<li>heat losses in the temperature dependent stator winding resistances</li>
<li>optional, when enabled: heat losses in the temperature dependent damper cage resistances</li>
<li>friction losses</li>
<li>core losses (only eddy current losses, no hysteresis losses)</li>
<li>stray load losses</li>
</ul>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited\">SM_ElectricalExcited</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet\">SM_PermanentMagnet</a>,
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end SM_ReluctanceRotor;
    end SynchronousInductionMachines;

    package Components "Components for quasi stationary machine models"
    extends Modelica.Icons.Package;
      model SymmetricMultiPhaseWinding
      "Symmetric winding model coupling electrical and magnetic domain"
      import QuasiStationaryFundamentalWave;
        // Orientation changed
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
                                                               plug_p(final m=m)
        "Positive plug"   annotation (Placement(transformation(
              origin={-100,100},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
                                                               plug_n(final m=m)
        "Negative plug"   annotation (Placement(transformation(
              origin={-100,-100},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Interfaces.NegativeMagneticPort
          port_n "Negative complex magnetic port" annotation (Placement(
              transformation(extent={{90,-110},{110,-90}}, rotation=0)));
        Interfaces.PositiveMagneticPort
          port_p "Positive complex magnetic port" annotation (Placement(
              transformation(extent={{90,90},{110,110}}, rotation=0)));
        parameter Integer m=3 "Number of phases";
        parameter Boolean useHeatPort=false
        "Enable / disable (=fixed temperatures) thermal port"
          annotation (Evaluate=true);
        // Resistor model
        parameter Modelica.SIunits.Resistance RRef
        "Winding resistance per phase at TRef";
        parameter Modelica.SIunits.Temperature TRef(start=293.15)
        "Reference temperature of winding";
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20(start=0) "Temperature coefficient of winding at 20 degC";
        final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=
            Modelica.Electrical.Machines.Thermal.convertAlpha(
                  alpha20,
                  TRef,
                  293.15);
        parameter Modelica.SIunits.Temperature TOperational(start=293.15)
        "Operational temperature of winding"
          annotation (Dialog(enable=not useHeatPort));
        parameter Modelica.SIunits.Inductance Lsigma
        "Winding stray inductance per phase";
        parameter Real effectiveTurns=1 "Effective number of turns per phase";
        parameter Modelica.SIunits.Conductance GcRef
        "Electrical reference core loss reluctance";
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          electroMagneticConverter(
          final m=m, final effectiveTurns=effectiveTurns)                                     annotation (
            Placement(transformation(extent={{-10,-40},{10,-20}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor
                                                      resistor(
          final m=m,
          final useHeatPort=useHeatPort,
          final T_ref=fill(TRef, m),
          final T=fill(TOperational, m),
        R_ref=fill(RRef, m),
        final alpha_ref=fill(alphaRef, m)) "Winding resistor"
                                                            annotation (
            Placement(transformation(
              origin={-18,70},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortWinding[m] if
        useHeatPort "Heat ports of winding resistors"
          annotation (Placement(transformation(extent={{-50,-110},{-30,-90}})));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortCore if
        useHeatPort "Heat ports of winding resistor"
          annotation (Placement(transformation(extent={{30,-110},{50,-90}})));
        QuasiStationaryFundamentalWave.Components.EddyCurrent    core(final
            useHeatPort=useHeatPort, final G=(m/2)*GcRef*effectiveTurns^2)
        "Core loss model (currently eddy currents only)"   annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={50,-40})));
        QuasiStationaryFundamentalWave.Components.Reluctance    strayReluctance(
            final R_m(d=m*effectiveTurns^2/2/Lsigma, q=m*effectiveTurns^2/2/
                Lsigma))
        "Stray reluctance equivalent to ideally coupled stray inductances"
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={80,30})));
      equation
        connect(resistor.heatPort, heatPortWinding) annotation (Line(
            points={{-28,70},{-40,70},{-40,-100}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(core.heatPort, heatPortCore) annotation (Line(
            points={{40,-50},{40,-100}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(strayReluctance.port_n, core.port_n) annotation (Line(
            points={{80,20},{80,-40},{60,-40}},
            color={255,128,0},
            smooth=Smooth.None));
      connect(electroMagneticConverter.plug_p, resistor.plug_n) annotation (
          Line(
          points={{-10,-20},{-18,-20},{-18,60}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(plug_n, electroMagneticConverter.plug_n) annotation (Line(
          points={{-100,-100},{-100,-40},{-10,-40}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(plug_p, resistor.plug_p) annotation (Line(
          points={{-100,100},{-18,100},{-18,80}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(port_p, electroMagneticConverter.port_p) annotation (Line(
          points={{100,100},{10,100},{10,-20}},
          color={255,170,85},
          smooth=Smooth.None));
      connect(strayReluctance.port_p, port_p) annotation (Line(
          points={{80,40},{80,100},{100,100}},
          color={255,170,85},
          smooth=Smooth.None));
      connect(port_n, core.port_n) annotation (Line(
          points={{100,-100},{100,-40},{60,-40}},
          color={255,170,85},
          smooth=Smooth.None));
      connect(electroMagneticConverter.port_n, core.port_p) annotation (Line(
          points={{10,-40},{40,-40}},
          color={255,170,85},
          smooth=Smooth.None));
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics={
              Rectangle(
                extent={{-100,60},{100,-60}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{100,-100},{94,-100},{84,-98},{76,-94},{64,-86},{50,
                    -72},{42,-58},{36,-40},{30,-18},{30,0},{30,18},{34,36},{46,
                    66},{62,84},{78,96},{90,100},{100,100}}, color={255,128,0}),
              Line(points={{40,60},{-100,60},{-100,100}}, color={85,170,255}),
              Line(points={{40,-60},{-100,-60},{-100,-98}}, color={85,170,255}),
              Line(points={{40,60},{100,20},{40,-20},{0,-20},{-40,0},{0,20},{40,
                  20},{100,-20},{40,-60}}, color={85,170,255}),
              Text(
                extent={{0,160},{0,120}},
                lineColor={0,0,255},
                fillColor={255,128,0},
                fillPattern=FillPattern.Solid,
                textString="%name")}),
          Documentation(info="<html>
<p>
The symmetrical multi phase winding consists of a symmetrical winding
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.Resistor\">resistor</a>, a
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.MutualInductor\">zero</a> and
<a href=\"modelica://Modelica.Electrical.MultiPhase.Basic.Inductor\">stray inductor</a> as well as a symmetrical
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">multi phase electromagnetic coupling</a> and a
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.EddyCurrent\">core loss</a> model including
heat <a href=\"modelica://Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a\">port</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseCageWinding\">SymmetricMultiPhaseCageWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SaliencyCageWinding\">SaliencyCageWinding</a>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics));
      end SymmetricMultiPhaseWinding;

      model QuasiStionaryAnalogWinding
      "Quasi stationary single phase winding neglecting induced voltage"
        Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive pin"
          annotation (Placement(transformation(
              origin={-100,100},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative pin"
          annotation (Placement(transformation(
              origin={-100,-100},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        Interfaces.NegativeMagneticPort
          port_n "Negative complex magnetic port" annotation (Placement(
              transformation(extent={{90,-110},{110,-90}}, rotation=0)));
        Interfaces.PositiveMagneticPort
          port_p "Positive complex magnetic port" annotation (Placement(
              transformation(extent={{90,90},{110,110}}, rotation=0)));
        parameter Boolean useHeatPort=false
        "Enable / disable (=fixed temperatures) thermal port"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Resistance RRef
        "Winding resistance per phase at TRef";
        parameter Modelica.SIunits.Temperature TRef(start=293.15)
        "Reference temperature of winding";
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20(start=0) "Temperature coefficient of winding at 20 degC";
        final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=
            Modelica.Electrical.Machines.Thermal.convertAlpha(
                  alpha20,
                  TRef,
                  293.15);
        parameter Modelica.SIunits.Temperature TOperational(start=293.15)
        "Operational temperature of winding"
          annotation (Dialog(enable=not useHeatPort));
        parameter Real effectiveTurns=1 "Effective number of turns per phase";
        Modelica.Electrical.Analog.Basic.Resistor resistor(
          final useHeatPort=useHeatPort,
          final R=RRef,
          final T_ref=TRef,
          final alpha=alphaRef,
          final T=TOperational) annotation (Placement(transformation(
              origin={-10,70},
              extent={{10,10},{-10,-10}},
              rotation=90)));
        QuasiStationaryFundamentalWave.Components.QuasiStionaryAnalogElectroMagneticConverter
          electroMagneticConverter(final effectiveTurns=effectiveTurns)
                                     annotation (Placement(transformation(
                extent={{-10,-10},{10,10}}, rotation=0)));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortWinding if
          useHeatPort "Heat ports of winding resistor"
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      equation
        connect(pin_p, resistor.p) annotation (Line(points={{-100,100},{-10,100},
                {-10,80}}, color={0,0,255}));
        connect(electroMagneticConverter.pin_n, pin_n) annotation (Line(points=
                {{-10,-10},{-10,-100},{-100,-100}}, color={0,0,255}));
        connect(electroMagneticConverter.port_p, port_p) annotation (Line(
              points={{10,10},{10,100},{100,100}}, color={255,128,0}));
        connect(electroMagneticConverter.port_n, port_n) annotation (Line(
              points={{10,-10},{10,-100},{100,-100}}, color={255,128,0}));
        connect(heatPortWinding, resistor.heatPort) annotation (Line(
            points={{5.55112e-16,-100},{5.55112e-16,-60},{-40,-60},{-40,70},{-20,
                70}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(resistor.n, electroMagneticConverter.pin_p) annotation (Line(
            points={{-10,60},{-10,10}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                      extent={{-100,60},{100,-60}},
                      lineColor={0,0,255},
                      pattern=LinePattern.None,
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Line(points={{100,-100},{
                94,-100},{84,-98},{76,-94},{64,-86},{50,-72},{42,-58},{36,-40},
                {30,-18},{30,0},{30,18},{34,36},{46,66},{62,84},{78,96},{90,100},
                {100,100}}, color={255,128,0}),Line(points={{40,60},{-100,60},{
                -100,100}}, color={0,0,255}),Line(points={{40,-60},{-100,-60},{
                -100,-98}}, color={0,0,255}),Line(points={{40,60},{100,20},{40,
                -20},{0,-20},{-40,0},{0,20},{40,20},{100,-20},{40,-60}}, color=
                {0,0,255}),Text(
                      extent={{0,160},{0,120}},
                      lineColor={0,0,255},
                      fillColor={255,128,0},
                      fillPattern=FillPattern.Solid,
                      textString="%name")}), Documentation(info="<html>
<p>
The single phase winding consists of a
<a href=\"modelica://Modelica.Electrical.Analog.Basic.Resistor\">resistor</a>, a symmetrical
<a href=\"modelica://Modelica.Electrical.Analog.Basic.Inductor\">stray inductor</a> and a
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.SinglePhaseElectroMagneticConverter\">single phase electromagnetic coupling</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseCageWinding\">SymmetricMultiPhaseCageWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SaliencyCageWinding\">SaliencyCageWinding</a>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end QuasiStionaryAnalogWinding;

      model RotorSaliencyAirGap "Air gap model with rotor saliency"
        import Modelica.Constants.pi;
        Interfaces.PositiveMagneticPort port_sp
        "Positive complex magnetic stator port"   annotation (Placement(
              transformation(extent={{-110,-110},{-90,-90}}, rotation=0)));
        Interfaces.NegativeMagneticPort port_sn
        "Negative complex magnetic stator port"   annotation (Placement(
              transformation(extent={{-110,90},{-90,110}}, rotation=0)));
        Interfaces.PositiveMagneticPort port_rp
        "Positive complex magnetic rotor port"   annotation (Placement(
              transformation(extent={{90,90},{110,110}}, rotation=0)));
        Interfaces.NegativeMagneticPort port_rn
        "Negative complex magnetic rotor port"   annotation (Placement(
              transformation(extent={{90,-110},{110,-90}}, rotation=0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a
        "Flange of the rotor"   annotation (Placement(transformation(extent={{-10,
                  110},{10,90}}, rotation=0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a support
        "Support at which the reaction torque is acting"   annotation (
            Placement(transformation(extent={{-10,-110},{10,-90}}, rotation=0)));

        parameter Integer p "Number of pole pairs";
        parameter Modelica.Magnetic.FundamentalWave.Types.SalientInductance L0(
          d(start=1), q(start=1))
        "Salient inductance of a single unchorded coil w.r.t. the fundamental wave";
        final parameter
        Modelica.Magnetic.FundamentalWave.Types.SalientReluctance                 R_m(
          d=1/L0.d,q=1/L0.q) "Reluctance of the air gap model";
        // Complex phasors of magnetic potential differences
        Modelica.SIunits.ComplexMagneticPotentialDifference V_ms
        "Complex magnetic potential difference of stator w.r.t. stator reference frame";
        Modelica.SIunits.ComplexMagneticPotentialDifference V_msr =  V_ms * Modelica.ComplexMath.fromPolar(1,gammar)
        "Complex magnetic potential difference of stator w.r.t. rotor fixed reference frame";
        Modelica.SIunits.ComplexMagneticPotentialDifference V_mr
        "Complex magnetic potential difference of rotor w.r.t. rotor reference frame";
        Modelica.SIunits.ComplexMagneticPotentialDifference V_mrr = V_mr * Modelica.ComplexMath.fromPolar(1,gammar)
        "Complex magnetic potential difference of rotor w.r.t. rotor fixed reference frame";

        // Complex phasors of magnetic fluxes
        Modelica.SIunits.ComplexMagneticFlux Phi_s
        "Complex magnetic flux of stator w.r.t. stator reference frame";
        Modelica.SIunits.ComplexMagneticFlux Phi_sr = Phi_s * Modelica.ComplexMath.fromPolar(1,gammar)
        "Complex magnetic flux of stator w.r.t. rotor fixed reference frame";
        Modelica.SIunits.ComplexMagneticFlux Phi_r
        "Complex magnetic flux of rotor w.r.t. rotor refernce frame";
        Modelica.SIunits.ComplexMagneticFlux Phi_rr = Phi_r * Modelica.ComplexMath.fromPolar(1,gammar)
        "Complex magnetic flux of rotor w.r.t. rotor fixed reference frame";

        // Electrical torque and mechanical angle
        Modelica.SIunits.Torque tauElectrical "Electrical torque";
        // Modelica.SIunits.Torque tauTemp "Electrical torque";
        Modelica.SIunits.Angle gamma = p*(flange_a.phi - support.phi)
        "Electrical angle between rotor and stator";
        Modelica.SIunits.Angle gammas = port_sp.reference.gamma
        "Angle electrical qantities in stator reference frame";
        Modelica.SIunits.Angle gammar = port_rp.reference.gamma
        "Angle electrical qantities in rotor reference frame";
      equation
        // Stator flux into positive stator port
        port_sp.Phi = Phi_s;
        // Balance of stator flux
        port_sp.Phi + port_sn.Phi = Complex(0, 0);
        // Rotor flux into positive rotor port
        port_rp.Phi = Phi_r;
        // Balance of rotor flux
        port_rp.Phi + port_rn.Phi = Complex(0, 0);

        // Magneto motive force of stator
        port_sp.V_m - port_sn.V_m = V_ms;
        // Magneto motive force of stator
        port_rp.V_m - port_rn.V_m = V_mr;

        // Stator and rotor flux are equal with respect to different reference frames
        Phi_s = Phi_r;
        // Local balance of magneto motive force
        (pi/2.0)*(V_mrr.re + V_msr.re) = Phi_rr.re*R_m.d;
        (pi/2.0)*(V_mrr.im + V_msr.im) = Phi_rr.im*R_m.q;

        // Torque
        tauElectrical = -(pi*p/2.0)*(Phi_s.im*V_ms.re - Phi_s.re*V_ms.im);
        flange_a.tau = -tauElectrical;
        support.tau = tauElectrical;

        // Potential root of rotor has been removed. Only the stator positive
        //   plug is a potential root so that being a root determines that not
        //   electrical stator root is connected from outside; in this case the
        //   machine is operated as generator and the rotor angle is set to zero.
        // Magnetic stator and rotor port are (again) connected through
        //   Connections.branch, even though it is not clear yet whether this
        //   implementation is Modelica compliant
        Connections.potentialRoot(port_sp.reference);
        // Connections.potentialRoot(port_rp.reference);
        Connections.branch(port_sp.reference, port_sn.reference);
        port_sp.reference.gamma = port_sn.reference.gamma;
        Connections.branch(port_rp.reference, port_rn.reference);
        port_rp.reference.gamma = port_rn.reference.gamma;
        Connections.branch(port_sp.reference, port_rp.reference);
        gammas = gammar + gamma;
        if Connections.isRoot(port_sp.reference) then
          gammar=0;
        end if;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-100,90},{-100,60},{-80,60}}, color={255,128,0}),
              Line(points={{-100,-90},{-100,-60},{-80,-60}}, color={255,128,0}),
              Line(points={{40,60},{100,60},{100,90}}, color={255,128,0}),
              Line(points={{40,-60},{100,-60},{100,-90}}, color={255,128,0}),
              Ellipse(
                extent={{-60,80},{60,-80}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{0,80},{0,90}}, color={0,0,0})}), Documentation(info=
               "<html>
<p>
This salient air gap model can be used for machines with uniform airgaps and for machines with rotor saliencies. The air gap model is not symmetrical towards stator and rotor since it is assumed the saliency always refers to the rotor. The saliency of the air gap is represented by a main field inductance in the d- and q-axis.
</p>

<p>
For the mechanical interaction of the air gap model with the stator and the rotor it is equipped with to
<a href=\"modelica://Modelica.Mechanics.Rotational.Interfaces.Flange_a\">rotational connectors</a>. The torques acting on both connectors have the same absolute values but different signs. The difference between the stator and the rotor angle,
<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/gamma.png\">, is required for the transformation of the magnetic stator quantities to the rotor side.</p>

<p>
The air gap model has two magnetic stator and two magnetic rotor
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Interfaces.MagneticPort\">ports</a>. The magnetic potential difference and the magnetic flux of the stator (superscript s) are transformed to the rotor fixed reference frame (superscript r). The effective reluctances of the main field with respect to the d- and q-axis are considered then in the balance equations
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/Machines/Components/airgap.png\">
</p>

<p>
according to the following figure.
</p>
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig:</b> Magnetic equivalent circuit of the air gap model</caption>
  <tr>
    <td>
      <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/Machines/Components/airgap_phasors.png\">
    </td>
  </tr>
</table>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseCageWinding\">SymmetricMultiPhaseCageWinding</a>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SaliencyCageWinding\">SaliencyCageWinding</a>
</p>

</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end RotorSaliencyAirGap;

      model SymmetricMultiPhaseCageWinding "Symmetrical rotor cage"
        import Modelica.Constants.pi;
        import QuasiStationaryFundamentalWave;
        extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
        parameter Integer m=3 "Number of phases";
        parameter Boolean useHeatPort=false
        "Enable / disable (=fixed temperatures) thermal port"
          annotation (Evaluate=true);
        parameter Modelica.SIunits.Resistance RRef
        "Winding resistance per phase at TRef";
        parameter Modelica.SIunits.Temperature TRef(start=293.15)
        "Reference temperature of winding";
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20(start=0) "Temperature coefficient of winding at 20 degC";
        final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=
          Modelica.Electrical.Machines.Thermal.convertAlpha(
                  alpha20,
                  TRef,
                  293.15);
        parameter Modelica.SIunits.Temperature TOperational(start=293.15)
        "Operational temperature of winding"
          annotation (Dialog(enable=not useHeatPort));
        parameter Modelica.SIunits.Inductance Lsigma "Cage stray inductance";
        parameter Real effectiveTurns=1 "Effective number of turns";
        Modelica.SIunits.ComplexCurrent i[m] = strayInductor.i "Cage currents";
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          winding(
          final m=m, final effectiveTurns=effectiveTurns) "Symmetric winding"
                                      annotation (Placement(transformation(
              origin={0,-10},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor
                                                      strayInductor(final m=m,
            final L=fill(Lsigma, m)) annotation (Placement(transformation(
              origin={-20,-30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor
                                                      resistor(
          final useHeatPort=useHeatPort,
          final m=m,
          final T_ref=fill(TRef, m),
          final T=fill(TRef, m),
          R_ref=fill(RRef, m),
          alpha_ref=fill(alphaRef, m))
                                 annotation (Placement(transformation(
              origin={-20,-70},
              extent={{10,10},{-10,-10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
                                                  star(final m=m) annotation (
            Placement(transformation(extent={{30,-30},{50,-10}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
                                                ground annotation (Placement(
              transformation(
              origin={70,-20},
              extent={{-10,10},{10,-10}},
              rotation=270)));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortWinding if
          useHeatPort "Heat ports of winding resistor"
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
             useHeatPort "Connector of thermal rotor resistance heat ports"
          annotation (Placement(transformation(extent={{-50,-90},{-30,-70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
                                                  starAuxiliary(final m=m)
          annotation (Placement(transformation(extent={{30,-90},{50,-70}},
                rotation=0)));
      equation
        connect(thermalCollector.port_a, resistor.heatPort) annotation (Line(
            points={{-40,-70},{-30,-70}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(thermalCollector.port_b, heatPortWinding) annotation (Line(
            points={{-40,-90},{-40,-100},{5.55112e-16,-100}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(port_p, winding.port_p) annotation (Line(
            points={{-100,0},{-10,0}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(winding.port_n, port_n) annotation (Line(
            points={{10,0},{100,0}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(starAuxiliary.plug_p, resistor.plug_n) annotation (Line(
            points={{30,-80},{-20,-80}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor.plug_p, strayInductor.plug_n) annotation (Line(
            points={{-20,-60},{-20,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(strayInductor.plug_p, winding.plug_p) annotation (Line(
            points={{-20,-20},{-10,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(winding.plug_n, star.plug_p) annotation (Line(
            points={{10,-20},{30,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.pin) annotation (Line(
            points={{50,-20},{60,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starAuxiliary.pin_n, ground.pin) annotation (Line(
            points={{50,-80},{60,-80},{60,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={Ellipse(
                      extent={{-80,80},{80,-80}},
                      lineColor={0,0,0},
                      fillColor={175,175,175},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{-20,76},{20,36}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{28,46},{68,6}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{28,-8},{68,-48}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{-20,-36},{20,-76}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{-68,-6},{-28,-46}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Ellipse(
                      extent={{-66,50},{-26,10}},
                      lineColor={0,0,0},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid),Line(points={{-80,0},{-100,
                0}}, color={255,128,0}),Line(points={{100,0},{80,0}}, color={
                255,128,0}),Text(
                      extent={{0,100},{0,140}},
                      lineColor={0,0,255},
                      textString="%name")}), Documentation(info="<html>
<p>
<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/Machines/Components/rotorcage.png\">
</p>
<p>
The symmetric rotor cage model of this library does not consist of rotor bars and end rings. Instead the symmetric cage is modeled by an equivalent symmetrical winding. The rotor cage model consists of
<img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/m.png\"> phases. If the cage is modeled by equivalent stator winding parameters, the number of effective turns, <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/effectiveTurns.png\">, has to be chosen equivalent to the effective number of stator turns.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SaliencyCageWinding\">SaliencyCageWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics));
      end SymmetricMultiPhaseCageWinding;

      model SaliencyCageWinding "Rotor cage with saliency in d- and q-axis"
        import QuasiStationaryFundamentalWave;
        extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
        parameter Boolean useHeatPort=false
        "Enable / disable (=fixed temperatures) thermal port"
          annotation (Evaluate=true);
        parameter Modelica.Magnetic.FundamentalWave.Types.SalientResistance
          RRef(d(start=1), q(start=1)) "Salient cage resistance";
        parameter Modelica.SIunits.Temperature TRef(start=293.15)
        "Reference temperature of winding";
        parameter
        Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
          alpha20(start=0) "Temperature coefficient of winding at 20 degC";
        final parameter Modelica.SIunits.LinearTemperatureCoefficient alphaRef=
            Modelica.Electrical.Machines.Thermal.convertAlpha(
                  alpha20,
                  TRef,
                  293.15);
        parameter Modelica.SIunits.Temperature TOperational(start=293.15)
        "Operational temperature of winding"
          annotation (Dialog(enable=not useHeatPort));
        parameter Modelica.Magnetic.FundamentalWave.Types.SalientInductance
          Lsigma(d(start=1), q(start=1)) "Salient cage stray inductance";
        parameter Real effectiveTurns=1 "Effective number of turns";
        Modelica.SIunits.ComplexCurrent i[2] = strayInductor.i "Cage currents";
        Modelica.Blocks.Interfaces.RealOutput lossPower(
          final quantity="Power", final unit="W")=sum(resistor.resistor.LossPower)
        "Damper losses";
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
          winding(
          final m=2, final effectiveTurns=effectiveTurns) "Symmetric winding"
          annotation (Placement(transformation(
              origin={0,-10},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor
                                                      strayInductor(final m=2, L={
              Lsigma.d,Lsigma.q})        annotation (Placement(transformation(
              origin={-20,-30},
              extent={{10,-10},{-10,10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor
                                                      resistor(
          final useHeatPort=useHeatPort,
          final m=2,
          final T_ref=fill(TRef, 2),
          final T=fill(TOperational, 2),
          R_ref={RRef.d,RRef.q},
          alpha_ref=fill(alphaRef, 2))   annotation (Placement(transformation(
              origin={-20,-70},
              extent={{10,10},{-10,-10}},
              rotation=90)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
                                                  star(final m=2) annotation (
            Placement(transformation(extent={{30,-90},{50,-70}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
                                                ground annotation (Placement(
              transformation(
              origin={70,-80},
              extent={{-10,10},{10,-10}},
              rotation=270)));
        Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPortWinding if
          useHeatPort "Heat ports of winding resistor"
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
        Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=2) if
             useHeatPort "Connector of thermal rotor resistance heat ports"
          annotation (Placement(transformation(extent={{-50,-90},{-30,-70}})));
      equation
        connect(thermalCollector.port_b, heatPortWinding) annotation (Line(
            points={{-40,-90},{-40,-100},{0,-100}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(resistor.heatPort, thermalCollector.port_a) annotation (Line(
            points={{-30,-70},{-40,-70}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(port_p, winding.port_p) annotation (Line(
            points={{-100,4.44089e-16},{-55,4.44089e-16},{-55,0},{-10,0}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(winding.port_n, port_n) annotation (Line(
            points={{10,0},{100,0}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(winding.plug_n, resistor.plug_n) annotation (Line(
            points={{10,-20},{10,-80},{-20,-80}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor.plug_n, star.plug_p) annotation (Line(
            points={{-20,-80},{30,-80}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor.plug_p, strayInductor.plug_n) annotation (Line(
            points={{-20,-60},{-20,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(strayInductor.plug_p, winding.plug_p) annotation (Line(
            points={{-20,-20},{-10,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.pin) annotation (Line(
            points={{50,-80},{60,-80}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics={
              Ellipse(
                extent={{-80,80},{80,-80}},
                lineColor={0,0,0},
                fillColor={175,175,175},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-20,76},{20,36}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{28,46},{68,6}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{28,-8},{68,-48}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-20,-36},{20,-76}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-68,-6},{-28,-46}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-66,50},{-26,10}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-80,0},{-100,0}}, color={255,128,0}),
              Line(points={{100,0},{80,0}}, color={255,128,0}),
              Text(
                extent={{0,100},{0,140}},
                lineColor={0,0,255},
                textString="%name")}), Documentation(info="<html>

<p>
The salient cage model is a two axis model with two phases. The electromagnetic coupling therefore is also two phase coupling model. The angles of the two orientations are 0 and <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/pi_over_2.png\">. This way an asymmetrical rotor cage with different resistances and stray inductances in d- and q-axis can be modeled.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.SymmetricMultiPhaseCageWinding\">SymmetricMultiPhaseCageWinding</a>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end SaliencyCageWinding;

      model PermanentMagnet
      "Permanent magnet model without intrinsic reluctance, represeted by magnetic potential difference"
        extends QuasiStationaryFundamentalWave.Losses.PermanentMagnetLosses;
        extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
        parameter Modelica.SIunits.ComplexMagneticPotentialDifference V_m=
          Complex(re=1, im=0) "Complex magnetic potential difference";
        Modelica.SIunits.Angle gamma "Angle of V_m fixed reference frame";
        Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
        Modelica.SIunits.ComplexMagneticPotentialDifference V_mGamma=
          V_m * Modelica.ComplexMath.fromPolar(1,+gamma)
        "Magnetic potential difference transformed with reference angle";
      equation
        // Magneto motive force with respect to rotor fixed reference
        port_p.V_m - port_n.V_m = V_mGamma;
        // Flux into positive port with respect to rotor fixed reference
        port_p.Phi = Phi;
        // Local flux balance
        port_p.Phi + port_n.Phi = Complex(0,0);
        // Reference angular speed and angle
        gamma = port_p.reference.gamma;
        // Connections.root(port_p.reference);
        annotation (Documentation(info="<html>
</html>"),       Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                  {100,100}}), graphics));
      end PermanentMagnet;
    end Components;
  end BasicMachines;


  package Utilities "Utilities for quasi stationary fundamental wave machines"
    extends Modelica.Icons.Package;
    block VfController "Voltage-Frequency-Controller"
      constant Modelica.SIunits.Angle pi=Modelica.Constants.pi;
      parameter Integer m=3 "Number of phases";
      parameter Modelica.SIunits.Angle orientation[m]=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m)
      "Orientation of phases";
      parameter Modelica.SIunits.Voltage VNominal
      "Nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fNominal "Nominal frequency";
      parameter Modelica.SIunits.Angle BasePhase=0 "Common phase shift";
      output Modelica.SIunits.Voltage amplitude;
      Modelica.ComplexBlocks.Interfaces.ComplexOutput y[m] annotation (Placement(
            transformation(extent={{100,-10},{120,10}}),  iconTransformation(extent={{100,-10},
                {120,10}})));
      Modelica.Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    equation
    //amplitude = VNominal*min(abs(u)/fNominal, 1);
      amplitude = VNominal*(if abs(u)<fNominal then abs(u)/fNominal else 1);
      y = Modelica.ComplexMath.fromPolar(fill(amplitude,m),orientation + fill(BasePhase,m));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
                            graphics={
            Line(points={{-100,-100},{0,60},{80,60}}, color={0,0,255}),
            Line(points={{-70,0},{-60.2,29.9},{-53.8,46.5},{-48.2,58.1},{-43.3,
                  65.2},{-38.3,69.2},{-33.4,69.8},{-28.5,67},{-23.6,61},{-18.6,
                  52},{-13,38.6},{-5.98,18.6},{8.79,-26.9},{15.1,-44},{20.8,-56.2},
                  {25.7,-64},{30.6,-68.6},{35.5,-70},{40.5,-67.9},{45.4,-62.5},
                  {50.3,-54.1},{55.9,-41.3},{63,-21.7},{70,0}}, color={192,192,
                  192}, smooth=Smooth.Bezier),
            Line(points={{-40,0},{-30.2,29.9},{-23.8,46.5},{-18.2,58.1},{-13.3,
                  65.2},{-8.3,69.2},{-3.4,69.8},{1.5,67},{6.4,61},{11.4,52},{17,
                  38.6},{24.02,18.6},{38.79,-26.9},{45.1,-44},{50.8,-56.2},{
                  55.7,-64},{60.6,-68.6},{65.5,-70},{70.5,-67.9},{75.4,-62.5},{
                  80.3,-54.1},{85.9,-41.3},{93,-21.7},{100,0}}, color={192,192,
                  192}, smooth=Smooth.Bezier),
            Line(points={{-100,0},{-90.2,29.9},{-83.8,46.5},{-78.2,58.1},{-73.3,
                  65.2},{-68.3,69.2},{-63.4,69.8},{-58.5,67},{-53.6,61},{-48.6,
                  52},{-43,38.6},{-35.98,18.6},{-21.21,-26.9},{-14.9,-44},{-9.2,
                  -56.2},{-4.3,-64},{0.6,-68.6},{5.5,-70},{10.5,-67.9},{15.4,-62.5},
                  {20.3,-54.1},{25.9,-41.3},{33,-21.7},{40,0}}, color={192,192,
                  192}, smooth=Smooth.Bezier),
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0})}),
        Documentation(info="<HTML>
Simple Voltage-Frequency-Controller.<br>
Amplitude of voltage is linear dependent (VNominal/fNominal) on frequency (input signal \"u\"), but limited by VNominal (nominal RMS voltage per phase).<br>
m sine-waves with amplitudes as described above are provided as output signal \"y\".<br>
The sine-waves are intended to feed a m-phase SignalVoltage.<br>
Phase shifts between sine-waves may be chosen by the user; default values are <i>(k-1)/m*pi for k in 1:m</i>.
</HTML>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics));
    end VfController;

    model SwitchedRheostat "Rheostat which is shortened after a given time"
      parameter Integer m= 3 "Number of phases";
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
                                                             plug_p(final m=m)
      "To positive rotor plug"
        annotation (Placement(transformation(extent={{90,70},{110,50}}, rotation=0)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
                                                             plug_n(final m=m)
      "To negative rotor plug"
        annotation (Placement(transformation(extent={{90,-50},{110,-70}}, rotation=0)));
      parameter Modelica.SIunits.Resistance RStart "Starting resistance";
      parameter Modelica.SIunits.Time tStart
      "Duration of switching on the starting resistor";
      Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
                                                star(final m=m)
        annotation (Placement(transformation(extent={{-40,-70},{-60,-50}}, rotation=
               0)));
      Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
                                              ground
        annotation (Placement(transformation(
            origin={-80,-60},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Ideal.IdealCommutingSwitch
        idealCommutingSwitch(final m=m,
        Ron=fill(1e-5, m),
        Goff=fill(1e-5, m))
        annotation (Placement(transformation(
            origin={40,20},
            extent={{-10,10},{10,-10}},
            rotation=270)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor
                                                    rheostat(
        final m=m, R_ref=fill(RStart, m))
        annotation (Placement(transformation(extent={{0,-30},{-20,-10}}, rotation=0)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
                                                starRheostat(final m=m)
        annotation (Placement(transformation(extent={{-40,-30},{-60,-10}}, rotation=
               0)));
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](
        final startTime=fill(tStart, m),
        final startValue=fill(false, m))
        annotation (Placement(transformation(extent={{-60,10},{-40,30}}, rotation=0)));
    equation
      connect(booleanStep.y, idealCommutingSwitch.control) annotation (Line(
          points={{-39,20},{32,20}},
          color={255,0,255},
          smooth=Smooth.None));
    connect(idealCommutingSwitch.plug_p, plug_p) annotation (Line(
        points={{40,30},{40,60},{100,60}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(idealCommutingSwitch.plug_n1, rheostat.plug_p) annotation (Line(
        points={{35,10},{35,-20},{4.44089e-16,-20}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(rheostat.plug_n, starRheostat.plug_p) annotation (Line(
        points={{-20,-20},{-40,-20}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(starRheostat.pin_n, ground.pin) annotation (Line(
        points={{-60,-20},{-60,-60},{-70,-60}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(ground.pin, star.pin_n) annotation (Line(
        points={{-70,-60},{-60,-60}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(star.plug_p, idealCommutingSwitch.plug_n2) annotation (Line(
        points={{-40,-60},{40,-60},{40,10}},
        color={85,170,255},
        smooth=Smooth.None));
    connect(plug_n, idealCommutingSwitch.plug_n2) annotation (Line(
        points={{100,-60},{40,-60},{40,10}},
        color={85,170,255},
        smooth=Smooth.None));
      annotation (         Icon(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2}),          graphics={
            Rectangle(
              extent={{26,40},{54,-40}},
              lineColor={0,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{100,60},{-40,60},{-40,40}}, color={0,0,255}),
            Line(points={{100,-60},{-40,-60},{-40,-40}}, color={0,0,255}),
            Ellipse(extent={{-44,40},{-36,32}}, lineColor={0,0,255}),
            Ellipse(extent={{-44,-32},{-36,-40}}, lineColor={0,0,255}),
            Line(points={{-80,40},{-42,-34}}, color={0,0,255}),
            Line(points={{40,40},{40,42},{40,60}}, color={0,0,255}),
            Line(points={{40,-40},{40,-60}}, color={0,0,255}),
            Line(points={{10,-80},{70,-80}}, color={0,0,255}),
            Line(points={{40,-60},{40,-80}}, color={0,0,255}),
            Line(points={{20,-90},{60,-90}}, color={0,0,255}),
            Line(points={{30,-100},{50,-100}}, color={0,0,255})}), Documentation(info="<HTML>
<p>Switched rheostat, used for starting asynchronous induction motors with slipring rotor:</p>
<p>The external rotor resistance <code>RStart</code> is shortened at time <code>tStart</code>.</p>
</HTML>"),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics));
    end SwitchedRheostat;
  end Utilities;


  package Losses "Loss models"
  extends Modelica.Icons.Package;
    model StrayLoad "Model of stray load losses dependent on current and speed"
      extends Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.OnePort;
      extends Modelica.Electrical.Machines.Interfaces.FlangeSupport;
      import QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.quasiRMS;
      parameter Modelica.Electrical.Machines.Losses.StrayLoadParameters strayLoadParameters
      "Stray load loss parameters";
      extends
      Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT(
         useHeatPort=false);
      Modelica.SIunits.Current iRMS = quasiRMS(i);
    equation
      v = {Complex(0,0) for k in 1:m};
      if (strayLoadParameters.PRef<=0) then
        tau = 0;
      else
        tau = -strayLoadParameters.tauRef*(iRMS/strayLoadParameters.IRef)^2*
               smooth(1,if w >= 0 then +(+w/strayLoadParameters.wRef)^strayLoadParameters.power_w else
                                       -(-w/strayLoadParameters.wRef)^strayLoadParameters.power_w);
      end if;
      lossPower = -tau*w;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
              -100,-100},{100,100}}),
                       graphics={
              Rectangle(
            extent={{-70,30},{70,-30}},
            lineColor={85,170,255},
            pattern=LinePattern.Dot), Line(
            points={{-102,0},{100,0}},
            color={85,170,255},
            smooth=Smooth.None)}),
        Documentation(info="<html>
<p>
Stray load losses are modeled similar to standards EN 60034-2 and IEEE 512, i.e., they are dependent on square of current,
but without scaling them to zero at no-load current.
</p>
<p>
For an estimation of dependency on varying angular velocity see:<br>
W. Lang, &Uuml;ber die Bemessung verlustarmer Asynchronmotoren mit K&auml;figl&auml;ufer f&uuml;r Pulsumrichterspeisung,
Doctoral Thesis, Technical University of Vienna, 1984.
</p>
<p>
The stray load losses are modeled such way that they do not cause a voltage drop in the electric circuit.
Instead, the dissipated losses are considered through an equivalent braking torque at the shaft.
</p>
<p>
The stray load loss torque is
</p>
<pre>
  tau = PRef/wRef * (i/IRef)^2 * (w/wRef)^power_w
</pre>
<p>
where <code>i</code> is the current of the machine and <code>w</code> is the actual angular velocity.
The dependency of the stray load torque on the angular velocity is modeled by the exponent <code>power_w</code>.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Electrical.Machines.Losses.StrayLoadParameters\">StrayLoad parameters</a>
</p>
<p>
If it is desired to neglect stray load losses, set <code>strayLoadParameters.PRef = 0</code> (this is the default).
</p>
</html>"));
    end StrayLoad;

    model PermanentMagnetLosses
    "Model of permanent magnet losses dependent on current and speed"
      extends Modelica.Electrical.Machines.Interfaces.FlangeSupport;
      import QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.quasiRMS;
      parameter Integer m(min=1)=3 "Number of phases";
      parameter
      Modelica.Electrical.Machines.Losses.PermanentMagnetLossParameters           permanentMagnetLossParameters
      "Permanent magnet loss parameters";
      extends
      Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT(
         useHeatPort=false);
      input Modelica.SIunits.ComplexCurrent is[m]
      "Instantaneous stator currents";
      Modelica.SIunits.Current iRMS=quasiRMS(is);
    equation
      if (permanentMagnetLossParameters.PRef<=0) then
        tau = 0;
      else
        tau = -permanentMagnetLossParameters.tauRef*(permanentMagnetLossParameters.c + (1 - permanentMagnetLossParameters.c)*
                 (iRMS/permanentMagnetLossParameters.IRef)^permanentMagnetLossParameters.power_I)*
               smooth(1,if w >= 0 then +(+w/permanentMagnetLossParameters.wRef)^permanentMagnetLossParameters.power_w else
                                       -(-w/permanentMagnetLossParameters.wRef)^permanentMagnetLossParameters.power_w);
      end if;
      lossPower = -tau*w;
      annotation (Icon(graphics={            Ellipse(
                  extent={{-40,-40},{40,40}},
                  lineColor={200,0,0})}),
        Documentation(info="<html>
<p>
Permanent magnet losses are modeled dependent on current and speed.
</p>
<p>
The permanent magnet losses are modeled such way that they do not cause a voltage drop in the electric circuit.
Instead, the dissipated losses are considered through an equivalent braking torque at the shaft.
</p>
<p>
The permanent magnet loss torque is
</p>
<pre>
  tau = PRef/wRef * (c + (1 - c) * (i/IRef)^power_I) * (w/wRef)^power_w
</pre>
<p>
where <code>i</code> is the current of the machine and <code>w</code> is the actual angular velocity.
The parameter <code>c</code> designates the part of the permanent magnet losses that are present even at current = 0, i.e. independent of current.
The dependency of the permanent magnet loss torque on the stator current is modeled by the exponent <code>power_I</code>.
The dependency of the permanent magnet loss torque on the angular velocity is modeled by the exponent <code>power_w</code>.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Electrical.Machines.Losses.PermanentMagnetLossParameters\">Permanent magnet loss parameters</a>
</p>
<p>
If it is desired to neglect permanent magnet losses, set <code>strayLoadParameters.PRef = 0</code> (this is the default).
</p>
</html>"));
    end PermanentMagnetLosses;
  end Losses;


  package Sources "Sources to supply magnetic networks"
    extends Modelica.Icons.SourcesPackage;
    model ConstantMagneticPotentialDifference
    "Source with constant magnetic potential difference"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      parameter Modelica.SIunits.Frequency f(start=1) "frequency of the source";
      parameter Modelica.SIunits.ComplexMagneticPotentialDifference V_m=
        Complex(re=1, im=0) "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0,0);
      // Referenec angular speed and angle
      omega = 2*Modelica.Constants.pi*f;
      Connections.root(port_p.reference);
      annotation (         Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Text(
              extent={{-80,-20},{-80,-40}},
              lineColor={255,170,85},
              textString="+"),
            Text(
              extent={{80,-20},{80,-40}},
              lineColor={255,170,85},
              textString="-"),
            Ellipse(
              extent={{-50,-50},{50,50}},
              lineColor={255,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{100,0},{50,0}}, color={255,127,0}),
            Line(points={{-50,0},{-100,0}}, color={255,127,0}),
            Line(points={{-50,0},{50,0}}, color={255,127,0}),
            Text(
              extent={{0,-120},{0,-80}},
              textString="%name",
              lineColor={0,0,255})}),
        Documentation(info="<html>
<p>
Source of constant magneto motive force.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalMagneticPotentialDifference\">
   SignalMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantFlux\">ConstantFlux</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalFlux\">SignalFlux</a>
</p>
</html>"));
    end ConstantMagneticPotentialDifference;

    model SignalMagneticPotentialDifference
    "Source of magnetic potential difference with signal input"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      Modelica.ComplexBlocks.Interfaces.ComplexInput V_m
      "Complex signal input of magnetic potential difference"   annotation (
          Placement(transformation(
            origin={0,100},
            extent={{-20,-20},{20,20}},
            rotation=270)));
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0, 0);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Text(
                  extent={{80,-20},{80,-40}},
                  lineColor={255,170,85},
                  textString="-"),Ellipse(
                  extent={{-50,-50},{50,50}},
                  lineColor={255,170,85},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(points={{100,0},{50,0}},
              color={255,170,85}),
                                 Line(points={{-50,0},{-100,0}}, color={255,128,
              0}),Line(points={{-50,0},{50,0}}, color={255,170,85}),
                                                                   Line(points=
              {{0,100},{0,50}}, color={255,170,85}),
                                                   Text(
                  extent={{0,-120},{0,-80}},
                  textString="%name",
                  lineColor={0,0,255}),
            Text(
              extent={{-80,-20},{-80,-40}},
              lineColor={255,170,85},
              textString="+")}),          Documentation(info="<html>
<p>
Source of magneto motive force with complex signal input.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantMagneticPotentialDifference\">ConstantMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantFlux\">ConstantFlux</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalFlux\">SignalFlux</a>
</p>

</html>"));
    end SignalMagneticPotentialDifference;

    model ConstantFlux "Source of constant magnetic flux"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      parameter Modelica.SIunits.Frequency f(start=1) "frequency of the source";
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      parameter Modelica.SIunits.ComplexMagneticFlux Phi=
        Complex(re=1, im=0) "Complex magnetic flux";
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0,0);
      // Referenec angular speed and angle
      omega = 2*Modelica.Constants.pi*f;
      Connections.root(port_p.reference);
      annotation (         Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-50,-50},{50,50}},
              lineColor={255,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{100,0},{50,0}}, color={255,127,0}),
            Line(points={{-50,0},{-100,0}}, color={255,127,0}),
            Line(points={{0,50},{0,-50}}, color={255,127,0}),
            Polygon(
              points={{80,0},{60,6},{60,-6},{80,0}},
              lineColor={255,128,0},
              fillColor={255,170,85},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{0,-120},{0,-80}},
              textString="%name",
              lineColor={0,0,255})}),
        Documentation(info="<html>
<p>
Source of constant magnetic flux.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantMagneticPotentialDifference\">
   ConstantMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalMagneticPotentialDifference\">
   SignalMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalFlux\">SignalFlux</a>
</p>

</html>"));
    end ConstantFlux;

    model SignalFlux "Source of constant magnetic flux"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.ComplexBlocks.Interfaces.ComplexInput Phi
      "Complex signal input of magnetic flux"   annotation (Placement(
            transformation(
            origin={0,100},
            extent={{-20,-20},{20,20}},
            rotation=270)));
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0, 0);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={Ellipse(
                  extent={{-50,-50},{50,50}},
                  lineColor={255,170,85},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),Line(points={{100,0},{50,0}},
              color={255,170,85}),
                                 Line(points={{-50,0},{-100,0}}, color={255,170,
                  85}),
                  Line(points={{0,50},{0,-50}}, color={255,170,85}),
                                                                   Polygon(
                  points={{80,0},{60,6},{60,-6},{80,0}},
                  lineColor={255,170,85},
                  fillColor={255,170,85},
                  fillPattern=FillPattern.Solid),Line(points={{0,100},{0,50}},
              color={255,170,85}),
                                 Text(
                  extent={{0,-120},{0,-80}},
                  textString="%name",
                  lineColor={0,0,255})}), Documentation(info="<html>
<p>
Source of magnetic flux with complex signal input.
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantMagneticPotentialDifference\">
   ConstantMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.SignalMagneticPotentialDifference\">
   SignalMagneticPotentialDifference</a>,
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sources.ConstantFlux\">ConstantFlux</a>,
</p>

</html>"));
    end SignalFlux;
  end Sources;


  package Sensors "Sensors to measure variables in magnetic networks"
    extends Modelica.Icons.SensorsPackage;
    model MagneticFluxSensor "Sensor to measure magnetic flux"
      extends Modelica.Icons.RotationalSensor;
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.ComplexBlocks.Interfaces.ComplexOutput Phi
      "Complex magnetic flux from por_ p to port_n as output signal"
        annotation (Placement(transformation(
            origin={0,-100},
            extent={{10,-10},{-10,10}},
            rotation=90)));
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0, 0);
      // No magnetic potential difference at sensor
      V_m = Complex(0, 0);
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={Text(
                  extent={{-29,-11},{30,-70}},
                  lineColor={0,0,0},
                  textString="Phi"),Line(points={{-72,0},{-90,0}}, color={0,0,0}),
              Text(
                  extent={{-140,120},{140,80}},
                  textString="%name",
                  lineColor={0,0,255}),Line(points={{70,0},{90,0}}, color={0,0,
              0}),Line(points={{0,-90},{0,-70}})}), Documentation(info="<html>
<p>Sensor for magnetic flux.</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sensors.MagneticPotentialDifferenceSensor\">MagneticPotentialDifferenceSensor</a>
</p>
</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics));
    end MagneticFluxSensor;

    model MagneticPotentialDifferenceSensor
    "Sensor to measure magnetic potential difference"
      extends Modelica.Icons.RotationalSensor;
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      Modelica.ComplexBlocks.Interfaces.ComplexOutput V_m
      "Complex magnetic potential difference between port_p and port_n as output signal"
        annotation (Placement(transformation(
            origin={0,-100},
            extent={{10,-10},{-10,10}},
            rotation=90)));
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
    equation
      // Flux into positive port
      port_p.V_m - port_n.V_m = V_m;
      // Magneto motive force
      port_p.Phi = Phi;
      // Local flux balance
      port_p.Phi + port_n.Phi = Complex(0, 0);
      // No magnetic flux through sensor
      Phi = Complex(0, 0);
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={Text(
                  extent={{-52,1},{48,-57}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid,
                  textString="V_m"),Line(points={{-70,0},{-90,0}}, color={0,0,0}),
              Line(points={{70,0},{90,0}}, color={0,0,0}),Line(points={{0,-90},
              {0,-70}}),Text(
                  extent={{-140,120},{140,80}},
                  textString="%name",
                  lineColor={0,0,255})}), Documentation(info="<html>
<p>Sensor for magnetic potential difference.</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sensors.MagneticFluxSensor\">MagneticFluxSensor</a>
</p></html>"));
    end MagneticPotentialDifferenceSensor;

    model MagneticPotentialSensor "Sensor to measure magnetic potential"
      extends Modelica.Icons.RotationalSensor;
      Modelica.ComplexBlocks.Interfaces.ComplexOutput V_m
      "Complex magnetic potential as output signal"   annotation (Placement(
            transformation(
            origin={0,-100},
            extent={{10,-10},{-10,10}},
            rotation=90)));
      QuasiStationaryFundamentalWave.Interfaces.PositiveMagneticPort port_p
      "Magnetic connector of sensor"
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
    equation
      // No magnetic flux through sensor
      port_p.Phi = Complex(0, 0);
      // Magnetic potential
      V_m = port_p.V_m;
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={Text(
                  extent={{-52,1},{48,-57}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid,
                  textString="V_m"),Line(points={{-70,0},{-90,0}}, color={0,0,0}),
              Line(points={{0,-90},{0,-70}}),Text(
                  extent={{-140,120},{140,80}},
                  textString="%name",
                  lineColor={0,0,255})}), Documentation(info="<html>
<p>Sensor for magnetic potential difference.</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Sensors.MagneticFluxSensor\">MagneticFluxSensor</a>
</p></html>"));
    end MagneticPotentialSensor;
    annotation (Documentation(info="<html>
<p>
This package provides sensors for the magnetic potential difference and the magnetic flux in magnetic circuit.
</p>
</html>"));
  end Sensors;


  package Interfaces "Interfaces"
    extends Modelica.Icons.InterfacesPackage;
    connector Pin "Basic quasi stationary magnet connector"
      Modelica.SIunits.ComplexMagneticPotential V_m
      "Complex magnetic potential at the node";
      flow Modelica.SIunits.ComplexMagneticFlux Phi
      "Complex magnetic flux flowing into the pin";
      annotation (Documentation(info="<html></html>"));
    end Pin;

    connector PositiveMagneticPort "Positive magnetic port"
      extends QuasiStationaryFundamentalWave.Interfaces.Pin;
      Modelica.Electrical.QuasiStationary.Types.Reference reference "Reference";
      annotation (defaultComponentName="port_p",
        Diagram(graphics={Text(
              extent={{-100,100},{100,60}},
              lineColor={255,170,85},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="%name"),
                             Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={255,170,85},
              fillColor={255,170,85},
              fillPattern=FillPattern.Solid)}),
                                Icon(graphics={Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={255,170,85},
              fillColor={255,170,85},
              fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>

<p>
The positive pin is based on <a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>.
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of the quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.NegativePin\">negative pin</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
    end PositiveMagneticPort;

    connector NegativeMagneticPort "Negative magnetic port"
      extends QuasiStationaryFundamentalWave.Interfaces.Pin;
      Modelica.Electrical.QuasiStationary.Types.Reference reference "Reference";
      annotation (defaultComponentName="port_n",
        Diagram(graphics={Text(
              extent={{-100,100},{100,60}},
              lineColor={255,170,85},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="%name"),
                             Ellipse(
              extent={{-40,40},{40,-40}},
              lineColor={255,170,85},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
                                Icon(graphics={Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={255,170,85},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>

<p>
The negative pin is based on <a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>.
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of the quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive pin</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
    end NegativeMagneticPort;

    partial model PartialTwoPort "Partial two port for graphical programming"
      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma);
      QuasiStationaryFundamentalWave.Interfaces.PositiveMagneticPort port_p
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}},
          rotation=0)));
      QuasiStationaryFundamentalWave.Interfaces.NegativeMagneticPort port_n
        annotation (Placement(transformation(extent={{90,-10},{110,10}},
          rotation=0)));
    equation
      Connections.branch(port_p.reference, port_n.reference);
      port_p.reference.gamma = port_n.reference.gamma;
      annotation (Documentation(info="<html></html>"), Icon(graphics));
    end PartialTwoPort;

    partial model PartialTwoPortElementary
    "Elementary partial two port for textual programming"
      extends QuasiStationaryFundamentalWave.Interfaces.PartialTwoPort;
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";
    equation
      V_m = port_p.V_m - port_n.V_m;
      Phi = port_p.Phi;
      port_p.Phi + port_n.Phi = Complex(0,0);
      annotation (Documentation(info="<html></html>"), Icon(graphics));
    end PartialTwoPortElementary;

    partial model PartialBasicInductionMachine
    "Partial model for induction machine"
      extends
      QuasiStationaryFundamentalWave.Icons.QuasiStationaryFundamentalWaveMachine;
      constant Modelica.SIunits.Angle pi=Modelica.Constants.pi;
      parameter Integer m(min=3) = 3 "Number of stator phases";
      // Mechanical parameters
      parameter Modelica.SIunits.Inertia Jr(start=0.29) "Rotor inertia";
      parameter Boolean useSupport=false
      "Enable / disable (=fixed stator) support"   annotation (Evaluate=true);
      parameter Modelica.SIunits.Inertia Js(start=Jr) "Stator inertia"
        annotation (Dialog(enable=useSupport));
      parameter Boolean useThermalPort=false
      "Enable / disable (=fixed temperatures) thermal port"
        annotation (Evaluate=true);
      parameter Integer p(min=1, start=2) "Number of pole pairs (Integer)";
      parameter Modelica.SIunits.Frequency fsNominal(start=50)
      "Nominal frequency";
      parameter Modelica.SIunits.Temperature TsOperational(start=293.15)
      "Operational temperature of stator resistance"   annotation (Dialog(group=
             "Operational temperatures", enable=not useThermalPort));
      parameter Modelica.SIunits.Resistance Rs(start=0.03)
      "Stator resistance per phase at TRef"
        annotation (Dialog(tab="Nominal resistances and inductances"));
      parameter Modelica.SIunits.Temperature TsRef(start=293.15)
      "Reference temperature of stator resistance"
        annotation (Dialog(tab="Nominal resistances and inductances"));
      parameter
      Modelica.Electrical.Machines.Thermal.LinearTemperatureCoefficient20
        alpha20s(start=0)
      "Temperature coefficient of stator resistance at 20 degC"
        annotation (Dialog(tab="Nominal resistances and inductances"));
      parameter Real effectiveStatorTurns=1 "Effective number of stator turns";
      parameter Modelica.SIunits.Inductance Lssigma(start=3*(1 - sqrt(1 -
            0.0667))/(2*pi*fsNominal)) "Stator stray inductance"
        annotation (Dialog(tab="Nominal resistances and inductances"));
      parameter Modelica.Magnetic.FundamentalWave.Types.SalientInductance L0(d(
            start=1), q(start=1)) "Salient inductance of an unchorded coil"
        annotation (Dialog(tab="Nominal resistances and inductances"));
      parameter Modelica.Electrical.Machines.Losses.FrictionParameters
        frictionParameters(wRef=2*pi*fsNominal/p) "Friction losses"
        annotation (Dialog(tab="Losses"));
      parameter Modelica.Electrical.Machines.Losses.CoreParameters
        statorCoreParameters(
        final m=3,
        wRef=2*pi*fsNominal/p,
        VRef(start=100))
      "Stator core losses; all parameters refer to stator side"
        annotation (Dialog(tab="Losses"));
      parameter Modelica.Electrical.Machines.Losses.StrayLoadParameters
        strayLoadParameters(IRef(start=100), wRef=2*pi*fsNominal/p)
      "Stray load losses"   annotation (Dialog(tab="Losses"));

      output Modelica.SIunits.Angle gammas(start=0) = airGap.gammas
      "Angle of stator reference frame";
      output Modelica.SIunits.Angle gammar(start=0) = airGap.gammar
      "Angle of stator reference frame";
      output Modelica.SIunits.Angle gamma(start=0) = airGap.gamma
      "Electrical angle between stator and rotor";

      // Mechanical quantities
      output Modelica.SIunits.Angle phiMechanical = flange.phi -
        internalSupport.phi "Mechanical angle of rotor against stator";
      output Modelica.SIunits.AngularVelocity wMechanical(start=0,
        displayUnit="1/min") = der(phiMechanical)
      "Mechanical angular velocity of rotor against stator";
      output Modelica.SIunits.Torque tauElectrical=inertiaRotor.flange_a.tau
      "Electromagnetic torque";
      output Modelica.SIunits.Torque tauShaft=-flange.tau "Shaft torque";
      replaceable output
      Modelica.Electrical.Machines.Interfaces.InductionMachines.PartialPowerBalanceInductionMachines
        powerBalance(
        final powerStator=
            QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.activePower(
            vs, is),
        final powerMechanical=wMechanical*tauShaft,
        final powerInertiaStator=inertiaStator.J*inertiaStator.a*inertiaStator.w,
        final powerInertiaRotor=inertiaRotor.J*inertiaRotor.a*inertiaRotor.w,
        final lossPowerStatorWinding=sum(stator.resistor.resistor.LossPower),
        final lossPowerStatorCore=stator.core.lossPower,
        final lossPowerStrayLoad=strayLoad.lossPower,
        final lossPowerFriction=friction.lossPower) "Power balance";
      // Stator voltages and currents
      output Modelica.SIunits.ComplexVoltage vs[m]=plug_sp.pin.v - plug_sn.pin.v
      "Stator instantaneous voltages";
      output Modelica.SIunits.ComplexCurrent is[m]=plug_sp.pin.i
      "Stator instantaneous currents";
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange "Shaft"
        annotation (Placement(transformation(extent={{90,-10},{110,10}},
              rotation=0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertiaRotor(final J=Jr)
        annotation (Placement(transformation(
            origin={80,0},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a support if useSupport
      "Support at which the reaction torque is acting"   annotation (Placement(
            transformation(extent={{90,-110},{110,-90}}, rotation=0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertiaStator(final J=Js)
        annotation (Placement(transformation(
            origin={80,-100},
            extent={{10,10},{-10,-10}},
            rotation=180)));
      Modelica.Mechanics.Rotational.Components.Fixed fixed if (not useSupport)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={70,-90})));
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
                                                             plug_sp(final m=m)
      "Positive plug of stator"   annotation (Placement(transformation(extent={
                {50,90},{70,110}}, rotation=0)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
                                                             plug_sn(final m=m)
      "Negative plug of stator"   annotation (Placement(transformation(extent={
                {-70,90},{-50,110}}, rotation=0)));
      BasicMachines.Components.SymmetricMultiPhaseWinding stator(
        final useHeatPort=true,
        final m=m,
        final RRef=Rs,
        final TRef=TsRef,
        final Lsigma=Lssigma,
        final effectiveTurns=effectiveStatorTurns,
        final TOperational=TsOperational,
        final GcRef=statorCoreParameters.GcRef,
      final alpha20=alpha20s)
      "Symmetric stator winding including resistances, zero and stray inductances and core losses"
        annotation (Placement(transformation(
            origin={0,40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      replaceable
      Modelica.Electrical.Machines.Interfaces.InductionMachines.PartialThermalAmbientInductionMachines
        thermalAmbient(
        final useTemperatureInputs=false,
        final Ts=TsOperational,
        final m=m) if not useThermalPort annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-70,-90})));
      replaceable
      Modelica.Electrical.Machines.Interfaces.InductionMachines.PartialThermalPortInductionMachines
        thermalPort(final m=m) if useThermalPort
      "Thermal port of induction machines"
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      Components.Ground                                   groundS
      "Ground of stator magnetic circuit"   annotation (Placement(
            transformation(extent={{-40,30},{-20,10}}, rotation=0)));
      BasicMachines.Components.RotorSaliencyAirGap
        airGap(final p=p, final L0=L0) annotation (Placement(transformation(
            origin={0,0},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Components.Ground                                   groundR
      "Ground of rotor magnetic circuit"   annotation (Placement(transformation(
              extent={{-40,-30},{-20,-10}}, rotation=0)));
      Losses.StrayLoad strayLoad(
        final strayLoadParameters=strayLoadParameters,
        final useHeatPort=true,
        final m=m)
        annotation (Placement(transformation(extent={{60,60},{40,80}})));
      Modelica.Electrical.Machines.Losses.Friction friction(final
          frictionParameters=frictionParameters, final useHeatPort=true)
                                                        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-30})));
  protected
      replaceable
      Modelica.Electrical.Machines.Interfaces.InductionMachines.PartialThermalPortInductionMachines
        internalThermalPort(final m=m)
        annotation (Placement(transformation(extent={{-44,-94},{-36,-86}})));
      Modelica.Mechanics.Rotational.Interfaces.Support internalSupport
        annotation (Placement(transformation(extent={{56,-104},{64,-96}},
              rotation=0)));
    initial algorithm
      assert(not Modelica.Math.isPowerOf2(m), String(m) +
        " phases are currently not supported in this version of FundametalWave");

    equation
      connect(stator.plug_n, plug_sn) annotation (Line(
        points={{-10,50},{-10,70},{-60,70},{-60,100}},
        color={85,170,255},
        smooth=Smooth.None));
      connect(thermalPort, internalThermalPort) annotation (Line(
          points={{5.55112e-16,-100},{5.55112e-16,-90},{-40,-90}},
          color={199,0,0},
          smooth=Smooth.None));
      connect(thermalAmbient.thermalPort, internalThermalPort) annotation (Line(
          points={{-60,-90},{-40,-90}},
          color={199,0,0},
          smooth=Smooth.None));
      connect(inertiaRotor.flange_b, flange) annotation (Line(points={{90,-1.72421e-15},
              {100,-1.72421e-15},{100,5.55112e-16}}, color={0,0,0}));
      connect(internalSupport, inertiaStator.flange_a) annotation (Line(
          points={{60,-100},{70,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(internalSupport, fixed.flange) annotation (Line(
          points={{60,-100},{60,-90},{70,-90}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(inertiaStator.flange_b, support) annotation (Line(points={{90,-100},
              {90,-100},{100,-100}}, color={0,0,0}));
      connect(airGap.flange_a, inertiaRotor.flange_a) annotation (Line(
          points={{10,-1.33731e-15},{25,-1.33731e-15},{25,-6.12304e-16},{40,-6.12304e-16},
              {40,7.25006e-16},{70,7.25006e-16}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(airGap.support, internalSupport) annotation (Line(
          points={{-10,2.33651e-15},{-50,2.33651e-15},{-50,-70},{60,-70},{60,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(groundR.port_p, airGap.port_rn) annotation (Line(points={{-30,-10},
            {-20,-10},{-10,-10}},   color={255,128,0}));
      connect(stator.plug_p, strayLoad.plug_n) annotation (Line(
        points={{10,50},{10,70},{40,70}},
        color={85,170,255},
        smooth=Smooth.None));
      connect(plug_sp, strayLoad.plug_p) annotation (Line(
        points={{60,100},{60,94},{60,94},{60,86},{60,86},{60,70}},
        color={85,170,255},
        smooth=Smooth.None));
      connect(strayLoad.support, internalSupport) annotation (Line(
          points={{50,60},{50,50},{60,50},{60,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(strayLoad.heatPort, internalThermalPort.heatPortStrayLoad)
        annotation (Line(
          points={{60,60},{60,50},{50,50},{50,-80},{-40,-80},{-40,-90}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(friction.support, internalSupport) annotation (Line(
          points={{90,-40},{90,-70},{60,-70},{60,-100}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(strayLoad.flange, inertiaRotor.flange_b) annotation (Line(
          points={{50,80},{90,80},{90,-1.72421e-15}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(friction.flange, inertiaRotor.flange_b) annotation (Line(
          points={{90,-20},{90,-1.72421e-15}},
          color={0,0,0},
          smooth=Smooth.None));
      connect(friction.heatPort, internalThermalPort.heatPortFriction)
        annotation (Line(
          points={{80,-40},{50,-40},{50,-80},{-40,-80},{-40,-90}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(groundS.port_p, airGap.port_sp) annotation (Line(
          points={{-30,10},{-10,10}},
          color={255,128,0},
          smooth=Smooth.None));
      connect(stator.port_n, airGap.port_sp) annotation (Line(
          points={{-10,30},{-10,10}},
          color={255,128,0},
          smooth=Smooth.None));
      connect(stator.port_p, airGap.port_sn) annotation (Line(
          points={{10,30},{10,10}},
          color={255,128,0},
          smooth=Smooth.None));
      connect(stator.heatPortWinding, internalThermalPort.heatPortStatorWinding)
        annotation (Line(
          points={{-10,44},{-40,44},{-40,-90}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(stator.heatPortCore, internalThermalPort.heatPortStatorCore)
        annotation (Line(
          points={{-10,36},{-40,36},{-40,-90}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (
        Documentation(info="<HTML>
<p>This partial model for induction machines contains elements common in all machine models.</p>
</HTML>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={
            Rectangle(
              extent={{80,-80},{120,-120}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-50,100},{-20,100},{-20,70}}, color={85,170,255}),
            Line(points={{50,100},{20,100},{20,70}}, color={85,170,255}),
            Text(
              extent={{-150,-120},{150,-180}},
              lineColor={0,0,255},
              textString="%name"),
            Line(
              visible=not useSupport,
              points={{80,-100},{120,-100}},
              color={0,0,0},
              smooth=Smooth.None),
            Line(
              visible=not useSupport,
              points={{90,-100},{80,-120}},
              color={0,0,0},
              smooth=Smooth.None),
            Line(
              visible=not useSupport,
              points={{100,-100},{90,-120}},
              color={0,0,0},
              smooth=Smooth.None),
            Line(
              visible=not useSupport,
              points={{110,-100},{100,-120}},
              color={0,0,0},
              smooth=Smooth.None),
            Line(
              visible=not useSupport,
              points={{120,-100},{110,-120}},
              color={0,0,0},
              smooth=Smooth.None)}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics));
    end PartialBasicInductionMachine;
  end Interfaces;


  package MoveTo_Modelica
    extends Modelica.Icons.Package;
    package Functions "Functions"
      extends Modelica.Icons.Package;
      function symmetricOrientationMatrix
      "Matrix symmetric orientation angles for creating the symmetric transformation matrix"
        extends Modelica.Icons.Function;
        import Modelica.Constants.pi;
        input Integer m "Number of phases";
        output Modelica.SIunits.Angle orientation[m,m]
        "Angles of symmetric transformation matrix";
      algorithm
        // Init transformation matrix with zeros
        orientation :=zeros(m, m);
        // Insert non zero coefficients
        if mod(m, 2) == 0 then
          // Even number of phases
          if m == 2 then
            // Special case two phase machine
            orientation := {{0,pi/2},{0,0}};
          else
            orientation[1:integer(m/2),1:integer(m/2)] :=
                symmetricOrientationMatrix(integer(m/2));
            orientation[1+integer(m/2):m,1+integer(m/2):m] :=
                symmetricOrientationMatrix(integer(m/2))
              - fill(pi/m,integer(m/2),integer(m/2));
          end if;
        else
          // Odd number of phases
          for k in 1:m loop
            orientation[k,:]:=
              Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m)*k;
          end for;
        end if;
        annotation (Documentation(info="<html>
<p>
This function determines the orientation of the symmetrical winding with <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/m.png\"> phases. For an odd number of phases the difference of the windings angles of two adjacent phases is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/2pi_over_m.png\">. In case of an even number of phases the aligned orientation of winding is not modeled since they do not add any information. Instead the <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/m.png\"> windings are divided into two different groups. The first group refers to the indices <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/k_le_m_over_2.png\">. The second group covers the indices <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/k_gt_m_over_2.png\">. The difference of the windings angles of two adjacent phases - of both the first and the second group, respectively - is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/4pi_over_m.png\">. The phase shift of the two groups is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/pi_over_m.png\">.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.UsersGuide.MultiPhase\">User's guide on multi phase winding</a>,
</p>
</html>"));
      end symmetricOrientationMatrix;

      function symmetricTransformationMatrix
      "Transformation matrix for symmetrical components"
        extends Modelica.Icons.Function;
        import Modelica.Constants.pi;
        input Integer m "Number of phases";
        output Complex transformation[m,m]
        "Transformation matrix for m phase symmetrical components";
      algorithm
        // Init transformation matrix with zeros
        transformation := Modelica.ComplexMath.fromPolar(fill(
              1,
              m,
              m),
          QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.symmetricOrientationMatrix(
          m))/m;
        annotation (Documentation(info="<html>
<p>
This function determines the orientation of the symmetrical winding with <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/m.png\"> phases. For an odd number of phases the difference of the windings angles of two adjacent phases is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/2pi_over_m.png\">. In case of an even number of phases the aligned orientation of winding is not modeled since they do not add any information. Instead the <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/m.png\"> windings are divided into two different groups. The first group refers to the indices <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/k_le_m_over_2.png\">. The second group covers the indices <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/k_gt_m_over_2.png\">. The difference of the windings angles of two adjacent phases - of both the first and the second group, respectively - is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/4pi_over_m.png\">. The phase shift of the two groups is <img src=\"modelica://Modelica/Resources/Images/Magnetic/FundamentalWave/pi_over_m.png\">.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.UsersGuide.MultiPhase\">User's guide on multi phase winding</a>,
</p>
</html>"));
      end symmetricTransformationMatrix;

      function numberOfSymmetricBaseSystems
      "Determines the number of symmeric base systems of m phase symmetric system"
        extends Modelica.Icons.Function;
        input Integer m = 3 "Number of phases";
        output Integer n "Number of symmetric base systems";
      algorithm
      // Init number of base systmes
      n := 1;
      if mod(m, 2) == 0 then
        // Even number of phases
        if m == 2 then
          // Special case two phase machine
          n :=1;
        else
          n :=n*2*numberOfSymmetricBaseSystems(integer(m/2));
        end if;
      else
        // Odd number of phases
        n :=1;
      end if;
      end numberOfSymmetricBaseSystems;

      function indexPositiveSequence
      "Determines the indices of the all positive sequences"
        extends Modelica.Icons.Function;
        input Integer m = 3 "Number of phases";
        output Integer ind[numberOfSymmetricBaseSystems(m)]
        "Number of symmetric base systems";
    protected
        Integer n = numberOfSymmetricBaseSystems(m);
      algorithm
      if n==1 then
        ind[1] := 1;
      else
        ind :=integer(linspace(0,n - 1,n))*integer(m/n) + ones(n);
      end if;
      end indexPositiveSequence;

      function indexNonPositiveSequence
      "Determines the indices of all non positive sequences"
        extends Modelica.Icons.Function;
        input Integer m = 3 "Number of phases";
        output Integer ind[numberOfSymmetricBaseSystems(m)
                         *(integer(m/numberOfSymmetricBaseSystems(m))-1)]
        "Indices of non positive sequences";
    protected
        Integer n = numberOfSymmetricBaseSystems(m) "Number of base systems";
        Integer mbas = integer(m/n) "Number of phases of base system";
      algorithm
      if mbas==1 then
        ind:=fill(0, 0);
      elseif mbas==2 then
        for k in 1:n loop
          ind[k] := 2+2*(k-1);
        end for;
      else
        for k in 1:n loop
          ind[(mbas-1)*(k-1)+1:(mbas-1)*k] :=
            integer(linspace(2,mbas,mbas - 1))
          + mbas*(k - 1)*ones(mbas - 1);
        end for;
      end if;
      end indexNonPositiveSequence;

      function quasiRMS "Calculate quasi-RMS value of input"
        extends Modelica.Icons.Function;
        import Modelica.ComplexMath.'abs';
        input Complex u[:];
        output Real y;
        import Modelica.Constants.pi;
    protected
        Integer m=size(u,1) "Number of phases";
      algorithm
        y:= sum({'abs'(u[k]) for k in 1:m})/m;
      end quasiRMS;

      function activePower
      "Calculate active power of voltage and current input"
        extends Modelica.Icons.Function;
        input Modelica.SIunits.ComplexVoltage v[:]
        "QuasiStationary voltage phasors";
        input Modelica.SIunits.ComplexCurrent i[size(v, 1)]
        "QuasiStationary current phasors";
        output Modelica.SIunits.Power p "Active power";
      algorithm
        p := sum(Modelica.ComplexMath.real({v[k]* Modelica.ComplexMath.conj(i[k]) for k in 1:size(v, 1)}));
        annotation (Inline=true, Documentation(info="<HTML>
<p>
Calculates instantaneous power from multiphase voltages and currents.
In quasistaionary operation, instantaneous power equals active power;
</p>
</HTML>"));
      end activePower;
    end Functions;

    package QuasiStationary_MultiPhase
      extends Modelica.Icons.VariantsPackage;
      model MStar "Star connection"
        parameter Integer m(final min=1) = 3 "Number of phases";
        final parameter Integer mBasic=integer(m/QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(m));
        final parameter Integer mSystems=integer(m/mBasic);
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug plug_p(final m=m)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
                 0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug starpoints(final m=
              mSystems)
          annotation (Placement(transformation(extent={{90,-10},{110,10}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_p plugToPins_p(final m=m)
                                  annotation (Placement(transformation(extent={{-80,-10},
                  {-60,10}},      rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_n
          plugToPins_n(final m=mSystems)
          annotation (Placement(transformation(extent={{80,-10},{60,10}})));
      equation
        for k in 1:mSystems loop
          for j in 1:mBasic loop
            connect(plugToPins_p.pin_p[(k - 1)*mBasic + j], plugToPins_n.pin_n[k]);
          end for;
        end for;
        connect(plug_p, plugToPins_p.plug_p)
          annotation (Line(points={{-100,0},{-72,0}},              color={85,170,255}));
        connect(plugToPins_n.plug_n, starpoints) annotation (Line(
            points={{72,0},{100,0}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Icon(graphics={
              Text(extent={{-150,60},{150,120}}, textString=
                                                    "%name",
                lineColor={0,0,255}),
              Line(
                points={{86,4},{6,4}},
                color={0,0,255},
                thickness=0.5),
              Line(
                points={{6,4},{-33,72}},
                color={0,0,255},
                thickness=0.5),
              Line(
                points={{6,4},{-32,-65}},
                color={0,0,255},
                thickness=0.5),
              Text(
                extent={{-100,-110},{100,-70}},
                lineColor={0,0,0},
                textString =                           "m=%m"),
              Line(points={{-90,0},{-40,0}}, color={0,0,255}),
              Line(points={{80,0},{90,0}}, color={0,0,255}),
              Line(
                points={{-6,-4},{-45,64}},
                color={0,0,255},
                thickness=0.5),
              Line(
                points={{74,-4},{-6,-4}},
                color={0,0,255},
                thickness=0.5),
              Line(
                points={{-6,-4},{-44,-73}},
                color={0,0,255},
                thickness=0.5)}),
        Documentation(info="<html>
<p>
Star (wye) connection of a multi phase circuit. The potentials at the star points are the same.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Delta\">Delta</a>
</p>
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                  100}}), graphics));
      end MStar;

      model MDelta "Delta (polygon) connection"
        parameter Integer m(final min=2) = 3 "Number of phases";
        final parameter Integer mBasic=integer(m/QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(m));
        final parameter Integer mSystems=integer(m/mBasic);
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug plug_p(final m=m)
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}}, rotation=
                 0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug plug_n(final m=m)
          annotation (Placement(transformation(extent={{90,-10},{110,10}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_p plugToPins_p(final m=m)
                                  annotation (Placement(transformation(extent={{-80,
                  -10},{-60,10}}, rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_n plugToPins_n(final m=m)
                                  annotation (Placement(transformation(extent={{80,
                  -10},{60,10}}, rotation=0)));
      equation
        for k in 1:mSystems loop
          for j in 1:mBasic-1 loop
            connect(plugToPins_n.pin_n[(k - 1)*mBasic + j], plugToPins_p.pin_p[(k - 1)*mBasic + j + 1]);
          end for;
          connect(plugToPins_n.pin_n[k*mBasic], plugToPins_p.pin_p[(k - 1)*mBasic + 1]);
        end for;
        connect(plug_p, plugToPins_p.plug_p)
          annotation (Line(points={{-100,0},{-93,0},{-86,0},{-72,0}},
              color={85,170,255}));
        connect(plugToPins_n.plug_n, plug_n)
          annotation (Line(points={{72,0},{79,0},{79,0},{86,0},
                {86,0},{100,0}},
              color={85,170,255}));
        annotation (Icon(graphics={
              Text(
                extent={{-150,60},{150,120}},
                lineColor={0,0,255},
                textString =                        "%name"),
              Line(
                points={{-44,62},{-44,-76},{75,-6},{-44,62},{-44,61}},
                color={0,0,255},
                thickness=0.5),
              Text(
                extent={{-100,-110},{100,-70}},
                lineColor={0,0,0},
                textString =                           "m=%m"),
              Line(points={{-90,0},{-44,0}}, color={0,0,255}),
              Line(points={{80,0},{90,0}}, color={0,0,255}),
              Line(
                points={{-36,74},{-36,-64},{83,6},{-36,74},{-36,73}},
                color={0,0,255},
                thickness=0.5)}),
        Documentation(info="<html>
<p>
Delta (polygon) connection of a multi phase circuit.
</p>
<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star\">Star</a>
</p>
</html>"));
      end MDelta;

      model TerminalBox "Terminal box Y/D-connection"
        extends BaseTerminalBox;
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
          plug_sp(final m=m) "To positive stator plug"
          annotation (Placement(transformation(extent={{50,-90},{70,-110}},
                rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
          plug_sn(final m=m) "To negative stator plug"
          annotation (Placement(transformation(extent={{-70,-90},{-50,-110}},
                rotation=0)));
        MStar mStar(final m=m) if (terminalConnection <> "D")
          annotation (Placement(transformation(
              origin={-70,-80},
              extent={{-10,10},{10,-10}},
              rotation=180)));
        MDelta mDelta(final m=m) if (terminalConnection == "D")
          annotation (Placement(transformation(extent={{-20,-70},{-40,-50}},
                rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.PositivePlug
          plugSupply(final m=m) "To grid"
          annotation (Placement(transformation(extent={{-10,-70},{10,-90}},
                rotation=0)));
        Modelica.Electrical.QuasiStationary.MultiPhase.Interfaces.NegativePlug
          starpoint(final m=mSystems) if (terminalConnection<>"D")
          annotation (Placement(transformation(extent={{-100,-90},{-80,-70}},
                rotation=0)));
      equation
        connect(mStar.plug_p, plug_sn) annotation (Line(
            points={{-60,-80},{-60,-100}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(starpoint, mStar.starpoints) annotation (Line(
            points={{-90,-80},{-80,-80}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(mDelta.plug_n, plug_sn) annotation (Line(
            points={{-40,-60},{-40,-100},{-60,-100}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(mDelta.plug_p, plug_sp) annotation (Line(
            points={{-20,-60},{60,-60},{60,-100}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(plugSupply, plug_sp) annotation (Line(
            points={{0,-80},{0,-100},{60,-100}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}}),
                graphics={
                Text(extent={{-40,-90},{40,-130}},
                lineColor={0,0,0},
                textString="%terminalConnection")}),
          Documentation(info="<html>
TerminalBox: at the bottom connected to both machine plugs, connect at the top to the grid as usual,<br>
choosing Y-connection (StarDelta=Y) or D-connection (StarDelta=D).
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                               graphics));
      end TerminalBox;

      model BaseTerminalBox "Terminal box Y/D-connection"
        parameter Integer m(min=1) = 3 "Number of phases";
        final parameter Integer mBasic=integer(m/QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.numberOfSymmetricBaseSystems(m));
        final parameter Integer mSystems=integer(m/mBasic);
        parameter String terminalConnection(start="Y") "Choose Y=star/D=delta"
          annotation(choices(choice="Y" "Star connection",choice="D"
            "Delta connection"));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}}),
                graphics={Polygon(
                points={{-74,-80},{-80,-86},{-80,-120},{-40,-140},{40,-140},{80,
                    -110},{80,-84},{76,-80},{-74,-80}},
                lineColor={95,95,95},
                fillColor={135,135,135},
                fillPattern=FillPattern.CrossDiag),
                Text(extent={{-40,-90},{40,-130}},
                lineColor={0,0,0},
                textString="%terminalConnection")}),
          Documentation(info="<html>
TerminalBox: at the bottom connected to both machine plugs, connect at the top to the grid as usual,<br>
choosing Y-connection (StarDelta=Y) or D-connection (StarDelta=D).
</html>"),Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                               graphics));
      end BaseTerminalBox;

      block SymmetricalComponents
      "Creates symmetrical components from signals representing quasi stationary phasors"
        extends Modelica.ComplexBlocks.Interfaces.ComplexMIMO(final nin=m,final nout=m);
        parameter Integer m = 3 "Number of phases";
      equation
        y = QuasiStationaryFundamentalWave.MoveTo_Modelica.Functions.symmetricTransformationMatrix(m)*u;
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics), Icon(coordinateSystem(
                preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
      end SymmetricalComponents;
    end QuasiStationary_MultiPhase;
  end MoveTo_Modelica;


  package Icons "Icons"
    extends Modelica.Icons.Package;
    partial model QuasiStationaryFundamentalWaveMachine
      annotation (Icon(graphics={
            Rectangle(
            extent={{-40,60},{80,-60}},
            lineColor={0,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={255,206,120}),
            Rectangle(
              extent={{-40,60},{-60,-60}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={128,128,128}),
            Rectangle(
              extent={{80,10},{100,-10}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={95,95,95}),
            Rectangle(
              extent={{-40,70},{40,50}},
              lineColor={95,95,95},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-50,-90},{-40,-90},{-10,-20},{40,-20},{70,-90},{80,-90},{80,
                  -100},{-50,-100},{-50,-90}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
This icon is designed for a <b>FundamentalWave machine</b> model.
</p>
</html>"));
    end QuasiStationaryFundamentalWaveMachine;
  end Icons;


  annotation (uses(Modelica(version="3.2.1"),
                   Complex(version="3.2.1")),
    version="0.2.2",
    versionDate="2013-09-24",
    versionBuild=0);
end QuasiStationaryFundamentalWave;
