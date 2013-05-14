within ;
package QuasiStationaryFundamentalWave 
  package UsersGuide "User's Guide"
    extends Modelica.Icons.Information;
    class Concept "Fundamental wave concept"
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
private contribtuion<br>
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

  extends Modelica.Icons.Package;


  package Examples "Examples"
  extends Modelica.Icons.ExamplesPackage;
  package Components
    "Examples for testing quasi stationary fundamental wave components"
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
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m,
            R_ref=fill(R, m))
        annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m,
            R_ref=fill(R, m))
        annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor inductor_e(m=m, L=
              fill(L, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,70})));
        QuasiStationaryFundamentalWave.Components.ThreePhaseElectroMagneticConverter
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
    extends Modelica.Icons.ExamplesPackage;
      model SinglePhaseInductance "Single phase inductance"
        extends Modelica.Icons.Example;

        parameter Modelica.SIunits.Frequency f = 1 "Supply frequency";
        parameter Modelica.SIunits.Voltage VRMS = 100 "RMS supply voltage";
        parameter Modelica.SIunits.Resistance R = 1E-5 "Resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load inductance";
        parameter Real effectiveTurns = 5 "Effective number of turns";
        // Single phase magnetic reluctance
        final parameter Modelica.SIunits.Reluctance R_m = effectiveTurns^2/L
        "Equivalent magnetic reluctance";

        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_e
          annotation (Placement(transformation(extent={{-70,0},{-50,20}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground ground_m
          annotation (Placement(transformation(extent={{-70,-80},{-50,-60}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource voltageSource_e(
          phi=0,
          f=f,
          V=VRMS)
                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,50})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Sources.VoltageSource voltageSource_m(
          phi=0,
          f=f,
          V=VRMS)
                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-30})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor_e(R_ref=
              R)
          annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Resistor resistor_m(R_ref=
              R)
          annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Inductor inductor_e(L=L)
                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,50})));
        QuasiStationaryFundamentalWave.Components.SinglePhaseElectroMagneticConverter
          converter_m(effectiveTurns=effectiveTurns)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        QuasiStationaryFundamentalWave.Components.Reluctance reluctance_m(R_m(d=R_m,
              q=R_m))                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={60,-30})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_m
          annotation (Placement(transformation(extent={{10,-80},{30,-60}})));
      equation
        connect(voltageSource_m.pin_n, ground_m.pin)
                                                   annotation (Line(
            points={{-60,-40},{-60,-60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_m.pin_n, converter_m.pin_n)
                                                        annotation (Line(
            points={{-60,-40},{0,-40}},
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
            points={{20,-40},{20,-60}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(voltageSource_e.pin_n, ground_e.pin)
                                                   annotation (Line(
            points={{-60,40},{-60,20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.pin_n, inductor_e.pin_n)
                                                      annotation (Line(
            points={{-60,40},{0,40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_e.pin_p, resistor_e.pin_p)
                                                      annotation (Line(
            points={{-60,60},{-40,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_e.pin_n, inductor_e.pin_p)
                                                annotation (Line(
            points={{-20,60},{2.44249e-15,60}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(voltageSource_m.pin_p, resistor_m.pin_p) annotation (Line(
            points={{-60,-20},{-40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_m.pin_n, converter_m.pin_p) annotation (Line(
            points={{-20,-20},{0,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}),
                            graphics), Icon(coordinateSystem(extent={{-100,-100},
                  {100,100}})),
          experiment(StopTime=100, Interval=0.01));
      end SinglePhaseInductance;

      model MultiPhaseInductance "Multi phase inductance"
      import QuasiStationaryFundamentalWave;
        extends Modelica.Icons.Example;
        extends Modelica.Icons.UnderConstruction;

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
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m,
            R_ref=fill(R, m))
        annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m,
            R_ref=fill(R, m))
        annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Inductor inductor_e(m=m, L=
              fill(L, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,70})));
        QuasiStationaryFundamentalWave.Components.MultiPhaseElectroMagneticConverter
                                                                          converter_m(
          m=m,
          effectiveTurns=fill(effectiveTurns, m))
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
        extends Modelica.Icons.UnderConstruction;

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
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_e(m=m,
            R_ref=fill(R, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-60,70})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_m(m=m,
            R_ref=fill(R, m))             annotation (Placement(transformation(
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
          m=m,
          G_ref=fill(Gc, m))
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,60})));
        QuasiStationaryFundamentalWave.Components.ThreePhaseElectroMagneticConverter
          converter_e(
          effectiveTurns=N)
          annotation (Placement(transformation(extent={{20,50},{40,70}})));
        QuasiStationaryFundamentalWave.Components.ThreePhaseElectroMagneticConverter
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
        extends Modelica.Icons.UnderConstruction;

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
        extends Modelica.Icons.UnderConstruction;

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
        extends Modelica.Icons.UnderConstruction;
        import Modelica.Constants.pi;
        constant Integer m=3 "Number of phases";
        // ## Original value R = 0.1 Ohm
        parameter Modelica.SIunits.Resistance R=1E-5 "Resistance";
        parameter Modelica.SIunits.Conductance Gc=1 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";

        Modelica.Electrical.Analog.Basic.Ground ground_t
          annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_f
          annotation (Placement(transformation(extent={{-130,-90},{-110,-70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star_t(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_f(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,-60})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_t(
          m=m,
          V=fill(1, m),
          freqHz=fill(1, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m) +
              fill(pi/2, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_f(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_t(m=m, R=fill(R, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-100,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_f(m=m,
            R_ref=fill(R, m))             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-100,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerb_t(m=m)
          annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_f(m=m)
          annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_t(
          m=m,
          effectiveTurns=fill(N, m),
          orientation=Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
        QuasiStationaryFundamentalWave.Components.ThreePhaseElectroMagneticConverter
          converter_f(
          effectiveTurns=N)
          annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
        Modelica.Magnetic.FundamentalWave.Components.Ground groundM_t
          annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_f
          annotation (Placement(transformation(extent={{-30,-76},{-10,-56}})));
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
        connect(sineVoltage_t.plug_n, star_t.plug_p) annotation (Line(
            points={{-120,40},{-120,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star_t.pin_n, ground_t.p) annotation (Line(
            points={{-120,10},{-120,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, converter_t.plug_n) annotation (Line(
            points={{-120,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_p, resistor_t.plug_p)
                                                       annotation (Line(
            points={{-120,60},{-110,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor_t.plug_n, powerb_t.pc)
                                              annotation (Line(
            points={{-90,60},{-80,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.pc, powerb_t.pv) annotation (Line(
            points={{-80,60},{-80,70},{-70,70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nc, converter_t.plug_p) annotation (Line(
            points={{-60,60},{-40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nv, sineVoltage_t.plug_n) annotation (Line(
            points={{-70,50},{-70,40},{-120,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(converter_t.port_n, reluctance_t.port_n) annotation (Line(
            points={{-20,40},{40,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_t.port_n, groundM_t.port_p) annotation (Line(
            points={{-20,40},{-20,40},{-20,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,star_f. plug_p) annotation (Line(
            points={{-120,-40},{-120,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,converter_f. plug_n) annotation (Line(
            points={{-120,-40},{-40,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_p,resistor_f. plug_p)
                                                       annotation (Line(
            points={{-120,-20},{-110,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_f.plug_n,powerb_f. currentP)
                                                    annotation (Line(
            points={{-90,-20},{-80,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentN,converter_f. plug_p) annotation (Line(
            points={{-60,-20},{-40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_f.pin_n,ground_f. pin) annotation (Line(
            points={{-120,-70},{-120,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentP,powerb_f. voltageP) annotation (Line(
            points={{-80,-20},{-80,-10},{-70,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.voltageN,sineVoltage_f. plug_n) annotation (Line(
            points={{-70,-30},{-70,-40},{-120,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_f.port_n, reluctance_f.port_n) annotation (Line(
            points={{-20,-40},{40,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, groundM_f.port_p) annotation (Line(
            points={{-20,-40},{-20,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_t.port_p, reluctance_t.port_p) annotation (Line(
            points={{-20,60},{40,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_f.port_p, reluctance_f.port_p) annotation (Line(
            points={{-20,-20},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                  -100},{60,100}}),  graphics),
          Icon(coordinateSystem(extent={{-140,-100},{60,100}})));
      end Example3;

      model Example4
        extends Modelica.Icons.Example;
        extends Modelica.Icons.UnderConstruction;
        constant Integer m=3 "Number of phases";
        // ## Original value R = 0.1 Ohm
        parameter Modelica.SIunits.Resistance R=0.1 "Resistance";
        parameter Modelica.SIunits.Conductance Gc=1 "Loss conductance";
        parameter Modelica.SIunits.Reluctance R_m=1
        "Reluctance of the magnetic circuit";
        parameter Real N=1 "Number of turns";

        Modelica.Electrical.Analog.Basic.Ground ground_t
          annotation (Placement(transformation(extent={{-130,-10},{-110,10}})));
        Modelica.Electrical.QuasiStationary.SinglePhase.Basic.Ground
          ground_f
          annotation (Placement(transformation(extent={{-130,-90},{-110,-70}})));
        Modelica.Electrical.MultiPhase.Basic.Star star_t(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,20})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Star
          star_f(m=m) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,-60})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage_t(
          m=m,
          V=fill(1, m),
          freqHz=fill(1, m)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,50})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sources.VoltageSource
          sineVoltage_f(
          m=m,
          f=1,
          phi=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m),
          V=fill(1/sqrt(2), m))
                             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-120,-30})));
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor_t(m=m, R=fill(R, m))
                     annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-100,60})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Basic.Resistor resistor_f(m=m,
            R_ref=fill(R, m))             annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-100,-20})));
        Modelica.Electrical.MultiPhase.Sensors.PowerSensor powerb_t(m=m)
          annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
        Modelica.Electrical.QuasiStationary.MultiPhase.Sensors.PowerSensor
          powerb_f(m=m)
          annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
        Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter
          converter_t(
          m=m,
          effectiveTurns=fill(N, m),
          orientation=Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m))
          annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
        QuasiStationaryFundamentalWave.Components.ThreePhaseElectroMagneticConverter
          converter_f(
          effectiveTurns=N)
          annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
        Modelica.Magnetic.FundamentalWave.Components.Ground groundM_t
          annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
        QuasiStationaryFundamentalWave.Components.Ground groundM_f
          annotation (Placement(transformation(extent={{-30,-76},{-10,-56}})));
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
        connect(eddyCurrent_t.port_n, reluctance_t.port_p) annotation (Line(
            points={{20,60},{40,60}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, star_t.plug_p) annotation (Line(
            points={{-120,40},{-120,30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star_t.pin_n, ground_t.p) annotation (Line(
            points={{-120,10},{-120,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_n, converter_t.plug_n) annotation (Line(
            points={{-120,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage_t.plug_p, resistor_t.plug_p)
                                                       annotation (Line(
            points={{-120,60},{-110,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor_t.plug_n, powerb_t.pc)
                                              annotation (Line(
            points={{-90,60},{-80,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.pc, powerb_t.pv) annotation (Line(
            points={{-80,60},{-80,70},{-70,70}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nc, converter_t.plug_p) annotation (Line(
            points={{-60,60},{-40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(powerb_t.nv, sineVoltage_t.plug_n) annotation (Line(
            points={{-70,50},{-70,40},{-120,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(converter_t.port_n, reluctance_t.port_n) annotation (Line(
            points={{-20,40},{40,40}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(converter_t.port_n, groundM_t.port_p) annotation (Line(
            points={{-20,40},{-20,20}},
            color={255,128,0},
            smooth=Smooth.None));
        connect(eddyCurrent_f.port_n, reluctance_f.port_p) annotation (Line(
            points={{20,-20},{40,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,star_f. plug_p) annotation (Line(
            points={{-120,-40},{-120,-50}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_n,converter_f. plug_n) annotation (Line(
            points={{-120,-40},{-40,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(sineVoltage_f.plug_p,resistor_f. plug_p)
                                                       annotation (Line(
            points={{-120,-20},{-110,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(resistor_f.plug_n,powerb_f. currentP)
                                                    annotation (Line(
            points={{-90,-20},{-80,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentN,converter_f. plug_p) annotation (Line(
            points={{-60,-20},{-40,-20}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(star_f.pin_n,ground_f. pin) annotation (Line(
            points={{-120,-70},{-120,-70}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.currentP,powerb_f. voltageP) annotation (Line(
            points={{-80,-20},{-80,-10},{-70,-10}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(powerb_f.voltageN,sineVoltage_f. plug_n) annotation (Line(
            points={{-70,-30},{-70,-40},{-120,-40}},
            color={85,170,255},
            smooth=Smooth.None));
        connect(converter_f.port_p, eddyCurrent_f.port_p) annotation (Line(
            points={{-20,-20},{0,-20}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, reluctance_f.port_n) annotation (Line(
            points={{-20,-40},{40,-40}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_f.port_n, groundM_f.port_p) annotation (Line(
            points={{-20,-40},{-20,-56}},
            color={255,170,85},
            smooth=Smooth.None));
        connect(converter_t.port_p, eddyCurrent_t.port_p) annotation (Line(
            points={{-20,60},{0,60}},
            color={255,128,0},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
                  -100},{60,100}}),  graphics),
          Icon(coordinateSystem(extent={{-140,-100},{60,100}})));
      end Example4;
    end ToBeRemovedLater;
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

    model SinglePhaseElectroMagneticConverter
    "Single phase electro magnetic converter"

      import Modelica.Constants.pi;
      constant Complex j = Complex(0,1);

      Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.PositivePin
        pin_p "Positive pin"
        annotation (Placement(transformation(
            origin={-100,100},
            extent={{-10,-10},{10,10}},
            rotation=180)));
      Modelica.Electrical.QuasiStationary.SinglePhase.Interfaces.NegativePin
        pin_n "Negative pin"
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

      parameter Real effectiveTurns "Effective number of turns";
      // IMPORTANT NOTE
      // orientation shall not be implemented in the final version of this
      // library, so that the user cannot build a three phase electromagnetic
      // converter model based in three single phase converters
      parameter Modelica.SIunits.Angle orientation = 0
      "Orientation of the resulting fundamental wave field phasor";

      // Local electric single phase quantities
      Modelica.SIunits.ComplexVoltage v "Voltage drop";
      Modelica.SIunits.ComplexCurrent i "Current";

      // Local electromagnetic fundamental wave quantities
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";

      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma)
      "Angular velocity";

      final parameter Complex N=
        effectiveTurns * Modelica.ComplexMath.exp(Complex(0,orientation))
      "Complex number of turns";

    equation
      // Magnetic flux and flux balance of the magnetic ports
      port_p.Phi = Phi;
      port_p.Phi + port_n.Phi = Complex(0,0);

      // Magnetic potential difference of the magnetic ports
      port_p.V_m - port_n.V_m = V_m;

      // Voltage drop between the electrical pins
      v = pin_p.v - pin_n.v;

      // Current and current balance of the electric pins
      i = pin_p.i;
      pin_p.i + pin_n.i = Complex(0,0);

      // Complex magnetic potential difference from currents, number
      // of turns and angles of orientation of winding
      // V_m.re = (2/pi) * effectiveTurns*cos(orientation)*i;
      // V_m.im = (2/pi) * effectiveTurns*sin(orientation)*i;
      V_m = sqrt(2) * (2.0/pi) * N * i;

      // Induced voltages from complex magnetic flux, number of turns
      // and angles of orientation of winding
      -v*sqrt(2) = Modelica.ComplexMath.conj(N)*j*omega*Phi;

      Connections.potentialRoot(pin_p.reference);
      // Connections.potentialRoot(pin_n.reference);
      Connections.potentialRoot(port_p.reference);
      // Connections.potentialRoot(port_n.reference);

      Connections.branch(port_p.reference, port_n.reference);
      port_p.reference.gamma = port_n.reference.gamma;

      Connections.branch(pin_p.reference, pin_n.reference);
      pin_p.reference.gamma = pin_n.reference.gamma;

      Connections.branch(pin_p.reference, port_p.reference);
      pin_p.reference.gamma = port_p.reference.gamma;

      annotation (Diagram(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics),
                           Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={
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
The single phase winding has an effective number of turns, <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/effectiveTurns.png\"> and a respective orientation of the winding, <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/orientation.png\">. The current in winding is <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/i.png\">.
</p>

<p>
The total complex magnetic potential difference of the single phase winding is determined by:
</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/singlephaseconverter_vm.png\">
</p>

<p>
In this equation the magneto motive force is aligned with the orientation of the winding.
</p>

<p>
The voltage <img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/v.png\"> induced in the winding depends on the cosine between the orientation of the winding and the angle of the complex magnetic flux. Additionally, the magnitudes of the induced voltages are propotional to the respective number of turns. This relationship can be modeled by means of</p>

<p>
&nbsp;&nbsp;<img src=\"modelica://Modelica/Images/Magnetic/FundamentalWave/Components/singlephaseconverter_phi.png\">
</p>

<p>The single phase electro magnetic converter is a special case of the
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

<h4>See also</h4>
<p>
<a href=\"modelica://Modelica.Magnetic.FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

</html>"));
    end SinglePhaseElectroMagneticConverter;

    model ThreePhaseElectroMagneticConverter
    "Three phase electro magnetic converter"

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

      constant Integer m = 3 "Number of phases";
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

      // Transformation matrix for symmetrical components
      final parameter Complex T[m,m]= 1/m *
        {{Modelica.ComplexMath.exp(Complex(0,2*pi/m*((k-1)*(l-1))))
          for k in 1:m} for l in 1:m};

      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma);

      // A technical solution with a rotator cannot be applied to the equations below
      final parameter Complex N=
        effectiveTurns*Modelica.ComplexMath.exp(Complex(0,orientation))
      "Complex effective number of turns";

      Modelica.SIunits.ComplexVoltage vSymmetricalComponent[m] = T*v
      "Symmetrical components of voltages";
      Modelica.SIunits.ComplexCurrent iSymmetricalComponent[m] = T*i
      "Symmetrical components of currents";

      // NOTE
      // Assert of asymmetric component iSymmetricalComponent[1] <> 0 and
      // iSymmetricalComponent[3] <> 0 have to be included in the future!
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

      V_m.re = sqrt(2) * (2.0/pi) * Modelica.ComplexMath.real(N*iSymmetricalComponent[2]);
      V_m.im = sqrt(2) * (2.0/pi) * Modelica.ComplexMath.imag(N*iSymmetricalComponent[2]);

      iSymmetricalComponent[1] = Complex(0,0);
      iSymmetricalComponent[3] = Complex(0,0);

      // Induced voltages from complex magnetic flux, number of turns
      // and angles of orientation of winding
      -sqrt(2) * vSymmetricalComponent[2] = Modelica.ComplexMath.conj(N)*j*omega*Phi;

      Connections.potentialRoot(plug_p.reference);
      Connections.potentialRoot(port_p.reference);

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
    end ThreePhaseElectroMagneticConverter;

    model MultiPhaseElectroMagneticConverter
    "Multi phase electro magnetic converter"

      extends Modelica.Icons.UnderConstruction;

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
        annotation (Placement(transformation(extent={{90,90},{110,110}},
          rotation=0)));
      QuasiStationaryFundamentalWave.Interfaces.NegativeMagneticPort port_n
      "Negative complex magnetic port"
        annotation (Placement(transformation(extent={{90,-110},{110,-90}},
          rotation=0)));

      // Global plug and port variables
      Modelica.SIunits.ComplexVoltage v[m] = plug_p.pin.v - plug_n.pin.v
      "Voltages";
      Modelica.SIunits.ComplexCurrent i[m] = plug_p.pin.i "Currents";
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m = port_p.V_m - port_n.V_m
      "Magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi = port_p.Phi "Magnetic flux";

      parameter Integer m = 3 "Number of phases";
      parameter Real effectiveTurns[m] "Effective number of turns";
      final parameter Modelica.SIunits.Angle orientation[m]=
        Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(m)
      "Orientation of the resulting fundamental wave field phasor";

      // A technical solution with a rotator cannot be applied to the equations below
      final parameter Complex N[m] = {effectiveTurns[k]*Modelica.ComplexMath.exp(Complex(0,orientation[k])) for k in 1:m}
      "Complex effective number of turns";

      SinglePhaseElectroMagneticConverter singlePhaseElectroMagneticConverter[m](
        final effectiveTurns=effectiveTurns,
        final orientation=orientation)
        annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
      Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_p
        plugToPins_p(final m=m)
        annotation (Placement(transformation(extent={{-70,0},{-50,20}},   rotation=
                0)));
      Modelica.Electrical.QuasiStationary.MultiPhase.Basic.PlugToPins_n
        plugToPins_n(final m=m)
        annotation (Placement(transformation(
            origin={-60,-10},
            extent={{10,-10},{-10,10}},
            rotation=180)));
    equation
      connect(plugToPins_p.pin_p, singlePhaseElectroMagneticConverter.pin_p)
        annotation (Line(
          points={{-58,10},{-8,10}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(plugToPins_n.pin_n, singlePhaseElectroMagneticConverter.pin_n)
        annotation (Line(
          points={{-58,-10},{-8,-10}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(plugToPins_p.plug_p, plug_p) annotation (Line(
          points={{-62,10},{-100,10},{-100,100}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(plugToPins_n.plug_n, plug_n) annotation (Line(
          points={{-62,-10},{-100,-10},{-100,-100}},
          color={85,170,255},
          smooth=Smooth.None));
      connect(singlePhaseElectroMagneticConverter[1].port_p, port_p) annotation (
          Line(
          points={{12,10},{100,10},{100,100}},
          color={255,170,85},
          smooth=Smooth.None));
      for k in 2:m loop
        connect(singlePhaseElectroMagneticConverter[k-1].port_n,singlePhaseElectroMagneticConverter[k].port_p);
      end for;
      connect(singlePhaseElectroMagneticConverter[m].port_n, port_n) annotation (
          Line(
          points={{12,-10},{100,-10},{100,-100}},
          color={255,170,85},
          smooth=Smooth.None));

      annotation (         Icon(coordinateSystem(preserveAspectRatio=true,
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
</html>"),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}),
                graphics));
    end MultiPhaseElectroMagneticConverter;

    model MultiConverterDuplicate "Multi phase electro magnetic converter"

      extends Modelica.Icons.UnderConstruction;

      import Modelica.Constants.pi;
      constant Complex j = Complex(0,1);

      // constant Modelica.SIunits.Angle offset = 0.0000
      //   "Development constant to be removed in the final version";
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
      parameter Real effectiveTurns[m] "Effective number of turns";
      parameter Modelica.SIunits.Angle orientation[m]
      "Orientation of the resulting fundamental wave field phasor";

      // Local electric multi phase quantities
      Modelica.SIunits.ComplexVoltage v[m] "Voltage drop";
      Modelica.SIunits.ComplexCurrent i[m] "Current";

      final parameter Complex T[m,m] = {{Modelica.ComplexMath.exp(Complex(0,Modelica.Constants.pi/m))^((k-1)*(l-1))
        for k in 1:m} for l in 1:m};

      // Local electromagnetic fundamental wave quantities
      Modelica.SIunits.ComplexMagneticPotentialDifference V_m
      "Complex magnetic potential difference";
      Modelica.SIunits.ComplexMagneticFlux Phi "Complex magnetic flux";

      Modelica.SIunits.AngularVelocity omega = der(port_p.reference.gamma);

      // A technical solution with a rotator cannot be applied to the equations below
      final parameter Complex N[m] = {effectiveTurns[k]*Modelica.ComplexMath.exp(Complex(0,orientation[k])) for k in 1:m}
      "Complex effective number of turns";

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

      V_m.re = (2.0/pi) * sum( Modelica.ComplexMath.real(N[k]*i[k]) for k in 1:m);
      V_m.im = (2.0/pi) * sum( Modelica.ComplexMath.imag(N[k]*i[k]) for k in 1:m);

      // Induced voltages from complex magnetic flux, number of turns
      // and angles of orientation of winding
      for k in 1:m loop
        v[k] = Modelica.ComplexMath.conj(N[k])*j*omega*Phi;
      end for;

      Connections.potentialRoot(plug_p.reference);
      Connections.potentialRoot(port_p.reference);

      // --- Implementation from Nick Raabe seems not to be valid
      // Connections.branch(port_p.reference, plug_p.reference);
      // port_p.reference.gamma = port_n.reference.gamma;
      // Connections.branch(port_n.reference, plug_n.reference);
      // port_n.reference.gamma = plug_n.reference.gamma;

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
    end MultiConverterDuplicate;

    model Idle "Salient reluctance"
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

    model Short "Salient reluctance"
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
    annotation (DymolaStoredErrors, Documentation(info="<html>
<p>Basic components of the FundamentalWave library for modeling magnetic circuits. Machine specific components are
located at <a href=\"modelica://Modelica.Magnetic.FundamentalWave.BasicMachines.Components\">Machines.Components</a>.</p>
</html>"));
  end Components;


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
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString=
                   "%name"), Ellipse(
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
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString=
                   "%name"), Ellipse(
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
  end Interfaces;


  annotation (uses(Modelica(version="3.2.1"), Complex(version="3.2.1")));
end QuasiStationaryFundamentalWave;