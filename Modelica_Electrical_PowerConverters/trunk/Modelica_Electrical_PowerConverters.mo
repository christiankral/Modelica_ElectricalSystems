within ;
package Modelica_Electrical_PowerConverters "Rectifiers and DC/DC converters"
  extends Modelica.Icons.Package;
  package Examples
    extends Modelica.Icons.ExamplesPackage;
    package FullBridge1Pulse "Single pulse rectifier"
      extends Modelica.Icons.ExamplesPackage;
      model Impulses "Generic test of impulses"
        extends Modelica.Icons.Example;
        Control.TwoPulse twoPulse(f=10, alpha=0.5235987755983) annotation (
            Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,0})));
        Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage(freqHz=10)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={0,40})));
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
      equation
        connect(sineVoltage.p, twoPulse.p) annotation (Line(
            points={{10,40},{20,40},{20,-1.77636e-15},{10,-1.77636e-15}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage.n, twoPulse.n) annotation (Line(
            points={{-10,40},{-20,40},{-20,6.66134e-16},{-10,6.66134e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(ground.p, sineVoltage.n) annotation (Line(
            points={{-40,40},{-10,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{
                  -100,-100},{100,100}}), graphics));
      end Impulses;

      model FullBridge1Pulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge1Pulse(
            twoPulse(
            useSignal=false,
            f=f,
            alpha=alpha), idealthyristor(Vknee=0));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(idealthyristor.n, resistor.p) annotation (Line(
            points={{4.44089e-16,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=5000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse_R;

      model FullBridge1Pulse_R_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge1Pulse(
            twoPulse(useSignal=true, f=f), idealthyristor(Vknee=0));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      equation
        connect(ramp.y, twoPulse.u) annotation (Line(
            points={{-59,70},{-56,70},{-56,0},{-50,0}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.p, idealthyristor.n) annotation (Line(
            points={{30,40},{0,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse_R_Characteristic;
    end FullBridge1Pulse;

    package FullBridge2PulseCenterTap "Examples of Power Electronics with M2C"
      extends Modelica.Icons.ExamplesPackage;
      model FullBridge2PulseCenterTap_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
           b2C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,4.44089e-16},{-10,4.44089e-16},{-10,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_R;

      model FullBridge2PulseCenterTap_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge6Pulse(
                                                                            b6C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b6C.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RL;

      model FullBridge2PulseCenterTap_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
                                                                            b2C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,0},{-10,0},{-10,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
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
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RLV;

      model FullBridge2PulseCenterTap_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2PulseCenterTap(
                                                                            b2C(
              useSignal=true, f=f));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,50})));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,0},{-10,0},{-10,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
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
        connect(ramp.y, b2C.u) annotation (Line(
            points={{-40,39},{-40,10}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap_RLV_Characteristic;
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})));
    end FullBridge2PulseCenterTap;

    package FullBridge2Pulse "Two pulse Graetz bridge"
      extends Modelica.Icons.ExamplesPackage;
      model FullBridge2Pulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
                                                                            b2C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_R;

      model FullBridge2Pulse_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
                                                                            b2C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RL;

      model FullBridge2Pulse_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
                                                                            b2C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
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
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RLV;

      model FullBridge2Pulse_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridgeTwoPulse(
                                                                            b2C(
              useSignal=true, f=f));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-40,50})));
      equation
        connect(ramp.y, b2C.u) annotation (Line(
            points={{-40,39},{-40,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(b2C.dc_p, resistor.p) annotation (Line(
            points={{-30,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10},{30,10}},
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
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2Pulse_RLV_Characteristic;
    end FullBridge2Pulse;

    package FullBridge6Pulse
      extends Modelica.Icons.ExamplesPackage;
      model Impulses
        extends Modelica.Icons.Example;
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));
        Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          m=3,
          V=fill(sqrt(2)*100, 3),
          freqHz=fill(1/360, 3)) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,30})));
        Control.SixPulse sixPulse(f=1/360, alpha=0.017453292519943)
          annotation (Placement(transformation(extent={{-20,30},{0,50}})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,20},{-80,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-20},{-80,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, sixPulse.ac) annotation (Line(
            points={{-80,40},{-20,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=360),
          __Dymola_experimentSetupOutput);
      end Impulses;

      model FullBridge6Pulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge6Pulse(
                                                                            b6C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(b6C.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge6Pulse_R;

      model FullBridge6Pulse_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge6Pulse(
                                                                            b6C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(b6C.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge6Pulse_RL;

      model FullBridge6Pulse_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge6Pulse(
                                                                            b6C(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
      equation
        connect(resistor.p, b6C.dc_p) annotation (Line(
            points={{30,40},{-20,40},{-20,6},{-40,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, constantVoltage.p) annotation (Line(
            points={{30,-10},{30,-10},{30,-20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(constantVoltage.n, currentSensor.p) annotation (Line(
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge6Pulse_RLV;

      model FullBridge6Pulse_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge6Pulse(
                                                                            b6C(
              useSignal=true, f=f));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,70})));
      equation
        connect(ramp.y, b6C.u) annotation (Line(
            points={{-50,59},{-50,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(b6C.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
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
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=10, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge6Pulse_RLV_Characteristic;
    end FullBridge6Pulse;

    package FullBridge2mPulse
      extends Modelica.Icons.ExamplesPackage;
      model Impulses
        extends Modelica.Icons.Example;
        parameter Integer m(final min=3)=3 "Number of phases";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
                                                       annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          V=fill(sqrt(2)*100, m),
          freqHz=fill(1/360, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m))                annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,30})));
        Control.Pulse2m   twomPulse(
          f=1/360,
          final m=m,
          alpha=0.017453292519943)
          annotation (Placement(transformation(extent={{-20,40},{0,60}})));
        Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=m)
                                                               annotation (
            Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=90,
              origin={-60,50})));
        Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=m)
          annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=270,
              origin={-40,50})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,20},{-80,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-20},{-80,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, deltaP.plug_n) annotation (Line(
            points={{-80,40},{-60,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(deltaP.plug_n, voltageSensorP.plug_p) annotation (Line(
            points={{-60,40},{-40,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
            points={{-60,60},{-50,60},{-50,60},{-40,60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltageSensorP.v, twomPulse.v) annotation (Line(
            points={{-29,50},{-20,50}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=360),
          __Dymola_experimentSetupOutput);
      end Impulses;

      model FullBridge2mPulse_R
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            b2mC(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)
          annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, currentSensor.p) annotation (Line(
            points={{30,20},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2mC.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_R;

      model FullBridge2mPulse_RL
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            b2mC(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
      equation
        connect(resistor.n, inductor.p) annotation (Line(
            points={{30,20},{30,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(inductor.n, currentSensor.p) annotation (Line(
            points={{30,-10},{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2mC.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RL;

      model FullBridge2mPulse_RLV
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            b2mC(
            useSignal=false,
            f=f,
            alpha=alpha));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
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
            points={{30,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2mC.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.1, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RLV;

      model FullBridge2mPulse_RLV_Characteristic
        extends
          Modelica_Electrical_PowerConverters.Examples.ExampleTemplates.FullBridge2mPulse(
            b2mC(useSignal=true, f=f));
        extends Modelica.Icons.Example;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Resistor resistor(R=R)    annotation(Placement(visible = true, transformation(origin={30,30},                     extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Basic.Inductor inductor(L=L)    annotation(Placement(visible = true, transformation(origin={30,0},         extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={30,-30})));
        Modelica.Blocks.Sources.Ramp ramp(height=pi, duration=10)
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-50,70})));
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
        connect(ramp.y, b2mC.u) annotation (Line(
            points={{-50,59},{-50,10},{-50,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(b2mC.dc_p, resistor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{30,40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=10, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput);
      end FullBridge2mPulse_RLV_Characteristic;
    end FullBridge2mPulse;

    package ExampleTemplates
      partial model FullBridge1Pulse
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor  annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*Vrms,
            freqHz=f) annotation (Placement(visible=true, transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(visible=true,
              transformation(
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
        Control.TwoPulse twoPulse
         annotation (Placement(
              transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-40,0})));
        Modelica.Electrical.Analog.Ideal.IdealThyristor idealthyristor annotation (
            Placement(visible=true, transformation(
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
        connect(twoPulse.n, ground.p) annotation (Line(
            points={{-40,-10},{-40,-40},{-80,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(twoPulse.p, sinevoltage.p) annotation (Line(
            points={{-40,10},{-40,40},{-80,40},{-80,10}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(twoPulse.yPositive, idealthyristor.fire) annotation (Line(
            points={{-30,4},{-3,4},{-3,29}},
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
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=0.1,
            __Dymola_NumberOfIntervals=5000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput,
          Documentation(info="<html>
<p>Inductive load does not make sense, since average DC voltage is very low due to long conduction period of the thyristor. </p>
</html>"));
      end FullBridge1Pulse;
      extends Modelica.Icons.Package;
      model FullBridge2PulseCenterTap
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(visible=true,
              transformation(
              origin={-90,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage2(V = sqrt(2) * Vrms, freqHz = f)
          annotation(Placement(visible = true, transformation(origin={-80,
                  -13.9999},                                                                                                    extent={{
                  -9.99989,-10},{10,10}},                                                                                                    rotation = -90)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage1(V = sqrt(2) * Vrms, freqHz = f)
          annotation(Placement(visible = true, transformation(origin={-80,14},             extent={{-10,-10},
                  {10,10}},                                                                                                    rotation = -90)));
        Modelica_Electrical_PowerConverters.ACDC.FullBridge2PulseCenterTap b2C
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f)
          annotation (Placement(
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
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
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
        connect(sinevoltage1.p, b2C.ac_p) annotation (Line(
            points={{-80,24},{-80,32},{-60,32},{-60,6},{-50,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage2.n, b2C.ac_n) annotation (Line(
            points={{-80,-23.9999},{-80,-30},{-60,-30},{-60,-6},{-50,-6}},
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
        connect(voltagesensor.p, b2C.dc_p) annotation (Line(
            points={{50,20},{50,40},{-10,40},{-10,0},{-30,0}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridge2PulseCenterTap;

      partial model FullBridgeTwoPulse "Template of B2C without load"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-120 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(visible=true,
              transformation(
              origin={-80,-50},
              extent={{-10,-10},{10,10}},
              rotation=0)));
        Modelica.Electrical.Analog.Sources.SineVoltage sinevoltage(V=sqrt(2)*Vrms,
            freqHz=f) annotation (Placement(visible=true, transformation(
              origin={-80,0},
              extent={{-10,-10},{10,10}},
              rotation=-90)));
        Modelica_Electrical_PowerConverters.ACDC.HalfBridge2Pulse b2C(
            useHeatPort=false)
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor  annotation(Placement(visible = true, transformation(origin={50,10},          extent={{10,-10},
                  {-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*f)
          annotation (Placement(
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
        Modelica.Blocks.Math.Mean meanCurrent(f=2*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
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
        connect(sinevoltage.p, b2C.ac_p) annotation (Line(
            points={{-80,10},{-62,10},{-62,6},{-50,6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(sinevoltage.n, b2C.ac_n) annotation (Line(
            points={{-80,-10},{-62,-10},{-62,-6},{-50,-6}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2C.dc_n, currentSensor.n) annotation (Line(
            points={{-29.8,-6},{-20,-6},{-20,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2C.dc_p, voltagesensor.p) annotation (Line(
            points={{-30,6},{-20,6},{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(voltagesensor.n, currentSensor.p) annotation (Line(
            points={{50,0},{50,-40},{10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Diagram(coordinateSystem(extent={{-100,
                  -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
              graphics),                                                                                                    experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=50000,
            Tolerance=1e-06),
          __Dymola_experimentSetupOutput);
      end FullBridgeTwoPulse;

      partial model FullBridge6Pulse "Template of B6C without load"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
        Modelica.Electrical.MultiPhase.Basic.Star star annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-40})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          m=3,
          V=fill(sqrt(2)*Vrms, 3),
          freqHz=fill(f, 3),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              3)) annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        Modelica.Electrical.MultiPhase.Basic.Delta delta annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-92,30})));
        Modelica.Electrical.MultiPhase.Sensors.VoltageSensor suppylVoltageSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-70,30})));
        ACDC.FullBridge2mPulse                       b6C
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation(Placement(visible = true, transformation(origin={50,10},
            extent={{10,-10},{-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=6*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,40})));
        Modelica.Blocks.Math.RootMeanSquare rootMeanSquareVoltage(f=6*f)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,10})));
        Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=180,
              origin={0,-40})));
        Modelica.Blocks.Math.Mean meanCurrent(f=6*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,-20},{-80,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-50},{-80,-60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(meanCurrent.u,currentSensor. i) annotation (Line(
            points={{68,-60},{0,-60},{0,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,meanVoltage. u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,rootMeanSquareVoltage. u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, b6C.ac) annotation (Line(
            points={{-80,0},{-60,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b6C.dc_n, currentSensor.n) annotation (Line(
            points={{-39.8,-6},{-20,-6},{-20,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b6C.dc_p, voltagesensor.p) annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_p, sineVoltage.plug_p) annotation (Line(
            points={{-92,20},{-92,10},{-80,10},{-80,4.44089e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_n, suppylVoltageSensor.plug_p) annotation (Line(
            points={{-92,40},{-70,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_p, suppylVoltageSensor.plug_n) annotation (Line(
            points={{-92,20},{-70,20}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.04, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics));
      end FullBridge6Pulse;

      partial model FullBridge2mPulse "Template of B2*mC without load"
        extends Modelica_Electrical_PowerConverters.Icons.ExampleTemplate;
        import Modelica.Constants.pi;
        parameter Integer m(final min=3)=3 "Number of phases";
        parameter Modelica.SIunits.Voltage Vrms = 110 "RMS supply voltage";
        parameter Modelica.SIunits.Frequency f = 50 "Frequency";
        // parameter Modelica.SIunits.Angle alpha = 90 * pi / 180 "Firing angle";
        // parameter Modelica.SIunits.Resistance R = 20 "Load resistance";
        // parameter Modelica.SIunits.Inductance L = 1 "Load resistance" annotation(Evaluate=true);
        // parameter Modelica.SIunits.Voltage VDC=-260 "DC load offset voltage";
        Modelica.Electrical.Analog.Basic.Ground ground
          annotation (Placement(transformation(extent={{-90,-80},{-70,-60}})));
        Modelica.Electrical.MultiPhase.Basic.Star star(final m=m)
                                                       annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-40})));
        Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
          final m=m,
          V=fill(sqrt(2)*Vrms, m),
          phase=-Modelica.Electrical.MultiPhase.Functions.symmetricOrientation(
              m),
          freqHz=fill(f, m))
                  annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-10})));
        Modelica.Electrical.MultiPhase.Basic.Delta delta(final m=m)
                                                         annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-92,30})));
        Modelica.Electrical.MultiPhase.Sensors.VoltageSensor suppylVoltageSensor(final m=m)
          annotation (Placement(transformation(
              extent={{-10,10},{10,-10}},
              rotation=270,
              origin={-70,30})));
        ACDC.FullBridge2mPulse
                        b2mC(final m=m)
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltagesensor
          annotation(Placement(visible = true, transformation(origin={50,10},
            extent={{10,-10},{-10,10}},                                                                                                    rotation=90)));
        Modelica.Blocks.Math.Mean meanVoltage(f=2*m*f)
          annotation (Placement(
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
        Modelica.Blocks.Math.Mean meanCurrent(f=2*m*f)
          annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={80,-60})));
      equation
        connect(sineVoltage.plug_n, star.plug_p) annotation (Line(
            points={{-80,-20},{-80,-30}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(star.pin_n, ground.p) annotation (Line(
            points={{-80,-50},{-80,-60}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(meanCurrent.u,currentSensor. i) annotation (Line(
            points={{68,-60},{0,-60},{0,-50}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,meanVoltage. u) annotation (Line(
            points={{60,10},{64,10},{64,40},{68,40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(voltagesensor.v,rootMeanSquareVoltage. u) annotation (Line(
            points={{60,10},{68,10}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sineVoltage.plug_p, b2mC.ac)
                                            annotation (Line(
            points={{-80,0},{-60,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2mC.dc_n, currentSensor.n)
                                           annotation (Line(
            points={{-39.8,-6},{-20,-6},{-20,-40},{-10,-40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(b2mC.dc_p, voltagesensor.p)
                                           annotation (Line(
            points={{-40,6},{-20,6},{-20,40},{50,40},{50,20}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(currentSensor.p, voltagesensor.n) annotation (Line(
            points={{10,-40},{50,-40},{50,0}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_p, sineVoltage.plug_p) annotation (Line(
            points={{-92,20},{-92,10},{-80,10},{-80,4.44089e-16}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_n, suppylVoltageSensor.plug_p) annotation (Line(
            points={{-92,40},{-70,40}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(delta.plug_p, suppylVoltageSensor.plug_n) annotation (Line(
            points={{-92,20},{-70,20}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}}), graphics),
          experiment(StopTime=0.04, __Dymola_NumberOfIntervals=5000),
          __Dymola_experimentSetupOutput,
          Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
              graphics));
      end FullBridge2mPulse;
    end ExampleTemplates;
  end Examples;

  package Control "Control components for rectifiers"
    extends Modelica.Icons.Package;
    model TwoPulse "Boolean impulses for two pulse rectifiers"
      extends Modelica.Electrical.Analog.Interfaces.TwoPin;
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Blocks.Sources.Constant constantalpha(k=alpha) if not useSignal
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,90})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(extent={{-10,30},{10,50}})));
      Modelica.Blocks.Logical.GreaterThreshold positiveThreshold
                                                                annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,0})));
      Modelica.Blocks.Logical.LessThreshold negativeThreshold
                                                          annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,0})));
      Modelica.Blocks.Logical.Timer timerPositive
                                                 annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-30})));
      Modelica.Blocks.Logical.Timer timerNegative
                                              annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-30})));
      Modelica.Blocks.Logical.Greater      greaterPositive
                                                        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-70})));
      Modelica.Blocks.Logical.Greater      negativeEqual annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-70})));
      Modelica.Blocks.Interfaces.BooleanOutput yPositive annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-100})));
      Modelica.Blocks.Interfaces.BooleanOutput yNegative
                                                     annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-100})));
      Modelica.Blocks.Math.Gain gain(k=1/2/pi/f) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,20})));
      Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi,
            alphaMax), final uMin=0)            annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,50})));
    equation
      connect(p, voltageSensor.p) annotation (Line(
          points={{-100,4.44089e-16},{-100,0},{-102,0},{-102,0},{-60,0},{-60,40},{-10,
              40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, n) annotation (Line(
          points={{10,40},{100,40},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.v, positiveThreshold.u)
                                                   annotation (Line(
          points={{4.44089e-16,30},{-40,30},{-40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeThreshold.u, voltageSensor.v)
                                                annotation (Line(
          points={{40,12},{40,30},{4.44089e-16,30}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(positiveThreshold.y, timerPositive.u)
                                                  annotation (Line(
          points={{-40,-11},{-40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeThreshold.y, timerNegative.u)
                                            annotation (Line(
          points={{40,-11},{40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(timerPositive.y, greaterPositive.u1) annotation (Line(
          points={{-40,-41},{-40,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeEqual.u1, timerNegative.y) annotation (Line(
          points={{40,-58},{40,-41}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterPositive.y, yPositive) annotation (Line(
          points={{-40,-81},{-40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeEqual.y, yNegative)
                                      annotation (Line(
          points={{40,-81},{40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(gain.y, greaterPositive.u2) annotation (Line(
          points={{-80,9},{-80,-50},{-48,-50},{-48,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(gain.y, negativeEqual.u2) annotation (Line(
          points={{-80,9},{-80,-50},{32,-50},{32,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, gain.u) annotation (Line(
          points={{-80,39},{-80,32}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(u, limiter.u) annotation (Line(
          points={{0,100},{0,70},{-80,70},{-80,62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(constantalpha.y, limiter.u) annotation (Line(
          points={{-80,79},{-80,62}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,-60}},
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
    end TwoPulse;

    model SixPulse "Boolean impulses for six pulse rectifiers"
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Blocks.Sources.Constant constantalpha(final k=alpha) if
                                                                 not useSignal
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,90})));
      Modelica.Blocks.Logical.GreaterThreshold positiveThreshold[3](threshold=
            zeros(3)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,0})));
      Modelica.Blocks.Logical.LessThreshold negativeThreshold[3](threshold=
            zeros(3)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,0})));
      Modelica.Blocks.Logical.Timer timerPositive[3] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-30})));
      Modelica.Blocks.Logical.Timer timerNegative[3] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-30})));
      Modelica.Blocks.Logical.Greater greaterPositive[3] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-70})));
      Modelica.Blocks.Logical.Greater negativeEqual[3] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-70})));
      Modelica.Blocks.Interfaces.BooleanOutput yPositive[3] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-100})));
      Modelica.Blocks.Interfaces.BooleanOutput yNegative[3] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-100})));
      Modelica.Blocks.Math.Gain gain(final k=1/2/pi/f)
                                                 annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,20})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(m=3)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Blocks.Routing.Replicator replicator(final nout=3)
                                                            annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,-30})));
      Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=3)
                                                             annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,40})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=3)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-20,40})));
      Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi,
            alphaMax), final uMin=0)            annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,50})));
    equation
      connect(positiveThreshold.y, timerPositive.u)
                                                  annotation (Line(
          points={{-40,-11},{-40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeThreshold.y, timerNegative.u)
                                            annotation (Line(
          points={{40,-11},{40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(timerPositive.y, greaterPositive.u1) annotation (Line(
          points={{-40,-41},{-40,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeEqual.u1, timerNegative.y) annotation (Line(
          points={{40,-58},{40,-41}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterPositive.y, yPositive) annotation (Line(
          points={{-40,-81},{-40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeEqual.y, yNegative)
                                      annotation (Line(
          points={{40,-81},{40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(gain.y, replicator.u) annotation (Line(
          points={{-80,9},{-80,-18}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, greaterPositive.u2) annotation (Line(
          points={{-80,-41},{-80,-48},{-48,-48},{-48,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, negativeEqual.u2) annotation (Line(
          points={{-80,-41},{-80,-48},{32,-48},{32,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ac, deltaP.plug_n) annotation (Line(
          points={{-100,4.44089e-16},{-60,4.44089e-16},{-60,30},{-50,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
          points={{-50,50},{-20,50}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaP.plug_n, voltageSensorP.plug_p) annotation (Line(
          points={{-50,30},{-20,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.v, positiveThreshold.u) annotation (Line(
          points={{-9,40},{0,40},{0,20},{-40,20},{-40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensorP.v, negativeThreshold.u) annotation (Line(
          points={{-9,40},{0,40},{0,20},{40,20},{40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, gain.u) annotation (Line(
          points={{-80,39},{-80,32}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(u, limiter.u) annotation (Line(
          points={{8.88178e-16,100},{0,100},{0,70},{-80,70},{-80,62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(constantalpha.y, limiter.u) annotation (Line(
          points={{-80,79},{-80,62}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,-60}},
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
              textString="6")}));
    end SixPulse;

    block Pulse2m "Boolean impulses for 2*m pulse rectifiers"
      import Modelica.Constants.pi;
      parameter Integer m(final min=1)=3 "Number of phases";
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Blocks.Sources.Constant constantalpha(final k=alpha) if not useSignal
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-30,90})));
      Modelica.Blocks.Logical.GreaterThreshold positiveThreshold[m](threshold=zeros(m)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,0})));
      Modelica.Blocks.Logical.LessThreshold negativeThreshold[m](threshold=zeros(m)) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,0})));
      Modelica.Blocks.Logical.Timer timerPositive[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-30})));
      Modelica.Blocks.Logical.Timer timerNegative[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-30})));
      Modelica.Blocks.Logical.Greater greaterPositive[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-70})));
      Modelica.Blocks.Logical.Greater negativeEqual[m] annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-70})));
      Modelica.Blocks.Interfaces.BooleanOutput yPositive[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-40,-100})));
      Modelica.Blocks.Interfaces.BooleanOutput yNegative[m] annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={40,-100})));
      Modelica.Blocks.Math.Gain gain(final k=1/2/pi/f)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,0})));
      Modelica.Blocks.Routing.Replicator replicator(final nout=m)
        annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
      Modelica.Blocks.Nonlinear.Limiter limiter(final uMax=max(Modelica.Constants.pi, alphaMax), final uMin=0)
        annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,50})));
      Modelica.Blocks.Interfaces.RealInput v[m]
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-100,0})));
    equation
      connect(positiveThreshold.y, timerPositive.u)
                                                  annotation (Line(
          points={{-40,-11},{-40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeThreshold.y, timerNegative.u)
                                            annotation (Line(
          points={{40,-11},{40,-18}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(timerPositive.y, greaterPositive.u1) annotation (Line(
          points={{-40,-41},{-40,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(negativeEqual.u1, timerNegative.y) annotation (Line(
          points={{40,-58},{40,-41}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(greaterPositive.y, yPositive) annotation (Line(
          points={{-40,-81},{-40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(negativeEqual.y, yNegative)
                                      annotation (Line(
          points={{40,-81},{40,-100}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(gain.y, replicator.u) annotation (Line(
          points={{-2.22045e-015,-11},{-2.22045e-015,-18},{2.22045e-015,-18}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, greaterPositive.u2) annotation (Line(
          points={{-2.22045e-015,-41},{-2.22045e-015,-48},{-48,-48},{-48,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(replicator.y, negativeEqual.u2) annotation (Line(
          points={{-2.22045e-015,-41},{-2.22045e-015,-48},{32,-48},{32,-58}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(limiter.y, gain.u) annotation (Line(
          points={{-1.9984e-015,39},{-1.9984e-015,12},{2.22045e-015,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(u, limiter.u) annotation (Line(
          points={{8.88178e-016,100},{0,100},{0,70},{2.22045e-015,70},{2.22045e-015,
              62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(constantalpha.y, limiter.u) annotation (Line(
          points={{-30,79},{-30,70},{0,70},{0,62},{2.22045e-015,62}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(v, positiveThreshold.u) annotation (Line(
          points={{-100,0},{-60,0},{-60,20},{-40,20},{-40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(v, negativeThreshold.u) annotation (Line(
          points={{-100,0},{-60,0},{-60,20},{40,20},{40,12}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
            Line(
              points={{-40,-20},{-40,-24},{-20,-24},{-20,-40},{-40,-40},{-40,-60}},
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
    end Pulse2m;
  end Control;

  package ACDC "AC to DC and DC to AC converter"
    extends Modelica.Icons.Package;
    model DiodeBridge2PulseCenterTap
      "Two pulse diode rectifier with center tap"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
        final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,60},
            extent={{-10,10},{10,-10}},
            rotation=0)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,-60},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    equation
      if not useHeatPort then
        LossPower = idealDiodeP.LossPower + idealDiodeN.LossPower;
      end if;
      connect(ac_p, idealDiodeP.p) annotation (Line(
          points={{-100,60},{0,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealDiodeN.p) annotation (Line(
          points={{-100,-60},{0,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP.heatPort, heatPort) annotation (Line(
          points={{10,70},{30,70},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
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
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="M2C")}),                                                                                                    Diagram(coordinateSystem(extent={{-100,
                -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
            graphics),                                                                                                    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 0.000001));
    end DiodeBridge2PulseCenterTap;

    model FullBridge2PulseCenterTap "Two pulse full bridge SCR with center tap"
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
        final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort)
        annotation(Placement(visible = true, transformation(origin={10,60},
          extent={{-10,10},{10,-10}},                                                                                                    rotation = 0)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN(
        final Ron=Ron,
        final Goff=Goff,
        final Vknee=Vknee,
        final useHeatPort=useHeatPort)
        annotation(Placement(visible = true, transformation(origin={10,-60},
          extent={{-10,-10},{10,10}},                                                                                                    rotation = 0)));
      Control.Pulse2m  twoPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=1)
        annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-10,0})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-60,20})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP.LossPower + idealThyristorN.LossPower;
      end if;
      connect(twoPulse.u,u)  annotation (Line(
          points={{-20,1.11022e-15},{-40,1.11022e-15},{-40,80},{1.11022e-15,80},
              {1.11022e-15,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ac_p,idealThyristorP. p) annotation (Line(
          points={{-100,60},{-4.44089e-16,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n,idealThyristorN. p) annotation (Line(
          points={{-100,-60},{-4.44089e-16,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.n, dc_p) annotation (Line(
          points={{20,60},{100,60},{100,0}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.n, dc_p) annotation (Line(
          points={{20,-60},{100,-60},{100,4.44089e-16}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.heatPort, heatPort) annotation (Line(
          points={{10,-70},{10,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, heatPort) annotation (Line(
          points={{10,70},{10,80},{30,80},{30,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, voltageSensor.p) annotation (Line(
          points={{-100,60},{-60,60},{-60,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, ac_n) annotation (Line(
          points={{-60,10},{-60,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
          points={{-50,20},{-10,20},{-10,10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twoPulse.yPositive[1], idealThyristorP.fire) annotation (Line(
          points={{1.33227e-15,4},{17,4},{17,49}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yNegative[1], idealThyristorN.fire) annotation (Line(
          points={{0,-4},{17,-4},{17,-49}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
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
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="M2C")}),                                                                                                    Diagram(coordinateSystem(extent={{-100,
                -100},{100,100}},                                                                                                    preserveAspectRatio=false,  initialScale = 0.1, grid = {2,2}),
            graphics),                                                                                                    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 0.000001));
    end FullBridge2PulseCenterTap;

    model DiodeBridge2Pulse "Two pulse Graetz diode rectifier"
      import Modelica.Constants.pi;
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode idealDiodeN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
    equation
      if not useHeatPort then
        LossPower = idealDiodeP1.LossPower + idealDiodeP2.LossPower + idealDiodeN1.LossPower + idealDiodeN2.LossPower;
      end if;
      connect(idealDiodeP2.n, idealDiodeP1.n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, idealDiodeN2.p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN2.n, idealDiodeP2.p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP1.p, idealDiodeN1.n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeN2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac_p, idealDiodeP1.p) annotation (Line(
          points={{-100,60},{-60,60},{-60,20},{10,20},{10,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(ac_n, idealDiodeP2.p) annotation (Line(
          points={{-100,-60},{-60,-60},{-60,-20},{40,-20},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end DiodeBridge2Pulse;

    model HalfBridge2Pulse "Two pulse Graetz half bridge SCR"
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode     idealThyristorN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealDiode     idealThyristorN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Control.Pulse2m  twoPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=1)
       annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-30,0})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-80,20})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP1.LossPower + idealThyristorP2.LossPower + idealThyristorN1.LossPower + idealThyristorN2.LossPower;
      end if;
      connect(idealThyristorP2.n,idealThyristorP1. n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p,idealThyristorN2. p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yPositive[1],idealThyristorP1. fire) annotation (Line(
          points={{-20,4},{26,4},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yNegative[1],idealThyristorP2. fire) annotation (Line(
          points={{-20,-4},{56,-4},{56,57},{51,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN2.n,idealThyristorP2. p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.p,idealThyristorN1. n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(twoPulse.u, u) annotation (Line(
          points={{-40,1.33227e-15},{-60,1.33227e-15},{-60,70},{0,70},{0,100},{
              8.88178e-16,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(idealThyristorN1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorN2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
          points={{-70,20},{-30,20},{-30,10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.p, ac_p) annotation (Line(
          points={{-80,30},{-80,60},{-100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, ac_n) annotation (Line(
          points={{-80,10},{-80,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end HalfBridge2Pulse;

    model FullBridge2Pulse "Two pulse Graetz full bridge SCR"
      import Modelica.Constants.pi;
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorP2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,50},
            extent={{-10,10},{10,-10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN1(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={10,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Ideal.IdealThyristor idealThyristorN2(
        Ron=Ron,
        Goff=Goff,
        Vknee=Vknee,
        useHeatPort=useHeatPort) annotation (Placement(visible=true, transformation(
            origin={40,-50},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Control.Pulse2m  twoPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=1)
       annotation (Placement(
            transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-30,0})));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p
        annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-80,20})));
    equation
      if not useHeatPort then
        LossPower = idealThyristorP1.LossPower + idealThyristorP2.LossPower + idealThyristorN1.LossPower + idealThyristorN2.LossPower;
      end if;
      connect(idealThyristorP2.n,idealThyristorP1. n) annotation (Line(
          points={{40,60},{10,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p,idealThyristorN2. p) annotation (Line(
          points={{10,-60},{40,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yPositive[1],idealThyristorP1. fire) annotation (Line(
          points={{-20,4},{26,4},{26,57},{21,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yPositive[1],idealThyristorN2. fire) annotation (Line(
          points={{-20,4},{26,4},{26,-42},{30,-42},{30,-43},{29,-43}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twoPulse.yNegative[1],idealThyristorP2. fire) annotation (Line(
          points={{-20,-4},{56,-4},{56,57},{51,57}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.fire, twoPulse.yNegative[1]) annotation (Line(
          points={{-1,-43},{-4,-43},{-4,-4},{-20,-4}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN2.n,idealThyristorP2. p) annotation (Line(
          points={{40,-40},{40,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.p,idealThyristorN1. n) annotation (Line(
          points={{10,40},{10,-40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP1.n, dc_p) annotation (Line(
          points={{10,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN1.p, dc_n) annotation (Line(
          points={{10,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(twoPulse.u, u) annotation (Line(
          points={{-40,1.33227e-15},{-60,1.33227e-15},{-60,70},{0,70},{0,100},{
              8.88178e-16,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(idealThyristorN1.heatPort, heatPort) annotation (Line(
          points={{20,-50},{20,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorN2.heatPort, heatPort) annotation (Line(
          points={{50,-50},{50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP1.heatPort, heatPort) annotation (Line(
          points={{0,50},{-50,50},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP2.heatPort, heatPort) annotation (Line(
          points={{30,50},{30,30},{-50,30},{-50,-100},{4.44089e-16,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(voltageSensor.v, twoPulse.v[1]) annotation (Line(
          points={{-70,20},{-30,20},{-30,10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(voltageSensor.p, ac_p) annotation (Line(
          points={{-80,30},{-80,60},{-100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensor.n, ac_n) annotation (Line(
          points={{-80,10},{-80,-60},{-100,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-40,10},{40,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2C")}));
    end FullBridge2Pulse;

    model DiodeBridge2mPulse "2*m pulse diode bridge rectifier"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeP(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort) annotation (Placement(visible=true,
            transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
    equation
      if not useHeatPort then
        LossPower = sum(idealDiodeP.idealDiode.LossPower) + sum(idealDiodeN.idealDiode.LossPower);
      end if;
      connect(ac, idealDiodeP.plug_p) annotation (Line(
          points={{-100,4.44089e-16},{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.plug_p, idealDiodeN.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.plug_p, starN.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealDiodeN.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealDiodeP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end DiodeBridge2mPulse;

    model HalfBridge2mPulse "2*m pulse half bridge SCR"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorP(
        final m=m,
        final Ron=fill(Ron,m),
        final Goff=fill(Goff,m),
        final Vknee=fill(Vknee,m),
        each final useHeatPort=useHeatPort)
           annotation (Placement(visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealDiode idealDiodeN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort)
        annotation (Placement(visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Control.Pulse2m   twomPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=m)         annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-50,-20})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
      Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=m)
                                                             annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-50,40})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=m)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={-50,20})));
    equation
      if not useHeatPort then
        LossPower = sum(idealThyristorP.idealThyristor.LossPower) + sum(idealDiodeN.idealDiode.LossPower);
      end if;
      connect(twomPulse.u, u) annotation (Line(
          points={{-60,-20},{-80,-20},{-80,80},{0,80},{0,100},{8.88178e-16,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(ac,idealThyristorP. plug_p) annotation (Line(
          points={{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_p, idealDiodeN.plug_n)     annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealDiodeN.plug_p, starN.plug_p)     annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealDiodeN.heatPort)     annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac, voltageSensorP.plug_p) annotation (Line(
          points={{-100,0},{-60,0},{-60,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.plug_p, deltaP.plug_n) annotation (Line(
          points={{-60,20},{-60,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
          points={{-40,40},{-40,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.v, twomPulse.v) annotation (Line(
          points={{-50,9},{-50,-10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twomPulse.yPositive, idealThyristorP.fire) annotation (Line(
          points={{-40,-16},{-30,-16},{-30,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end HalfBridge2mPulse;

    model FullBridge2mPulse "2*m pulse full bridge SCR"
      import Modelica.Constants.pi;
      parameter Integer m(final min=3)=3 "Number of phases";
      parameter Boolean useSignal = false
        "Enables signal input instead of parameter";
      parameter Modelica.SIunits.Frequency f = 50 "Frequency";
      parameter Modelica.SIunits.Angle alpha = 0 "Firing angle"
        annotation (Dialog(enable=not useSignal));
      parameter Modelica.SIunits.Angle alphaMax(min=0,max=Modelica.Constants.pi) = Modelica.Constants.pi
        "Maximum firing angle";
      parameter Modelica.SIunits.Resistance Ron(final min=0) = 1.E-5
        "Closed thyristor resistance";
      parameter Modelica.SIunits.Conductance Goff(final min=0) = 1.E-5
        "Opened thyristor conductance";
      parameter Modelica.SIunits.Voltage Vknee(final min=0) = 0
        "Forward threshold voltage";
      extends Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort(
         final T=293.15);
      Modelica.Blocks.Interfaces.RealInput u if useSignal
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,100})));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorP(
        final m=m,
        final Ron=fill(Ron,m),
        final Goff=fill(Goff,m),
        final Vknee=fill(Vknee,m),
        each final useHeatPort=useHeatPort)
           annotation (Placement(visible=true, transformation(
            origin={10,40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Modelica.Electrical.MultiPhase.Ideal.IdealThyristor idealThyristorN(
        final m=m,
        final Ron=fill(Ron, m),
        final Goff=fill(Goff, m),
        final Vknee=fill(Vknee, m),
        each final useHeatPort=useHeatPort)
        annotation (Placement(visible=true, transformation(
            origin={10,-40},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Control.Pulse2m   twomPulse(
        final useSignal=useSignal,
        final f=f,
        final alpha=alpha,
        final alphaMax=alphaMax,
        final m=m)         annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-50,-20})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p
        annotation (Placement(transformation(extent={{90,50},{110,70}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n
        annotation (Placement(transformation(extent={{92,-70},{112,-50}})));
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug ac(final m=m)
        annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      Modelica.Electrical.MultiPhase.Basic.Star starP(final m=m)
        annotation (Placement(transformation(extent={{20,50},{40,70}})));
      Modelica.Electrical.MultiPhase.Basic.Star starN(final m=m)
        annotation (Placement(transformation(extent={{20,-70},{40,-50}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalCollector thermalCollector(final m=m) if
                                                                                    useHeatPort
        annotation (Placement(transformation(extent={{50,-100},{70,-80}})));
      Modelica.Electrical.MultiPhase.Basic.Delta deltaP(final m=m)
                                                             annotation (
          Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-50,40})));
      Modelica.Electrical.MultiPhase.Sensors.VoltageSensor voltageSensorP(final m=m)
        annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={-50,20})));
    equation
      if not useHeatPort then
        LossPower = sum(idealThyristorP.idealThyristor.LossPower) + sum(idealThyristorN.idealThyristor.LossPower);
      end if;
      connect(idealThyristorN.fire, twomPulse.yNegative)
                                                        annotation (Line(
          points={{-1,-33},{-28,-33},{-28,-24},{-40,-24}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(twomPulse.u, u)
                             annotation (Line(
          points={{-60,-20},{-80,-20},{-80,80},{0,80},{0,100},{8.88178e-16,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(twomPulse.yPositive, idealThyristorP.fire)
                                                        annotation (Line(
          points={{-40,-16},{-28,-16},{-28,47},{-1,47}},
          color={255,0,255},
          smooth=Smooth.None));
      connect(ac,idealThyristorP. plug_p) annotation (Line(
          points={{-100,0},{10,0},{10,30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_p, idealThyristorN.plug_n) annotation (Line(
          points={{10,30},{10,-30}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorP.plug_n, starP.plug_p) annotation (Line(
          points={{10,50},{10,60},{20,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starP.pin_n, dc_p) annotation (Line(
          points={{40,60},{100,60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(idealThyristorN.plug_p, starN.plug_p) annotation (Line(
          points={{10,-50},{10,-60},{20,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(starN.pin_n, dc_n) annotation (Line(
          points={{40,-60},{102,-60}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(heatPort, thermalCollector.port_b) annotation (Line(
          points={{4.44089e-16,-100},{60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(thermalCollector.port_a, idealThyristorN.heatPort) annotation (Line(
          points={{60,-80},{60,-40},{20,-40}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(idealThyristorP.heatPort, thermalCollector.port_a) annotation (Line(
          points={{20,40},{60,40},{60,-80}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ac, voltageSensorP.plug_p) annotation (Line(
          points={{-100,0},{-60,0},{-60,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.plug_p, deltaP.plug_n) annotation (Line(
          points={{-60,20},{-60,40}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(deltaP.plug_p, voltageSensorP.plug_n) annotation (Line(
          points={{-40,40},{-40,20}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(voltageSensorP.v, twomPulse.v) annotation (Line(
          points={{-50,9},{-50,-10}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
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
              textString="DC"),             Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            lineColor={0,0,255}),
            Text(
              extent={{-90,10},{90,-10}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              textString="B2*%m%C")}));
    end FullBridge2mPulse;
  end ACDC;

  package Icons
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
  annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}),
        graphics),                                                                                                    Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})),
    uses(Modelica(version="3.2.2")));
end Modelica_Electrical_PowerConverters;
