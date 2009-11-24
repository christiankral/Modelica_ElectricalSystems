within ;
package Modelica_FundamentalWave "Magnetic library for fundamental wave effects in electric machines"


  package UsersGuide "Users Guide" 
  
    annotation (DocumentationClass=true, Documentation(info="<html>

<p>
This library contains components for modelling of electromagnetic fundamental wave models for the application in 
<a href=Modelica_FundamentalWave.Machines>electric machines</a>. This library is an alternative approach to the <a href=\"Modelica.Electrical.Machines\">Modelica.Electrical.Machines</a> library. A great advantage of this library is the strict object orientation of the electrical and magnetic components that the electric machines models are composed of. From a didactic point of view this library is very beneficial for students in the field of electrical engineering.
</p>

<p>
For more details see the <a href=Modelica_FundamentalWave.UsersGuide.Concept>concept.</a>
</html>"));
  
    class Concept "Fundamental wave concept" 
    
      annotation (Documentation(info="<html>

<h4>Overview of the concept of fundamental waves</h4>

<p>
In the fundamental wave theory only a pure sinusoidal distribution of magnetic quantities is assumed. It is thus assumed that all other harmonic wave effects are not taken into account. The potential and flow quantities of this library are the complex magnetic potential difference and the complex magnetic flux as defined in the basic <a href=\"Modelica_FundamentalWave.Interfaces.MagneticPort\">magnetic port</a>. Due to the sinusoidal distribution of magnetic potential and flux, such a complex phasor representation can be used. This way, the FundamentalWave library can be seen as a spatial extension of the <a href=\"Modelica.Magnetics.FluxTubes\">FluxTubes</a> library.
</p>

<p>
The specific arrangement of windings in electric machines with <img src=\"../Images/p.png\"> pole pairs give rise to sinusoidal dominant magnetic potential wave. The spatial period of this wave is determined by one pole pair 
[<a href=\"Modelica_FundamentalWave.UsersGuide.References\">Mueller70</a>, 
 <a href=\"Modelica_FundamentalWave.UsersGuide.References\">Spaeth73</a>]. 
</p>

<p>
The main components of an electric machine model based on the FundamentalWave library are <a href=\"Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">multi phase</a> and <a href=\"Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding\">single phase windings</a>, <a href=\"Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap\">air gap</a> as well as <a href=\"Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">symmetric</a> or <a href=\"Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding\">salient cage</a> models. 
The electric machine models provided in this library are based on symmetrical three phase windings in the stator and equivalent two or three phase windings in the rotor. 
</p>

<h4>Assumptions</h4>

<p> 
The machine models of the FundamentalWave library are currently based on the following assumptions
</p>

<ul>
<li>The number of stator phases is limited to three
    [<a href=\"Modelica_FundamentalWave.UsersGuide.References\">Eckhardt82</a>] 
    </li>
<li>The phase windings are assumed to be symmetrical; an extension to this approach can be considererd</li>
<li>Only fundamental wave effects are taken into account</li>
<li>There are no restrictions on the waveforms of voltages and currents</li>
<li>All resistances and inductances are modeled as constant quantities; saturation effects, cross coupling effects 
    [<a href=\"Modelica_FundamentalWave.UsersGuide.References\">Li07</a>], temperature dependency of resistances and deep bar effects could be considered in an extension to this library</li>
<li>Core losses, i.e. eddy current and hysteresis losses, friction losses and stray load losses are currently not considered [<a href=\"Modelica_FundamentalWave.UsersGuide.References\">Haumer09</a>]</li>
<li>The only dissipated losses in the electric machine models are currently ohmic heat losses</li>
</ul>
 
</html>
"));
    equation 
    
    end Concept;
  
    class References "References" 
    
      annotation (Documentation(info="<html>
<h4>References</h4>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
    <tr>
      <td valign=\"top\">[Eckhardt82]</td> 
      <td valign=\"top\">H. Eckhardt,
        <i>Grundz&uuml;ge der elektrischen Maschinen</i> (in German), 
        B. G. Teubner Verlag, Stuttgart, 1982.</td>
    </tr>

    <tr>
      <td valign=\"top\">[Haumer09]</td> 
      <td valign=\"top\">A. Haumer, and C. Kral,
        &quot;<i><a href=\"http://www.modelica.org/events/modelica2009/Proceedings/memorystick/pages/papers/0103/0103.pdf\">The 
        AdvancedMachines Library: Loss Models for Electric Machines</<a></i>,&quot;
        Modelica Conference, 2009.</td>
    </tr>

    <tr>
      <td valign=\"top\">[Li07]</td> 
      <td valign=\"top\">Y. Li, Z. Q. Zhu, D. Howe, and C. M. Bingham,
        &quot;Modeling of Cross-Coupling Magnetic Saturation in Signal-Injection-Based
        Sensorless Control of Permanent-Magnet Brushless AC Motors,&quot;
        <i>IEEE Transactions on Magnetics</i>, 
        vol. 43, no. 6, pp. 2552-2554, June 2007.</td>
    </tr>

    <tr>
      <td valign=\"top\">[Mueller70]</td> 
      <td valign=\"top\">G, M&uuml;ller,
        <i>Elektrische Maschinen -- Grundlagen, Aufbau und Wirkungsweise</i> (in German), 
        VEB Verlag Technik Berlin, 4th edition, 1970.</td>
    </tr>

    <tr>
      <td valign=\"top\">[Spaeth73]</td> 
      <td valign=\"top\">H. Sp&auml;th,
        <i>Elektrische Maschinen -- Eine Einf�hrung in die Theorie des Betriebsverhaltens</i> (in German), 
        Springer-Verlag, Berlin, Heidelberg, New York, 1973.</td>
    </tr>


</table>
</html>
"));
    
    end References;
  
    class Contact "Contact" 
    
      annotation (Documentation(info="<html>
<h4>Contact</h4>
 
<p>
Christian Kral<br>
<a href=\"http://www.ait.ac.at\">Austrian Institute of Technology, AIT</a><br>
Mobility Department<br>
Giefinggasse 2, 1210 Vienna, Austria<br>
email: <a HREF=\"mailto:christian.kral@ait.ac.at\">christian.kral@ait.ac.at</a><br></dd>
</p>

<p>
Anton Haumer<br>
<a href=\"http://www.haumer.at\">Technical Consulting &amp; Electrical Engineering</a><br>
3423 St. Andrae-Woerdern, Austria<br>
email: <a HREF=\"mailto:a.haumer@haumer.at\">a.haumer@haumer.at</a><br></dd>
</p>

<h4>Acknowledgements</h4>

<p>
Based on an original idea of Michael Beuschel this library was revised and modified. The authors of the FundamentalWave library would like to thanks Michael Beuschel for contributing his source code to this library.
</p>
</html>
"));
    end Contact;
  
  end UsersGuide;


  package Examples 
  "Examples of electric machines based on the FundamentalWave concept" 
  
    model AIMC_DOL 
    "Asynchronous induction machine with squirrel cage direct-on-line" 
      extends Modelica.Icons.Example;
    
      constant Integer m=3 "number of phases";
      parameter Modelica.SIunits.Voltage VsNominal=100 
      "nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fsNominal=50 "nominal frequency";
      parameter Modelica.SIunits.Time tStart1=0.1 "start time";
      parameter Modelica.SIunits.Torque T_Load=161.4 "nominal load torque";
      parameter Modelica.SIunits.Conversions.NonSIunits.AngularVelocity_rpm 
      rpmLoad=
          1440.45 "nominal load speed";
      parameter Modelica.SIunits.Inertia J_Load=0.29 "load's moment of inertia";
    
      parameter Integer p = 2 "Number of pole pairs";
      parameter Modelica.SIunits.Resistance Rs=0.03 
      "Stator resistance per phase";
      parameter Modelica.SIunits.Inductance Lssigma=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "Stator stray inductance per phase";
      parameter Modelica.SIunits.Inductance Lm=3*sqrt(1 - 0.0667)/(2*Modelica.Constants.pi*fsNominal) 
      "Main field inductance";
      parameter Modelica.SIunits.Inductance Lrsigma=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "Rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rr=0.04 
      "Rotor resistance (equivalent three phase winding)";
    
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (extent=[-100, 80; -80, 100], rotation=-90);
      Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) 
        annotation (extent=[-50, 80; -70, 100]);
      Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
        final m=m,
        V=fill(sqrt(2/3)*VsNominal, m),
        freqHz=fill(fsNominal, m)) 
        annotation (extent=[10,70; -10,50],    rotation=-90);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorM 
        annotation (extent=[10,-10; -10,10],rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxM(
        StarDelta="D") 
        annotation (extent=[-10,-30; 10,-10]);
      Modelica_FundamentalWave.Machines.AsynchronousInductionMachines.AIM_SquirrelCage
      aimcM(
      p=p,
      Rs=Rs,
      Lssigma=Lssigma,
      Lm=Lm,
      Lrsigma=Lrsigma,
      Rr=Rr) 
        annotation (extent=[-10,-50; 10,-30],    rotation=0);
      Modelica.Mechanics.Rotational.Inertia loadInertiaM(J=J_Load) 
        annotation (extent=[40,-50; 60,-30]);
      Modelica.Mechanics.Rotational.QuadraticSpeedDependentTorque 
      quadraticLoadTorqueM(
        w_nominal=Modelica.SIunits.Conversions.from_rpm(rpmLoad),
        tau_nominal=-T_Load,
        TorqueDirection=false) 
        annotation (extent=[90,-50; 70,-30]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorE 
        annotation (extent=[-30,-10; -10,10],
        rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxE(
        StarDelta="D") 
        annotation (extent=[-10,-70; 10,-50]);
      Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage
      aimcE(
        p=p,
        Rs=Rs,
        Lssigma=Lssigma,
        Lm=Lm,
        Lrsigma=Lrsigma,
        Rr=Rr) 
        annotation (extent=[-10,-90; 10,-70],    rotation=0);
      Modelica.Mechanics.Rotational.Inertia loadInertiaE(J=J_Load) 
        annotation (extent=[40,-90; 60,-70]);
      Modelica.Mechanics.Rotational.QuadraticSpeedDependentTorque 
      quadraticLoadTorqueE(
        w_nominal=Modelica.SIunits.Conversions.from_rpm(rpmLoad),
        tau_nominal=-T_Load,
        TorqueDirection=false) 
        annotation (extent=[90,-90; 70,-70]);
    equation 
      connect(star.pin_n, ground.p) 
        annotation (points=[-70, 90; -80, 90], style(color=3));
      connect(sineVoltage.plug_n, star.plug_p) 
        annotation (points=[-1.22293e-15,70; -1.22293e-15,90; -50,90],
                                                        style(color=3));
      connect(aimcM.flange_a, loadInertiaM.flange_a) 
        annotation (points=[10,-40; 40,-40],    style(color=0, rgbcolor={0,0,0}));
      connect(loadInertiaM.flange_b, quadraticLoadTorqueM.flange) 
        annotation (points=[60,-40; 70,-40], style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxM.negativeMachinePlug, aimcM.plug_sn) 
        annotation (
          points=[-6,-30; -6,-30],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxM.positiveMachinePlug, aimcM.plug_sp) 
        annotation (
          points=[6,-30; 6,-30],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxM.plugToGrid, currentRMSsensorM.plug_n) 
        annotation (
          points=[6.10623e-16,-28; -1.22293e-15,-28; -1.22293e-15,-10],
          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(sineVoltage.plug_p, currentRMSsensorM.plug_p) 
        annotation (points=[1.68051e-18,50; 1.68051e-18,39.5; 3.36102e-18,39.5;
          3.36102e-18,29; 1.68051e-18,29; 1.68051e-18,10],
                                             style(color=3, rgbcolor={0,0,255}));
      connect(aimcE.flange_a, loadInertiaE.flange_a) 
        annotation (points=[10,-80; 40,-80],
                     style(color=0, rgbcolor={0,0,0}));
      connect(loadInertiaE.flange_b, quadraticLoadTorqueE.flange) 
        annotation (points=[60,-80; 70,-80], style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxE.negativeMachinePlug, aimcE.plug_sn) 
        annotation (
          points=[-6,-70; -6,-70],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxE.positiveMachinePlug, aimcE.plug_sp) 
        annotation (
          points=[6,-70; 6,-70],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(currentRMSsensorE.plug_p, currentRMSsensorM.plug_p) 
        annotation (points=[-20,10; -15,10; -15,10; -10,10; -10,10; 1.68051e-18,
          10],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=45,
          rgbfillColor={255,128,0},
          fillPattern=1));
      connect(currentRMSsensorE.plug_n,terminalBoxE. plugToGrid) 
        annotation (points=[-20,-10; -20,-60; 6.10623e-16,-60; 6.10623e-16,-68],
          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=45,
          rgbfillColor={255,128,0},
          fillPattern=1));
    
      annotation (
        Diagram,
        experiment(
          StopTime=1.5,
          Interval=0.0002,
          Tolerance=1e-05),
        experimentSetupOutput(doublePrecision=true),
        Documentation(info="<HTML>
<h4>Asynchronous induction machine with squirrel cage - direct on line (DOL) starting</h4>
<p>
At start time tStart three phase voltage is supplied to the asynchronous induction machine with squirrel cage;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed, finally reaching nominal speed.</p>

<p>
Simulate for 1.5 seconds and plot (versus time):
</p>
 
<ul>
<li>currentRMSsensorM|E.I: stator current RMS</li>
<li>aimcM|E.rpm_mechanical: motor's speed</li>
<li>aimcM|E.tau_electrical: motor's torque</li>
</ul>
</HTML>"));
    
    end AIMC_DOL;
  
    model AIMS_start "Test example 3: AsynchronousInductionMachineSlipRing" 
      extends Modelica.Icons.Example;
    
      constant Integer m=3 "Number of phases";
      parameter Modelica.SIunits.Voltage VsNominal=100 
      "Nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
      parameter Modelica.SIunits.Time tOn=0.1 "Start time of machine";
      parameter Modelica.SIunits.Resistance RStart=0.16 "starting resistance";
      parameter Modelica.SIunits.Time tRheostat=1.0 
      "Time of shortening the rheostat";
      parameter Modelica.SIunits.Torque T_Load=161.4 "Nominal load torque";
      parameter Modelica.SIunits.Conversions.NonSIunits.AngularVelocity_rpm 
      rpmLoad =                                                                       1440.45 
      "Nominal load speed";
      parameter Modelica.SIunits.Inertia J_Load=0.29 "Load's moment of inertia";
    
      parameter Integer p = 2 "Number of pole pairs";
      parameter Modelica.SIunits.Resistance Rs = 0.03 
      "Stator resistance per phase";
      parameter Modelica.SIunits.Inductance Lssigma = 3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "Stator stray inductance per phase";
      parameter Modelica.SIunits.Inductance Lm = 3*sqrt(1 - 0.0667)/(2*Modelica.Constants.pi*fsNominal) 
      "Main field inductance";
      parameter Modelica.SIunits.Inductance Lrsigma = 3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "Rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rr = 0.04 
      "Rotor resistance (equivalent three phase winding)";
    
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (extent=[-100, 80; -80, 100], rotation=-90);
      Modelica.Electrical.MultiPhase.Basic.Star star(final m=m) 
        annotation (extent=[-50, 80; -70, 100]);
      Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
        final m=m,
        V=fill(sqrt(2/3)*VsNominal, m),
        freqHz=fill(fsNominal, m)) 
        annotation (extent=[10,70; -10,50], rotation=-90);
      Modelica.Electrical.MultiPhase.Ideal.IdealClosingSwitch idealCloser(
        final m=m) 
        annotation (extent=[10,20; -10,40], rotation=-90);
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](
        each startTime=tOn) 
        annotation (extent=[-60,20; -40,40]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor 
      currentRMSsensorElectricalE 
        annotation (extent=[40,-20; 20,0],  rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox 
      terminalBoxElectrical(  StarDelta="D") 
        annotation (extent=[20,-40; 40,-20]);
      Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing
      aimsE(
        p=p,
        fNominal=fsNominal,
        Rs=Rs,
        Lssigma=Lssigma,
        Lm=Lm,
        Lrsigma=Lrsigma,
        Rr=Rr,
        TurnsRatio=0.7,
        useTurnsRatio=true) 
        annotation (extent=[20,-60; 40,-40], rotation=0);
      Modelica_FundamentalWave.MoveToModelica.SwitchedRheostat 
      rheostatElectricalE(  RStart=RStart, tStart=tRheostat) 
       annotation (extent=[0,-60; 20,-40]);
      Modelica.Mechanics.Rotational.Inertia loadInertiaElectricalE(J=J_Load) 
        annotation (extent=[50,-60; 70,-40]);
      Modelica.Mechanics.Rotational.QuadraticSpeedDependentTorque 
      quadraticLoadTorqueElectricalE(
        w_nominal=Modelica.SIunits.Conversions.from_rpm(rpmLoad),
        tau_nominal=-T_Load,
        TorqueDirection=false) 
        annotation (extent=[100,-60; 80,-40]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorM 
        annotation (extent=[-40,-20; -20,0],rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBox(
        StarDelta="D") 
        annotation (extent=[-20,-40; -40,-20]);
      Machines.AsynchronousInductionMachines.AIM_SlipRing aimsM(
        Rs=Rs,
        Lssigma=Lssigma,
        Lm=Lm,
        Lrsigma=Lrsigma,
        Rr=Rr,
        p=p,
        TurnsRatio=0.7,
      useTurnsRatio=true) 
        annotation (extent=[-20,-60; -40,-40],   rotation=0);
      Modelica_FundamentalWave.MoveToModelica.SwitchedRheostat rheostatM(
        RStart=RStart, tStart=tRheostat) 
                                    annotation (extent=[0,-60; -20,-40]);
      Modelica.Mechanics.Rotational.Inertia loadInertiaM(J=J_Load) 
        annotation (extent=[-50,-60; -70,-40]);
      Modelica.Mechanics.Rotational.QuadraticSpeedDependentTorque 
      quadraticLoadTorqueM(
        w_nominal=Modelica.SIunits.Conversions.from_rpm(rpmLoad),
        tau_nominal=-T_Load,
        TorqueDirection=false) 
        annotation (extent=[-100,-60; -80,-40]);
    equation 
      connect(star.pin_n, ground.p) 
        annotation (points=[-70, 90; -80, 90], style(color=3));
      connect(sineVoltage.plug_n, star.plug_p) 
        annotation (points=[-1.22293e-15,70; -1.22293e-15,90; -50,90],
                                                        style(color=3));
      connect(sineVoltage.plug_p, idealCloser.plug_p) 
        annotation (points=[1.68051e-18,50; 0,48; 1.22461e-15,46; 1.68051e-18,
          46; 1.68051e-18,40],                 style(color=3));
      connect(loadInertiaElectricalE.flange_b, quadraticLoadTorqueElectricalE.
        flange) 
        annotation (points=[70,-50; 80,-50], style(color=0, rgbcolor={0,0,0}));
      connect(aimsE.flange_a, loadInertiaElectricalE.flange_a) 
                                                     annotation (points=[40,-50; 50,
            -50], style(color=0, rgbcolor={0,0,0}));
      connect(booleanStep.y, idealCloser.control)   annotation (points=[-39,30;
          -7,30],   style(color=5, rgbcolor={255,0,255}));
      connect(idealCloser.plug_n, currentRMSsensorElectricalE.plug_p) 
                                                             annotation (points=[
          -1.22293e-15,20; -1.22293e-15,10; 30,10; 30,5.55112e-16],
                 style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxElectrical.negativeMachinePlug, aimsE.plug_sn) 
                                                               annotation (
          points=[24,-40; 24,-40],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxElectrical.positiveMachinePlug, aimsE.plug_sp) 
                                                               annotation (
          points=[36,-40; 36,-40], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxElectrical.plugToGrid, currentRMSsensorElectricalE.plug_n) 
                                                                 annotation (
          points=[30,-38; 30,-20],
          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(rheostatElectricalE.plug_p, aimsE.plug_rp) 
                                                       annotation (points=[20,-44;
            20,-44],  style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(rheostatElectricalE.plug_n, aimsE.plug_rn) 
                                                       annotation (points=[20,-56;
            20,-56],  style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(loadInertiaM.flange_b, quadraticLoadTorqueM.flange) 
        annotation (points=[-70,-50; -80,-50],
                                             style(color=0, rgbcolor={0,0,0}));
      connect(aimsM.flange_a, loadInertiaM.flange_a) annotation (points=[-40,-50;
            -50,-50],
                  style(color=0, rgbcolor={0,0,0}));
      connect(rheostatM.plug_n, aimsM.plug_rn)         annotation (points=[-20,-56;
            -20,-56], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(terminalBox.positiveMachinePlug, aimsM.plug_sp) 
                                                             annotation (points=[-36,-40;
            -36,-40],          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(terminalBox.negativeMachinePlug, aimsM.plug_sn) 
                                                             annotation (points=[-24,-40;
            -24,-40],          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(currentRMSsensorM.plug_n, terminalBox.plugToGrid) 
                                                               annotation (points=[-30,-20;
            -30,-38],          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(currentRMSsensorM.plug_p, idealCloser.plug_n) 
                                                           annotation (points=[-30,
          5.55112e-16; -30,10; -1.22293e-15,10; -1.22293e-15,20],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(aimsM.plug_rp, rheostatM.plug_p) 
                                             annotation (points=[-20,-44; -20,-44],
                  style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
    
      annotation (
        Diagram,
        experiment(
          StopTime=1.5,
          Interval=0.001,
          Tolerance=1e-05),
        experimentSetupOutput(doublePrecision=true),
        Documentation(info="<HTML>
<h4>Asynchronous induction machine with slipring rotor resistance starting</h4>
<p>
At start time tOn three phase voltage is supplied to the asynchronous induction machine with sliprings;
the machine starts from standstill, accelerating inertias against load torque quadratic dependent on speed,
using a starting resistance. At time tRheostat external rotor resistance is shortened, finally reaching nominal speed.</p>

<p>
Simulate for 1.5 seconds and plot (versus time):
</p>

<ul>
<li>currentRMSsensorM|E.I: stator current RMS</li>
<li>aimsM/E.rpm_mechanical: motor's speed</li>
<li>aimsM|E.tau_electrical: motor's torque</li>
</ul>
</HTML>"));
    
    end AIMS_start;
  
    model SMPM_Inverter 
    "Test example 6: PermanentMagnetSynchronousInductionMachine with inverter" 
      extends Modelica.Icons.Example;
    
      constant Integer m=3 "Number of phases";
      parameter Modelica.SIunits.Voltage VsNominal=100 
      "Nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
      parameter Modelica.SIunits.Frequency f=50 "Actual frequency";
      parameter Modelica.SIunits.Time tRamp=1 "Frequency ramp";
      parameter Modelica.SIunits.Torque T_Load=181.4 "Nominal load torque";
      parameter Modelica.SIunits.Time tStep=2 "Time of load torque step";
      parameter Modelica.SIunits.Inertia J_Load=0.29 "Load's moment of inertia";
    
      parameter Integer p = 2 "Number of pole pairs";
      parameter Modelica.SIunits.Resistance Rs=0.03 
      "warm stator resistance per phase";
      parameter Modelica.SIunits.Inductance Lssigma=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "stator stray inductance per phase";
      parameter Modelica.SIunits.Inductance Lmd=3*sqrt(1 - 0.0667)/(2*Modelica.Constants.pi*fsNominal) 
      "main field inductance";
      parameter Modelica.SIunits.Inductance Lmq=0.8*Lmd "main field inductance";
      parameter Modelica.SIunits.Inductance Lrsigmad=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Inductance Lrsigmaq=0.8*Lrsigmad 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrd=0.04 
      "warm rotor resistance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrq=0.8*Rrd 
      "warm rotor resistance (equivalent three phase winding)";
    
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (extent=[-100, 80; -80, 100], rotation=-90);
      Modelica.Electrical.MultiPhase.Basic.Star star(
        final m=m) 
        annotation (extent=[-50, 80; -70, 100]);
      Modelica.Electrical.MultiPhase.Sources.SignalVoltage signalVoltage(
        final m=m) 
        annotation (extent=[10,70; -10,50],    rotation=-90);
      Modelica.Blocks.Sources.Ramp ramp(
        height=f,
        duration=tRamp) 
        annotation (extent=[-80,50; -60,70]);
      Modelica.Electrical.Machines.Examples.Utilities.VfController vfController(
        final m=m,
        VNominal=VsNominal,
        fNominal=fsNominal,
        BasePhase=+Modelica.Constants.pi/2) 
        annotation (extent=[-40,50; -20,70]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorM 
        annotation (extent=[10,20; -10,40], rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxM 
        annotation (extent=[-20,0; 0,20]);
      Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_PermanentMagnet
      smpmM(
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigmad=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rrd=Rrd,
        Rrq=Rrq,
        useDamperCage=true,
        p=p) 
        annotation (extent=[-20,-20; 0,0],rotation=0);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleM(
         p=p) 
        annotation (extent=[30,-20; 10,0], rotation=-90);
      Modelica.Mechanics.Rotational.Inertia loadInertiaM(
        J=J_Load) 
        annotation (extent=[40,-20; 60,0]);
      Modelica.Mechanics.Rotational.TorqueStep torqueStepM(
        startTime=tStep,
        stepTorque=-T_Load) 
        annotation (extent=[90,-20; 70,0]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorE 
        annotation (extent=[-50,20; -30,40],rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxE 
        annotation (extent=[-20,-52; 0,-32]);
      Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnetDamperCage
      smpmE(
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigma=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rr=Rrd,
        Rrq=Rrq,
        DamperCage=true,
        p=p) 
        annotation (extent=[-20,-72; 0,-52],     rotation=0);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleE(
        p=p) 
        annotation (extent=[30,-72; 10,-52],
        rotation=-90);
      Modelica.Mechanics.Rotational.Inertia loadInertiaE(
        J=J_Load) 
        annotation (extent=[40,-72; 60,-52]);
      Modelica.Mechanics.Rotational.TorqueStep torqueStepE(
        startTime=tStep,
        stepTorque=-T_Load) 
        annotation (extent=[90,-72; 70,-52]);
    equation 
      connect(signalVoltage.plug_n, star.plug_p) 
        annotation (points=[-1.22293e-15,70; -1.22293e-15,90; -50,90],
                                                        style(color=3));
      connect(star.pin_n, ground.p) 
        annotation (points=[-70, 90; -80, 90], style(color=3));
      connect(ramp.y, vfController.u) 
        annotation (points=[-59,60; -42,60], style(color=3, rgbcolor={0,0,255}));
      connect(vfController.y, signalVoltage.v) 
        annotation (points=[-19,60; -7,60],  style(color=3, rgbcolor={0,0,255}));
      connect(loadInertiaM.flange_b, torqueStepM.flange) 
        annotation (points=[60,-10; 70,-10], style(color=0, rgbcolor={0,0,0}));
      connect(signalVoltage.plug_p,currentRMSsensorM. plug_p)  annotation (points=[
          1.68051e-18,50; 0,50; 0,40; 1.68051e-18,40],     style(color=3,
            rgbcolor={0,0,255}));
      connect(rotorAngleM.plug_n, smpmM.plug_sn)  annotation (points=[26,
          1.8735e-16; 26,6; -16,6; -16,5.55112e-16],
                      style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleM.plug_p, smpmM.plug_sp)  annotation (points=[14,
          9.22873e-16; 10,0; 6,1.47798e-15; 6,5.55112e-16; -4,5.55112e-16],
                                     style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleM.flange, smpmM.flange_a) 
        annotation (points=[10,-10; 5.55112e-16,-10],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(smpmM.flange_a, loadInertiaM.flange_a) 
        annotation (points=[5.55112e-16,-10; 40,-10],
                                            style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxM.negativeMachinePlug, smpmM.plug_sn)  annotation (points=[-16,
          -5.55112e-16; -16,5.55112e-16],
                               style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxM.positiveMachinePlug, smpmM.plug_sp)  annotation (
          points=[-4,-5.55112e-16; -4,-2.77556e-16; -4,-2.77556e-16; -4,
          -2.43843e-22; -4,5.55112e-16; -4,5.55112e-16],
                                   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxM.plugToGrid,currentRMSsensorM. plug_n) annotation (
          points=[-10,2; -10,20; -1.22293e-15,20],           style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(loadInertiaE.flange_b, torqueStepE.flange) 
        annotation (points=[60,-62; 70,-62], style(color=0, rgbcolor={0,0,0}));
      connect(rotorAngleE.plug_n, smpmE.plug_sn)  annotation (points=[26,-52; 26,
            -46; -16,-46; -16,-52],
                      style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleE.plug_p, smpmE.plug_sp)  annotation (points=[14,-52;
          -4,-52],                   style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleE.flange, smpmE.flange_a) 
        annotation (points=[10,-62; 5.55112e-16,-62],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(smpmE.flange_a, loadInertiaE.flange_a) 
        annotation (points=[5.55112e-16,-62; 40,-62],
                                            style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxE.negativeMachinePlug, smpmE.plug_sn)  annotation (points=[-16,-52;
            -16,-52],          style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxE.positiveMachinePlug, smpmE.plug_sp)  annotation (
          points=[-4,-52; -4,-52], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
    
      annotation (
        Diagram,
        experiment(
          StopTime=4,
          Interval=0.0005,
          Tolerance=1e-05),
        experimentSetupOutput(doublePrecision=true),
        Documentation(info="<HTML>
<h4>Permanent magnet synchronous induction machine fed by an ideal inverter</h4>
<p>
An ideal frequency inverter is modeled by using a VfController and a threephase SignalVoltage.<br>
Frequency is raised by a ramp, causing the permanent magnet synchronous induction machine to start, 
and accelerating inertias.</p>

<p>At time tStep a load step is applied. Simulate for 1.5 seconds and plot (versus time):</p>

<ul>
<li>currentRMSsensorM|E.I: stator current RMS</li>
<li>pmsmM|E.rpm_mechanical: motor's speed</li>
<li>pmsmM|E.tau_electrical: motor's torque</li>
<li>rotorAnglepmsmM|E.rotorAngle: rotor displacement angle</li>
</ul>
</HTML>"));
      connect(currentRMSsensorE.plug_p, currentRMSsensorM.plug_p)  annotation (
          points=[-40,40; 1.68051e-18,40], style(color=3, rgbcolor={0,0,255}));
      connect(currentRMSsensorE.plug_n, terminalBoxE.plugToGrid)  annotation (
          points=[-40,20; -40,-40; -10,-40; -10,-50], style(color=3, rgbcolor={0,0,
              255}));
    end SMPM_Inverter;
  
    model SMEE_Gen 
    "Test example 7: ElectricalExcitedSynchronousInductionMachine as Generator" 
      extends Modelica.Icons.Example;
    
      constant Integer m=3 "Number of phases";
      parameter Modelica.SIunits.Voltage VsNominal=100 
      "Nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
      parameter Modelica.SIunits.Conversions.NonSIunits.AngularVelocity_rpm rpm
      =                                                                         1499 
      "Nominal speed";
      parameter Modelica.SIunits.Current Ie = 19 "Excitation current";
      parameter Modelica.SIunits.Current Ie0 = 10 "Initial excitation current";
      parameter Modelica.SIunits.Conversions.NonSIunits.Angle_deg gamma0 = 0 
      "Initial rotor displacement angle";
    
      parameter Integer p = 2 "Number of pole pairs";
      parameter Modelica.SIunits.Resistance Rs=0.03 
      "warm stator resistance per phase";
      parameter Modelica.SIunits.Inductance Lssigma=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "stator stray inductance per phase";
      parameter Modelica.SIunits.Inductance Lmd=3*sqrt(1 - 0.0667)/(2*Modelica.Constants.pi*fsNominal) 
      "main field inductance";
      parameter Modelica.SIunits.Inductance Lmq=0.8*Lmd "main field inductance";
      parameter Modelica.SIunits.Inductance Lrsigmad=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Inductance Lrsigmaq=0.8*Lrsigmad 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrd=0.04 
      "warm rotor resistance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrq=0.8*Rrd 
      "warm rotor resistance (equivalent three phase winding)";
    
      Modelica.Electrical.MultiPhase.Basic.Star star(
        final m=m) 
        annotation (extent=[-50,70; -70,90]);
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (extent=[-100,70; -80,90],    rotation=-90);
      Modelica.Electrical.MultiPhase.Sources.SineVoltage sineVoltage(
        final m=m,
        final V=fill(VsNominal*sqrt(2), m),
        final freqHz=fill(fsNominal, m)) 
        annotation (extent=[-20,70; -40,90]);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxM 
        annotation (extent=[-20,20; 0,40]);
      Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ElectricalExcited
      smeeM(
        phi_mechanical(start=-(Modelica.Constants.pi + Modelica.SIunits.Conversions.from_deg(gamma0))/p),
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigmad=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rrd=Rrd,
        Rrq=Rrq) 
        annotation (extent=[-20,0; 0,20], rotation=0);
      Modelica.Electrical.Analog.Sources.RampCurrent rampCurrentM(
        duration=0.1,
        I=Ie - Ie0,
        offset=Ie0) 
        annotation (extent=[-60,0; -40,20], rotation=90);
      Modelica.Electrical.Analog.Basic.Ground groundM 
        annotation (extent=[-98,-10; -78,10],    rotation=-90);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleM(p=p) 
        annotation (extent=[30,0; 10,20],    rotation=-90);
      Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor 
      mechanicalPowerSensorM 
        annotation (extent=[40,0; 60,20]);
      Modelica.Mechanics.Rotational.ConstantSpeed constantSpeedM(
        final w_fixed=Modelica.SIunits.Conversions.from_rpm(rpm)) 
        annotation (extent=[90,0; 70,20]);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxE 
        annotation (extent=[-20,-40; 0,-20]);
      Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcitedDamperCage
      smeeE(  phi_mechanical(start=-(Modelica.Constants.pi + Modelica.SIunits.Conversions.from_deg(gamma0))/p),
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigma=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rr=Rrd,
        Rrq=Rrq) 
        annotation (extent=[-20,-60; 0,-40], rotation=0);
      Modelica.Electrical.Analog.Sources.RampCurrent rampCurrentE(
        duration=0.1,
        I=Ie - Ie0,
        offset=Ie0) 
        annotation (extent=[-60,-60; -40,-40], rotation=90);
      Modelica.Electrical.Analog.Basic.Ground groundE 
        annotation (extent=[-98,-70; -78,-50],   rotation=-90);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleE(
        p=p) 
        annotation (extent=[30,-60; 10,-40], rotation=-90);
      Modelica.Electrical.Machines.Sensors.MechanicalPowerSensor 
      mechanicalPowerSensorE 
        annotation (extent=[40,-60; 60,-40]);
      Modelica.Mechanics.Rotational.ConstantSpeed constantSpeedE(
        final w_fixed=Modelica.SIunits.Conversions.from_rpm(rpm)) 
        annotation (extent=[90,-60; 70,-40]);
    equation 
      connect(rotorAngleE.plug_n,smeeE. plug_sn)  annotation (points=[26,-40;
          26,-30; -16,-30; -16,-40],
                      style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleE.plug_p,smeeE. plug_sp)  annotation (points=[14,-40;
          -4,-40],                   style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleE.flange,smeeE. flange_a) 
        annotation (points=[10,-50; 5.55112e-16,-50],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(star.pin_n, ground.p) 
        annotation (points=[-70,80; -80,80],   style(color=3));
      connect(star.plug_p, sineVoltage.plug_n)   annotation (points=[-50,80; -40,80],
                     style(color=3, rgbcolor={0,0,255}));
      connect(smeeE.flange_a,mechanicalPowerSensorE. flange_a) 
        annotation (points=[5.55112e-16,-50; 40,-50],
                                            style(color=0, rgbcolor={0,0,0}));
      connect(mechanicalPowerSensorE.flange_b,constantSpeedE. flange) 
        annotation (points=[60,-50; 70,-50], style(color=0, rgbcolor={0,0,0}));
      connect(rampCurrentE.p,groundE. p) annotation (points=[-50,-60; -78,-60],
          style(color=3, rgbcolor={0,0,255}));
      connect(rampCurrentE.p,smeeE. pin_en)  annotation (points=[-50,-60; -40,-60;
            -40,-56; -20,-56],      style(color=3, rgbcolor={0,0,255}));
      connect(rampCurrentE.n,smeeE. pin_ep)  annotation (points=[-50,-40; -40,-40;
            -40,-44; -20,-44],      style(color=3, rgbcolor={0,0,255}));
      connect(smeeE.plug_sn,terminalBoxE. negativeMachinePlug)  annotation (
          points=[-16,-40; -16,-40], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(smeeE.plug_sp,terminalBoxE. positiveMachinePlug)  annotation (
          points=[-4,-40; -4,-40], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(rotorAngleM.plug_n, smeeM.plug_sn)  annotation (points=[26,20; 26,30;
            -16,30; -16,20],
                      style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleM.plug_p, smeeM.plug_sp)  annotation (points=[14,20; -4,
          20],                       style(color=3, rgbcolor={0,0,255}));
      connect(rotorAngleM.flange, smeeM.flange_a) 
        annotation (points=[10,10; 5.55112e-16,10],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(smeeM.flange_a, mechanicalPowerSensorM.flange_a) 
        annotation (points=[5.55112e-16,10; 40,10],
                                            style(color=0, rgbcolor={0,0,0}));
      connect(mechanicalPowerSensorM.flange_b, constantSpeedM.flange) 
        annotation (points=[60,10; 70,10],   style(color=0, rgbcolor={0,0,0}));
      connect(rampCurrentM.p, groundM.p) annotation (points=[-50,-5.55112e-16;
          -58,0; -64,5.56792e-16; -64,1.1119e-15; -78,1.1119e-15],
          style(color=3, rgbcolor={0,0,255}));
      connect(rampCurrentM.p, smeeM.pin_en)  annotation (points=[-50,
          -5.55112e-16; -40,-5.55112e-16; -40,4; -20,4],
                                    style(color=3, rgbcolor={0,0,255}));
      connect(rampCurrentM.n, smeeM.pin_ep)  annotation (points=[-50,20; -40,20;
            -40,16; -20,16],        style(color=3, rgbcolor={0,0,255}));
      connect(smeeM.plug_sn,terminalBoxM. negativeMachinePlug)  annotation (
          points=[-16,20; -16,20],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(smeeM.plug_sp,terminalBoxM. positiveMachinePlug)  annotation (
          points=[-4,20; -4,20],   style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(sineVoltage.plug_p,terminalBoxE. plugToGrid) annotation (points=[-20,80;
            -10,80; -10,40; -30,40; -30,-20; -10,-20; -10,-38],
                                style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(sineVoltage.plug_p,terminalBoxM. plugToGrid) annotation (points=[-20,80;
            -10,80; -10,22],                                        style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
    
      annotation (
        Diagram,
        experiment(
          StopTime=30,
          Interval=0.005,
          Tolerance=1e-05),
        experimentSetupOutput(doublePrecision=true),
        Documentation(info="<HTML>
<h4>Electrical excited synchronous induction machine as generator</h4>
<p>
An electrically excited synchronous generator is connected to the grid and driven with constant speed. 
Since speed is slightly smaller than synchronous speed corresponding to mains frequency, 
rotor angle is very slowly increased. This allows to see several charactersistics dependent on rotor angle.
</p>

<p>
Simulate for 30 seconds and plot (versus RotorAngle1.rotorAngle):
</p>

<ul>
<li>speedM|E.tau_electrical</li>
<li>mechanicalPowerSensorM|E.P</li>
</ul>
</HTML>"));
    
    end SMEE_Gen;
  
    model SMR_Inverter 
    "Test example 5: SynchronousInductionMachineReluctanceRotor with inverter" 
      extends Modelica.Icons.Example;
    
      constant Integer m=3 "Number of phases";
      parameter Modelica.SIunits.Voltage VsNominal=100 
      "Nominal RMS voltage per phase";
      parameter Modelica.SIunits.Frequency fsNominal=50 "Nominal frequency";
      parameter Modelica.SIunits.Frequency f=50 "Actual frequency";
      parameter Modelica.SIunits.Time tRamp=1 "Frequency ramp";
      parameter Modelica.SIunits.Torque T_Load=46 "Nominal load torque";
      parameter Modelica.SIunits.Time tStep=1.2 "Nime of load torque step";
      parameter Modelica.SIunits.Inertia J_Load=0.29 "Load's moment of inertia";
    
      parameter Integer p = 2 "Number of pole pairs";
      parameter Modelica.SIunits.Resistance Rs=0.03 
      "warm stator resistance per phase";
      parameter Modelica.SIunits.Inductance Lssigma=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "stator stray inductance per phase";
      parameter Modelica.SIunits.Inductance Lmd=3*sqrt(1 - 0.0667)/(2*Modelica.Constants.pi*fsNominal) 
      "main field inductance";
      parameter Modelica.SIunits.Inductance Lmq=0.8*Lmd "main field inductance";
      parameter Modelica.SIunits.Inductance Lrsigmad=3*(1 - sqrt(1 - 0.0667))/(2*Modelica.Constants.pi*fsNominal) 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Inductance Lrsigmaq=0.8*Lrsigmad 
      "rotor stray inductance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrd=0.04 
      "warm rotor resistance (equivalent three phase winding)";
      parameter Modelica.SIunits.Resistance Rrq=0.8*Rrd 
      "warm rotor resistance (equivalent three phase winding)";
    
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (extent=[-100, 80; -80, 100], rotation=-90);
      Modelica.Electrical.MultiPhase.Basic.Star star(
        final m=m) 
        annotation (extent=[-50, 80; -70, 100]);
      Modelica.Electrical.MultiPhase.Sources.SignalVoltage signalVoltage(
        final m=m) 
        annotation (extent=[10,70; -10,50],    rotation=-90);
      Modelica.Blocks.Sources.Ramp ramp(height=f, duration=tRamp) 
        annotation (extent=[-80,50; -60,70]);
      Modelica.Electrical.Machines.Examples.Utilities.VfController vfController(
        final m=m,
        VNominal=VsNominal,
        fNominal=fsNominal) 
        annotation (extent=[-40,50; -20,70]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorM 
        annotation (extent=[-10,20; -30,40],rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxM 
        annotation (extent=[-30,0; -10,20]);
      Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ReluctanceRotor
      smrM(
        p=p,
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigmad=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rrd=Rrd,
        Rrq=Rrq) 
        annotation (extent=[-30,-20; -10,0]);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleM(
        p=p) 
        annotation (extent=[0,0; 20,-20],  rotation=90);
      Modelica.Mechanics.Rotational.Inertia loadInertiaM(
        J=J_Load) 
        annotation (extent=[30,-20; 50,0]);
      Modelica.Mechanics.Rotational.TorqueStep torqueStepM(
        startTime=tStep,
        stepTorque=-T_Load) 
        annotation (extent=[80,-20; 60,0]);
      Modelica.Electrical.Machines.Sensors.CurrentRMSsensor currentRMSsensorE 
        annotation (extent=[-70,20; -50,40],rotation=-90);
      Modelica.Electrical.Machines.Examples.Utilities.TerminalBox terminalBoxE 
        annotation (extent=[-30,-60; -10,-40]);
      Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotorDamperCage
      smrE(
        p=p,
        Rs=Rs,
        Lssigma=Lssigma,
        Lmd=Lmd,
        Lmq=Lmq,
        Lrsigma=Lrsigmad,
        Lrsigmaq=Lrsigmaq,
        Rr=Rrd,
        Rrq=Rrq) 
        annotation (extent=[-30,-80; -10,-60]);
      Modelica.Electrical.Machines.Sensors.RotorAngle rotorAngleE(
        p=p) 
        annotation (extent=[0,-60; 20,-80],rotation=90);
      Modelica.Mechanics.Rotational.Inertia loadInertiaE(
        J=J_Load) 
        annotation (extent=[30,-80; 50,-60]);
      Modelica.Mechanics.Rotational.TorqueStep torqueStepE(
        startTime=tStep,
        stepTorque=-T_Load) 
        annotation (extent=[80,-80; 60,-60]);
    equation 
      connect(signalVoltage.plug_n, star.plug_p) 
        annotation (points=[-1.22293e-15,70; -1.22293e-15,90; -50,90],
                                                        style(color=3));
      connect(star.pin_n, ground.p) 
        annotation (points=[-70, 90; -80, 90], style(color=3));
      connect(smrE.flange_a, loadInertiaE.flange_a) annotation (points=[-10,-70; 30,
            -70],    style(color=0, rgbcolor={0,0,0}));
      connect(ramp.y, vfController.u) 
        annotation (points=[-59,60; -42,60], style(color=3, rgbcolor={0,0,255}));
      connect(vfController.y, signalVoltage.v) 
        annotation (points=[-19,60; -7,60],  style(color=3, rgbcolor={0,0,255}));
      connect(loadInertiaE.flange_b, torqueStepE.flange) 
        annotation (points=[50,-70; 60,-70], style(color=0, rgbcolor={0,0,0}));
      connect(smrE.plug_sn,rotorAngleE. plug_n)  annotation (points=[-26,-60; -26,
            -52; 16,-52; 16,-60],
                      style(color=3, rgbcolor={0,0,255}));
      connect(smrE.plug_sp,rotorAngleE. plug_p)  annotation (points=[-14,-60; 4,
          -60],                   style(color=3, rgbcolor={0,0,255}));
      connect(smrE.flange_a,rotorAngleE. flange) 
        annotation (points=[-10,-70; -5.55112e-16,-70],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxE.positiveMachinePlug,smrE. plug_sp)  annotation (points=[-14,-60;
            -14,-60],        style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxE.negativeMachinePlug,smrE. plug_sn)  annotation (
          points=[-26,-60; -26,-60], style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxE.plugToGrid,currentRMSsensorE. plug_n) annotation (
          points=[-20,-58; -20,-40; -60,-40; -60,20],        style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(smrM.flange_a, loadInertiaM.flange_a) annotation (points=[-10,-10; 30,
            -10],    style(color=0, rgbcolor={0,0,0}));
      connect(loadInertiaM.flange_b, torqueStepM.flange) 
        annotation (points=[50,-10; 60,-10], style(color=0, rgbcolor={0,0,0}));
      connect(smrM.plug_sn, rotorAngleM.plug_n)  annotation (points=[-26,
          5.55112e-16; -26,8; 16,8; 16,9.22873e-16],
                      style(color=3, rgbcolor={0,0,255}));
      connect(smrM.plug_sp, rotorAngleM.plug_p)  annotation (points=[-14,
          5.55112e-16; -4,5.55112e-16; -4,1.8735e-16; 4,1.8735e-16],
                                  style(color=3, rgbcolor={0,0,255}));
      connect(smrM.flange_a, rotorAngleM.flange) 
        annotation (points=[-10,-10; -5.55112e-16,-10],
                                           style(color=0, rgbcolor={0,0,0}));
      connect(terminalBoxM.positiveMachinePlug, smrM.plug_sp)  annotation (points=[-14,
          -5.55112e-16; -14,5.55112e-16],
                             style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(terminalBoxM.negativeMachinePlug, smrM.plug_sn)  annotation (
          points=[-26,-5.55112e-16; -26,5.55112e-16],
                                     style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=10,
          rgbfillColor={135,135,135},
          fillPattern=1));
      connect(currentRMSsensorM.plug_n,terminalBoxM. plugToGrid) annotation (
          points=[-20,20; -20,2],                     style(
          color=3,
          rgbcolor={0,0,255},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
      connect(signalVoltage.plug_p, currentRMSsensorM.plug_p) annotation (points=[
          1.68051e-18,50; 1.68051e-18,40; -20,40],   style(color=3, rgbcolor={0,0,
              255}));
      connect(signalVoltage.plug_p, currentRMSsensorE.plug_p) annotation (points=[
          1.68051e-18,50; 0,50; 0,40; -60,40],   style(color=3, rgbcolor={0,0,255}));
    
      annotation (
        Diagram,
        experiment(StopTime=1.5, Interval=0.001),
        experimentSetupOutput(
          doublePrecision=true),
        Documentation(info="<HTML>
<h4>Synchronous induction machine with reluctance rotor fed by an ideal inverter</h4>
<p>
An ideal frequency inverter is modeled by using a VfController and a threephase SignalVoltage.<br>
Frequency is raised by a ramp, causing the reluctance machine to start, 
and accelerating inertias.<br>At time tStep a load step is applied.
</p>

<p>
Simulate for 1.5 seconds and plot (versus time):
</p>

<ul>
<li>currentRMSsensorM|E.I: stator current RMS</li>
<li>smrM|E.rpm_mechanical: motor's speed</li>
<li>smrM|E.tau_electrical: motor's torque</li>
<li>rotorAngleM|R.rotorAngle: rotor displacement angle</li>
</ul>
</HTML>"));
    
    end SMR_Inverter;
  
  end Examples;


  package Components "Basic fundamental wave components" 
    model Ground "Magnetic ground" 
    
      Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_p 
      "Complex magnetic port" 
        annotation (extent=[-10,90; 10,110]);
    
      annotation (Diagram(
          Line(points=[0,100; 0,50], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-60,50; 60,50], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-40,30; 40,30], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-20,10; 20,10], style(color=45, rgbcolor={255,128,0}))),
            Icon(
          Line(points=[0,100; 0,50], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-60,50; 60,50], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-40,30; 40,30], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-20,10; 20,10], style(color=45, rgbcolor={255,128,0}))),
      Documentation(info="<html>

<p>
Grounding of the complex magnetic potential. Each magnetic circuit has to be grounded at least one point of the circuit.
</p>

</html>"));
    
    equation 
      port_p.V_m.re = 0;
      port_p.V_m.im = 0;
    end Ground;
  
    model Reluctance "Salient reluctance" 
    
    import Modelica.Constants.pi;
    
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPort;
      parameter Modelica_FundamentalWave.Math.SIunits.SalientReluctance R_m(
        d(start=1),
        q(start=1)) "Magnetic reluctance in d=re and q=im axis";
      annotation (Icon(
          Rectangle(extent=[-70,30; 70,-30], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=7,
              rgbfillColor={255,255,255})),
          Line(points=[-96,0; -70,0], style(color=45, rgbcolor={255,128,0})),
          Line(points=[70,0; 96,0], style(color=45, rgbcolor={255,128,0})),
          Text(extent=[0,60; 0,100],        string="%name",
            style(color=45, rgbcolor={255,128,0})),
          Text(
            extent=[0,-70; 0,-110],
            style(color=0),
            string="R_m.re=%R_m.re, R_m.im=%R_m.im")), Documentation(info="<html>
<p>
The salient reluctance models the relationship between the complex magnetic potential difference
<img src=\"../Images/V_m.png\"> and the complex magnetic flux <img src=\"../Images/Phi.png\">,
</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Components/reluctance.png\">
</p>

<p>which can also be expressed in terms complex phasors:</p>

<p> 
&nbsp;&nbsp;<img src=\"../Images/Components/reluctance_alt.png\">
</p> 
</html>"));
    
    equation 
      (pi/2) * V_m.re = R_m.d * Phi.re;
      (pi/2) * V_m.im = R_m.q * Phi.im;
    end Reluctance;
  
    model SinglePhaseElectroMagneticConverter 
    "Single phase electro magnetic converter" 
    
    import Modelica.Constants.pi;
    
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive pin" 
        annotation (extent=[-110,90; -90,110], rotation=180);
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative pin" 
        annotation (extent=[-110,-110; -90,-90], rotation=180);
    
      Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_p 
      "Positive complex magnetic port" 
        annotation (extent=[90,90; 110,110]);
      Modelica_FundamentalWave.Interfaces.NegativeMagneticPort port_n 
      "Negative complex magnetic port" 
        annotation (extent=[90,-110; 110,-90]);
    
      parameter Real effectiveTurns "Effective number of turns";
      parameter Modelica.SIunits.Angle windingAngle 
      "Angle of winding axis (with respect to fundamental)";
    
      // Local electric single phase quantities
      Modelica.SIunits.Voltage v "Voltage drop";
      Modelica.SIunits.Current i "Current";
    
      // Local electromagnetic fundamental wave quantities
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
      annotation (Diagram, Icon(
          Ellipse(extent=[-60,60; 58,0], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Ellipse(extent=[-58,0; 60,-60], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Rectangle(extent=[-60,60; 0,-60], style(
              pattern=0,
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,-100; 94,-100; 84,-98; 76,-94; 64,-86; 50,-72; 42,
                -58; 36,-40; 30,-18; 30,0; 30,18; 34,36; 46,66; 62,84; 78,96;
                90,100; 100,100], style(
              color=45,
              rgbcolor={255,128,0},
              fillPattern=1)),
          Line(points=[0,60; -100,60; -100,100], style(
              color=3,
              rgbcolor={0,0,255},
              fillPattern=1)),
          Line(points=[0,-60; -100,-60; -100,-98], style(
              color=3,
              rgbcolor={0,0,255},
              fillPattern=1)),
          Text(
            extent=[0,160; 0,120],
            string="%name",
            style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=45,
              rgbfillColor={255,128,0},
              fillPattern=1))),
        Coordsys(grid=[2,2], scale=0),
      Documentation(info="<html>
<p>
The single phase winding has an effective number of turns, <img src=\"../Images/effectiveTurns.png\"> and a respective winding angle, <img src=\"../Images/windingAngle.png\">. The current in winding is <img src=\"../Images/i.png\">.
</p>

<p>
The total complex magnetic potential difference of the single phase winding is determined by: 
</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Components/singlephaseconverter_vm.png\">
</p>

<p>
In this equation the magneto motive forc is aligned with the winding axis. 
</p>

<p>
The voltage <img src=\"../Images/v.png\"> induced in the winding depends on the cosine between the the winding angle and the angle of the complex magnetic flux. Additionally, the magnitudes of the induced voltages are propotional to the respective number of turns. This relationship can be modeled by means of</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Components/singlephaseconverter_phi.png\">
</p>

<p>The single phase electro magnetic converter is a special case of the 
<a href=\"Modelica://MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">MultiPhaseElectroMagneticConverter</a>
</p>

</html>"));
    
    equation 
      // Complex magnetic flux into positive port 
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Complex magnetic potential difference
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      // Voltage equation
      v = pin_p.v - pin_n.v;
    
      // Current equations
      i = pin_p.i;
      pin_p.i + pin_n.i = 0;
    
      // Complex magnetic potential difference from currents, number
      // of turns and angles of winding axis
      V_m.re = (2/pi) * effectiveTurns*cos(windingAngle)*i;
      V_m.im = (2/pi) * effectiveTurns*sin(windingAngle)*i;
    
      // Induced voltages from complex magnetic flux, number of turns
      // and angles of winding axis
      -v = effectiveTurns*cos(windingAngle)*der(Phi.re)
         + effectiveTurns*sin(windingAngle)*der(Phi.im);
    
    end SinglePhaseElectroMagneticConverter;
  
    model MultiPhaseElectroMagneticConverter 
    "Multi phase electro magnetic converter" 
    
    import Modelica.Constants.pi;
    
      // constant Modelica.SIunits.Angle offset = 0.0000 
      //   "Development constant to be removed in the final version";
      Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p(
        final m=m) "Positive plug" 
        annotation (extent=[-110,90; -90,110], rotation=180);
      Modelica.Electrical.MultiPhase.Interfaces.NegativePlug plug_n(
        final m=m) "Negative plug" 
        annotation (extent=[-110,-110; -90,-90], rotation=180);
    
      Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_p 
      "Positive complex magnetic port" 
        annotation (extent=[90,90; 110,110]);
      Modelica_FundamentalWave.Interfaces.NegativeMagneticPort port_n 
      "Negative complex magnetic port" 
        annotation (extent=[90,-110; 110,-90]);
    
      parameter Integer m = 3 "Number of phases";
      parameter Real effectiveTurns[m] "Effective number of turns";
      parameter Modelica.SIunits.Angle windingAngle[m] 
      "Angle of winding axis (with respect to fundamental)";
    
      // Local electric multi phase quantities
      Modelica.SIunits.Voltage v[m] "Voltage drop";
      Modelica.SIunits.Current i[m] "Current";
    
      // Local electromagnetic fundamental wave quantities
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
      annotation (Diagram, Icon(
          Ellipse(extent=[-60,60; 58,0], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Ellipse(extent=[-58,0; 60,-60], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Rectangle(extent=[-60,60; 0,-60], style(
              pattern=0,
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,-100; 94,-100; 84,-98; 76,-94; 64,-86; 50,-72; 42,
                -58; 36,-40; 30,-18; 30,0; 30,18; 34,36; 46,66; 62,84; 78,96;
                90,100; 100,100], style(
              color=45,
              rgbcolor={255,128,0},
              fillPattern=1)),
          Line(points=[0,60; -100,60; -100,100], style(
              color=3,
              rgbcolor={0,0,255},
              fillPattern=1)),
          Line(points=[0,-60; -100,-60; -100,-98], style(
              color=3,
              rgbcolor={0,0,255},
              fillPattern=1)),
          Text(
            extent=[0,160; 0,120],
            string="%name",
            style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=45,
              rgbfillColor={255,128,0},
              fillPattern=1))),
        Documentation(info="<html>

<p>
Each phase <img src=\"../Images/k.png\"> of an <img src=\"../Images/m.png\"> phase winding has an effective number of turns, <img src=\"../Images/effectiveTurns_k.png\"> and an respective winging angle <img src=\"../Images/windingAngle_k.png\"> and a phase current <img src=\"../Images/i_k.png\">. 
</p>

<p>
The total complex magnetic potential difference of the mutli phase winding is determined by:
</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Components/multiphaseconverter_vm.png\">
</p>

<p>
In this equation each contribution of a winding magneto motive force on the total complex magnetic potential difference is aligned with the respective winding axis. 
</p>

<p>
The voltages <img src=\"../Images/v_k.png\"> induced in each winding depend on the cosinus between the the winding angle and the angle of the complex magnetic flux. Additionally, the magnitudes of the induced voltages are propotional to the respective number of turns. This relationship can be modeled by means of</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Components/multiphaseconverter_phi.png\">
</p>

<p>for <img src=\"../Images/k_in_1_m.png\"> and is also illustrated by the following figure:</p>

<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig:</b> Winding axis and location of complex magnetic flux</caption>
  <tr>
    <td>
      <img src=\"../Images/Components/coupling.png\">
    </td>
  </tr>
</table> 

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Components.SinglePhaseElectroMagneticConverter\">SinglePhaseElectroMagneticConverter</a>
<p>
</p>
</html>"));
    
    equation 
      // Flux into positive port 
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Magneto motive force
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      // Voltage equation
      v = plug_p.pin.v - plug_n.pin.v;
    
      // Current equations
      i = plug_p.pin.i;
      plug_p.pin.i + plug_n.pin.i = zeros(m);
    
      // Complex magnetic potential difference from currents, number
      // of turns and angles of winding axis
      V_m.re = (2/pi) * sum( effectiveTurns[k]*cos(windingAngle[k])*i[k] for k in 1:m);
      V_m.im = (2/pi) * sum( effectiveTurns[k]*sin(windingAngle[k])*i[k] for k in 1:m);
    
      // Induced voltages from complex magnetic flux, number of turns
      // and angles of winding axis
      for k in 1:m loop
        -v[k] = effectiveTurns[k]*cos(windingAngle[k])*der(Phi.re)
              + effectiveTurns[k]*sin(windingAngle[k])*der(Phi.im);
      end for;
    
    end MultiPhaseElectroMagneticConverter;
  
    annotation (DymolaStoredErrors);
  
    model Short "Salient reluctance" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
      annotation (Icon(
          Text(extent=[0,60; 0,100],        string="%name",
            style(color=45, rgbcolor={255,128,0})),
          Rectangle(extent=[-100,40; 100,-40], style(
              color=7,
              rgbcolor={255,255,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[-100,0; 100,0],style(color=45, rgbcolor={255,128,0}))),
        Documentation(info="<html>
<p>
This is a simple short cut branch.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica_FundamentalWave.Components.Idle\">Idle</a>
</p>

</html>"),
        Diagram);
    
    equation 
      connect(port_p, port_n) annotation (points=[-100,5.55112e-16; -1,
          5.55112e-16; -1,5.55112e-16; 100,5.55112e-16],
                                              style(
          color=45,
          rgbcolor={255,128,0},
          fillColor=7,
          rgbfillColor={255,255,255},
          fillPattern=1));
    end Short;
  
    model Idle "Salient reluctance" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPort;
      annotation (Icon(
          Text(extent=[0,60; 0,100],        string="%name",
            style(color=45, rgbcolor={255,128,0})),
          Rectangle(extent=[-100,40; 100,-40], style(
              color=7,
              rgbcolor={255,255,255},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[-100,0; -40,0], style(color=45, rgbcolor={255,128,0})),
          Line(points=[40,0; 100,0], style(color=45, rgbcolor={255,128,0}))),
        Documentation(info="<html>
<p>
This is a simple idle running branch.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica_FundamentalWave.Components.Short\">Short</a>
</p>

</html>"),
        Diagram(
          Line(points=[-100,0; -60,0], style(color=45, rgbcolor={255,128,0})),
          Line(points=[60,0; 100,0], style(color=45, rgbcolor={255,128,0})),
          Line(points=[-60,0; -40,2; -18,6; 0,14; 12,26], style(color=45, rgbcolor=
                  {255,128,0})),
          Line(points=[60,0; 40,-2; 18,-6; 0,-14; -12,-26], style(color=45,
                rgbcolor={255,128,0}))));
    equation 
      Phi.re = 0;
      Phi.im = 0;
    end Idle;
  end Components;


  package Machines 
  "Machine components and modelsElectric machine models based on FundamentalWave package" 
  
    package Components "Components specially for electric machines" 
    
      model SinglePhaseWinding 
      "Symmetric winding model coupling electrical and magnetic domain" 
      
        Interfaces.NegativeMagneticPort port_n "Negative complex magnetic port"
          annotation (extent=[90,-110; 110,-90]);
        Interfaces.PositiveMagneticPort port_p "Positive complex magnetic port"
          annotation (extent=[90,90; 110,110]);
      
        parameter Modelica.SIunits.Resistance R 
        "Winding reference resistance per phase";
        parameter Modelica.SIunits.Inductance Lsigma 
        "Winding stray inductance per phase";
        parameter Real effectiveTurns = 1 "Effective number of turns per phase";
        parameter Modelica.SIunits.Angle windingAngle 
        "Angle of the winding axis";
      
        annotation (Icon(
            Rectangle(extent=[-100,60; 100,-60], style(
                pattern=0,
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[100,-100; 94,-100; 84,-98; 76,-94; 64,-86; 50,-72; 42,-58;
                  36,-40; 30,-18; 30,0; 30,18; 34,36; 46,66; 62,84; 78,96; 90,100;
                  100,100],         style(
                color=45,
                rgbcolor={255,128,0},
                fillPattern=1)),
            Line(points=[40,60; -100,60; -100,100],style(
                color=3,
                rgbcolor={0,0,255},
                fillPattern=1)),
            Line(points=[40,-60; -100,-60; -100,-98],style(
                color=3,
                rgbcolor={0,0,255},
                fillPattern=1)),
            Line(points=[40,60; 100,20; 40,-20; 0,-20; -40,0; 0,20; 40,20; 100,-20;
                  40,-60], style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(
              extent=[0,160; 0,120],
              string="%name",
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=45,
                rgbfillColor={255,128,0},
                fillPattern=1))), Diagram,
        Documentation(info="<html>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseCageWinding\">
   SymmetricMultiPhaseCageWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding\">
   SaliencyCageWinding</a>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"));
      
        Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive pin" 
          annotation (extent=[-110,90; -90,110], rotation=180);
        Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative pin" 
          annotation (extent=[-110,-110; -90,-90], rotation=180);
        Modelica.Electrical.Analog.Basic.Inductor strayInductor(final L=Lsigma) 
          annotation (extent=[0,20; -20,40],   rotation=270);
        Modelica.Electrical.Analog.Basic.Resistor resistor(final R=R) 
          annotation (extent=[-20,80; 0,60],     rotation=90);
        Modelica_FundamentalWave.Components.SinglePhaseElectroMagneticConverter
        electroMagneticConverter(
          final effectiveTurns=effectiveTurns,
          final windingAngle=windingAngle) 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(pin_p, resistor.p) 
          annotation (points=[-100,100; -10,100; -10,80],
            style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(resistor.n, strayInductor.p) 
          annotation (points=[-10,60; -10,50; -10,40; -10,40], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(strayInductor.n, electroMagneticConverter.pin_p) 
          annotation (points=[-10,20; -10,15; -10,10; -10,10], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.pin_n, pin_n) 
          annotation (points=[-10,-10;-10,-100; -100,-100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.port_p, port_p) 
          annotation (points=[10,10; 10,100; 100,100], style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.port_n, port_n) 
          annotation (points=[10,-10; 10,-100; 100,-100], style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end SinglePhaseWinding;
    
      model SymmetricMultiPhaseWinding 
      "Symmetric winding model coupling electrical and magnetic domain" 
      
        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_p(
          final m=m) "Positive plug" 
          annotation (extent=[-110,90; -90,110], rotation=180);
        Modelica.Electrical.MultiPhase.Interfaces.NegativePlug plug_n(
          final m=m) "Negative plug" 
          annotation (extent=[-110,-110; -90,-90], rotation=180);
        Interfaces.NegativeMagneticPort port_n "Negative complex magnetic port"
          annotation (extent=[90,-110; 110,-90]);
        Interfaces.PositiveMagneticPort port_p "Positive complex magnetic port"
          annotation (extent=[90,90; 110,110]);
      
        parameter Integer m =  3 "Number of phases";
        parameter Modelica.SIunits.Resistance R "Winding resistance per phase";
        parameter Modelica.SIunits.Inductance Lsigma 
        "Winding stray inductance per phase";
        parameter Real effectiveTurns = 1 "Effective number of turns per phase";
      
        Modelica_FundamentalWave.Components.MultiPhaseElectroMagneticConverter 
        electroMagneticConverter(
          final m=m,
          final effectiveTurns=fill(effectiveTurns, m),
          final windingAngle=Modelica_FundamentalWave.Math.symmetricWindingAngle(m)) 
          annotation (extent=[-12,-10; 8,10]);
        Modelica.Electrical.MultiPhase.Basic.Inductor strayInductor(
          final m=m,
          final L=fill(Lsigma, m)) 
          annotation (extent=[0,20; -20,40], rotation=270);
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(
          final m=m,
          final R=fill(R, m)) 
          annotation (extent=[0,60; -20,80], rotation=270);
      
        annotation (Icon(
            Rectangle(extent=[-100,60; 100,-60], style(
                pattern=0,
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[100,-100; 94,-100; 84,-98; 76,-94; 64,-86; 50,-72; 42,-58;
                  36,-40; 30,-18; 30,0; 30,18; 34,36; 46,66; 62,84; 78,96; 90,100;
                  100,100],         style(
                color=45,
                rgbcolor={255,128,0},
                fillPattern=1)),
            Line(points=[40,60; -100,60; -100,100],style(
                color=3,
                rgbcolor={0,0,255},
                fillPattern=1)),
            Line(points=[40,-60; -100,-60; -100,-98],style(
                color=3,
                rgbcolor={0,0,255},
                fillPattern=1)),
            Line(points=[40,60; 100,20; 40,-20; 0,-20; -40,0; 0,20; 40,20; 100,-20;
                  40,-60], style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(
              extent=[0,160; 0,120],
              string="%name",
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=45,
                rgbfillColor={255,128,0},
                fillPattern=1))), Diagram,
        Documentation(info="<html>
<p>
The symmetrical multi phase winding consists of a symmetrical winding 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Basic.Resistor\">resistor</a>, a symmetrical 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Basic.Inductor\">stray inductor</a> and a symmetrical 
<a href=\"Modelica://Modelica_FundamentalWave.Components.MultiPhaseElectroMagneticConverter\">multi phase electro magnetic coupling</a>. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseCageWinding\">
   SymmetricMultiPhaseCageWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding\">
   SaliencyCageWinding</a>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"));
      
      equation 
        connect(plug_p, resistor.plug_p) 
          annotation (points=[-100,100; -10,100; -10,80],
            style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(resistor.plug_n, strayInductor.plug_p) 
          annotation (points=[-10,60; -10,55; -10,55; -10,50; -10,40; -10,40],
            style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(strayInductor.plug_n, electroMagneticConverter.plug_p) 
          annotation (points=[-10,20; -10,10; -12,10],
            style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.plug_n, plug_n) 
          annotation (points=[-12,-10;-12,-100; -100,-100],
            style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.port_p, port_p) 
          annotation (points=[8,10; 8,100; 100,100],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(electroMagneticConverter.port_n, port_n) 
          annotation (points=[8,-10; 8,-100; 100,-100],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end SymmetricMultiPhaseWinding;
    
      model SymmetricMultiPhaseCageWinding "Symmetrical rotor cage" 
      
      import Modelica.Constants.pi;
      
        extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
      
        parameter Integer m = 3 "Number of phases";
        parameter Modelica.SIunits.Resistance R "Cage resistance";
        parameter Modelica.SIunits.Inductance Lsigma "Cage stray inductance";
        parameter Real effectiveTurns = 1 "Effective number of turns";
      
        Modelica.SIunits.Current i[m] "Cage currents";
      
        annotation (Diagram, Icon(
            Ellipse(extent=[-80,80; 80,-80], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=9,
                rgbfillColor={175,175,175},
                fillPattern=1)),
            Ellipse(extent=[-20,76; 20,36], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[28,46; 68,6], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[28,-8; 68,-48], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-20,-36; 20,-76], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-68,-6; -28,-46], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-66,50; -26,10], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[-80,0; -100,0], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[100,0; 80,0], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(extent=[0,100; 0,140],       string="%name",
              style(color=3, rgbcolor={0,0,255}))),
          Documentation(info="<html>
<p>
<img src=\"../Images/Machines/Components/rotorcage.png\">
</p>
<p>
The symmetric rotor cage model of this library does not consist of rotor bars and end rings. Instead the symmetric cage is modeled by an equivalent symmetrical winding. The rotor cage model consists of 
<img src=\"../Images/m.png\"> phases. If the cage is modeled by equivalent stator winding parameters, the number of effective turns, <img src=\"../Images/effectiveTurns.png\">, has to be chosen equivalent to the effective number of stator turns.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding\">SaliencyCageWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"));
        Modelica_FundamentalWave.Components.MultiPhaseElectroMagneticConverter 
        winding(
          final m=m,
          final windingAngle={2*pi*(k - 1)/m for k in 1:m},
          final effectiveTurns=fill(effectiveTurns, m)) "Symmetric winding" 
          annotation (extent=[-10,-20; 10,0], rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Inductor strayInductor(
          final m=m,
          final L=fill(Lsigma, m)) 
          annotation (extent=[-30,-20; -10,-40],
                                               rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(
          final m=m,
          final R=fill(R, m)) 
          annotation (extent=[-30,-60; -10,-80],
                                               rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Star star(
          final m=m) 
          annotation (extent=[30,-90; 50,-70]);
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (extent=[80,-90; 60,-70],   rotation=270);
      
      initial equation 
        i = zeros(m);
      
      equation 
      
        i = resistor.i;
      
        connect(port_p, winding.port_p)                            annotation (points=[-100,
            5.55112e-16; -55,5.55112e-16; -55,1.16747e-15; -10,1.16747e-15],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(winding.port_n, port_n)                            annotation (points=[10,
            -5.72459e-17; 58,-5.72459e-17; 58,5.55112e-16; 100,5.55112e-16],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(ground.p,star. pin_n) annotation (points=[60,-80; 50,-80],   style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(strayInductor.plug_n, resistor.plug_p) 
                                           annotation (points=[-20,-40; -20,-60],
                                                                                style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(strayInductor.plug_p, winding.plug_p) annotation (points=[-20,-20; -10,
              -20], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
        connect(resistor.plug_n, winding.plug_n) annotation (points=[-20,-80; 20,-80;
              20,-20; 10,-20], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
        connect(star.plug_p, winding.plug_n) annotation (points=[30,-80; 20,-80; 20,-20;
              10,-20], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
      end SymmetricMultiPhaseCageWinding;
    
      model SaliencyCageWinding "Rotor cage with saliency in d- and q-axis" 
      
        extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
      
        parameter Modelica_FundamentalWave.Math.SIunits.SalientResistance R 
        "Salient cage resistance";
        parameter Modelica_FundamentalWave.Math.SIunits.SalientInductance 
        Lsigma "Salient cage stray inductance";
        parameter Real effectiveTurns = 1 "Effective number of turns";
      
        Modelica_FundamentalWave.Math.SIunits.SalientCurrent i "Cage current";
      
        annotation (Diagram, Icon(
            Ellipse(extent=[-80,80; 80,-80], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=9,
                rgbfillColor={175,175,175},
                fillPattern=1)),
            Ellipse(extent=[-20,76; 20,36], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[28,46; 68,6], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[28,-8; 68,-48], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-20,-36; 20,-76], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-68,-6; -28,-46], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-66,50; -26,10], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[-80,0; -100,0], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[100,0; 80,0], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(extent=[0,100; 0,140],       string="%name",
              style(color=3, rgbcolor={0,0,255}))),
          Documentation(info="<html>

<p>
The salient cage model is a two axis model with two phases. The electro magnetic coupling therefore is also two phase coupling model. The angles of the two winding axis are 0 and <img src=\"../Images/pi_over_2.png\">. This way an asymmetrical rotor cage with different resistances and stray inductances in d- and q-axis can be modeled.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseCageWinding\">
   SymmetricMultiPhaseCageWinding</a>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap\">RotorSaliencyAirGap</a>
</p>
</html>"));
        Modelica_FundamentalWave.Components.MultiPhaseElectroMagneticConverter 
        winding(
          final m=2,
          final windingAngle={0,Modelica.Constants.pi/2},
          final effectiveTurns=fill(effectiveTurns, 2)) "Symmetric winding" 
          annotation (extent=[-10,-20; 10,0], rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Inductor strayInductor(
          final m=2,
          final L={Lsigma.d,Lsigma.q}) 
          annotation (extent=[-30,-20; -10,-40],
                                               rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Resistor resistor(
          final m=2,
          final R={R.d,R.q}) 
          annotation (extent=[-30,-60; -10,-80],
                                               rotation=90);
        Modelica.Electrical.MultiPhase.Basic.Star star(
          final m=2) 
          annotation (extent=[28,-90; 48,-70]);
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (extent=[80,-90; 60,-70],   rotation=270);
      
      initial equation 
      i = Modelica_FundamentalWave.Math.Salient(0, 0);
      equation 
        i.d = resistor.i[1];
        i.q = resistor.i[2];
      
        connect(port_p, winding.port_p)                            annotation (points=[-100,
            5.55112e-16; -55,5.55112e-16; -55,1.16747e-15; -10,1.16747e-15],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(winding.port_n, port_n)                            annotation (points=[10,
            -5.72459e-17; 58,-5.72459e-17; 58,5.55112e-16; 100,5.55112e-16],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(ground.p,star. pin_n) annotation (points=[60,-80; 48,-80],   style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(strayInductor.plug_n, resistor.plug_p) 
                                           annotation (points=[-20,-40; -20,-60],
                                                                                style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(winding.plug_n, resistor.plug_n) annotation (points=[10,-20; 20,-20; 20,
              -80; -20,-80], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
        connect(star.plug_p, winding.plug_n) annotation (points=[28,-80; 20,-80; 20,-20;
              10,-20], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
        connect(strayInductor.plug_p, winding.plug_p) annotation (points=[-20,-20; -10,
              -20], style(
            color=3,
            rgbcolor={0,0,255},
            smooth=0));
      end SaliencyCageWinding;
    
      model RotorSaliencyAirGap "Air gap model with rotor saliency" 
      
      import Modelica.Constants.pi;
      
        Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_sp 
        "Positive complex magnetic stator port" 
          annotation (extent=[-110,90;-90,110]);
        Modelica_FundamentalWave.Interfaces.NegativeMagneticPort port_sn 
        "Negative complex magnetic stator port" 
          annotation (extent=[-110,-110; -90,-90]);
        Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_rp 
        "Positive complex magnetic rotor port" 
          annotation (extent=[90,90;110,110]);
        Modelica_FundamentalWave.Interfaces.NegativeMagneticPort port_rn 
        "Negative complex magnetic rotor port" 
          annotation (extent=[90,-110;110,-90]);
      
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a 
        "Flange of the rotor" 
          annotation (extent=[-10,110; 10,90]);
        Modelica.Mechanics.Rotational.Interfaces.Flange_a support 
        "Support at which the reaction torque is acting" 
          annotation (extent=[-10,-110; 10,-90]);
      
        parameter Integer p "Number of pole pairs";
        parameter Modelica_FundamentalWave.Math.SIunits.SalientInductance L0 
        "Salient inductance of a single unchorded coil w.r.t. the fundamental wave";
        final parameter Modelica_FundamentalWave.Math.SIunits.SalientReluctance
        R_m(
          d=1/L0.d,
          q=1/L0.q) "Reluctance of the air gap model";
      
        // Stator magnetic quantities
        Complex.ComplexMagneticPotentialDifference V_mss 
        "Complex magnetic potential difference of stator w.r.t. stator reference frame";
        Complex.ComplexMagneticPotentialDifference V_msr 
        "Complex magnetic potential difference of stator w.r.t. rotor reference frame";
        Complex.ComplexMagneticPotentialDifference V_mrr 
        "Complex magnetic potential difference of rotor w.r.t. rotor reference frame";
        // Complex.ComplexMagneticPotentialDifference V_mrs 
        //   "Complex magnetic potential difference of rotor w.r.t. stator reference frame";
      
        Complex.ComplexMagneticFlux Phi_ss 
        "Complex magnetic potential difference of stator w.r.t. stator reference frame";
        Complex.ComplexMagneticFlux Phi_sr 
        "Complex magnetic potential difference of stator w.r.t. rotor reference frame";
        Complex.ComplexMagneticFlux Phi_rr 
        "Complex magnetic potential difference of rotor w.r.t. rotor reference frame";
        // Complex.ComplexMagneticFlux Phi_rs 
        //   "Complex magnetic potential difference of rotor w.r.t. stator reference frame";
      
        // Electrical torque and mechanical angle
        Modelica.SIunits.Torque tauElectrical "Electrical torque";
        // Modelica.SIunits.Torque tauTemp "Electrical torque";
        Modelica.SIunits.Angle gamma 
        "Electrical angle between rotor and stator";
      
        annotation (Diagram, Icon(
            Ellipse(extent=[-100,100; 100,-100], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[-100,90; -100,60; -80,60], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[-100,-90; -100,-60; -80,-60], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[40,60; 100,60; 100,90], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[40,-60; 100,-60; 100,-90], style(
                color=45,
                rgbcolor={255,128,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Ellipse(extent=[-60,80; 60,-80], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Line(points=[0,80; 0,90], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
          Documentation(info="<html>
<p> 
This salient air gap model can be used for machines with uniform airgaps and for machines with rotor saliencies. The air gap model is not symmetrical towards stator and rotor since it is assumed the saliency always refers to the rotor. The saliency of the air gap is represented by a main field inductance in the d- and q-axis. 
</p>

<p>
For the mechanical interaction of the air gap model with the stator and the rotor it is equipped with to 
<a href=\"Modelica://Modelica.Mechanics.Rotational.Interfaces.Flange_a\">rotational connectors</a>. The torques acting on both connectors have the same absolute values but different signs. The difference between the stator and the rotor angle, 
<img src=\"../Images/gamma.png\">, is required for the transformation of the magnetic stator quantities to the rotor side.</p>

<p>
The air gap model has two magnetic stator and two magnetic rotor 
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.MagneticPort\">ports</a>. The magnetic potential difference and the magnetic flux of the stator (superscript s) are transformed to the rotor fixed reference frame (superscript r). The effective reluctances of the main field with respect to the d- and q-axis are considered then in the balance equations
</p>

<p>
&nbsp;&nbsp;<img src=\"../Images/Machines/Components/airgap.png\">
</p>

<p>
according to the following figure. 
</p>
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <caption align=\"bottom\"><b>Fig:</b> Magnetic equivalent circuit of the air gap model</caption>
  <tr>
    <td>
      <img src=\"../Images/Machines/Components/airgap_phasors.png\">
    </td>
  </tr>
</table> 

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding\">SinglePhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding\">SymmetricMultiPhaseWinding</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseCageWinding\">
   SymmetricMultiPhaseCageWinding</a>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding\">
   SaliencyCageWinding</a>
</p>

</html>"));
      
      equation 
        // Stator flux into positive stator port 
        port_sp.Phi.re = Phi_ss.re;
        port_sp.Phi.im = Phi_ss.im;
        // Balance of stator flux
        port_sp.Phi.re + port_sn.Phi.re = 0;
        port_sp.Phi.im + port_sn.Phi.im = 0;
      
        // Rotor flux into positive rotor port 
        port_rp.Phi.re = Phi_rr.re;
        port_rp.Phi.im = Phi_rr.im;
        // Balance of rotor flux
        port_rp.Phi.re + port_rn.Phi.re = 0;
        port_rp.Phi.im + port_rn.Phi.im = 0;
      
        // Magneto motive force of stator
        port_sp.V_m.re - port_sn.V_m.re = V_mss.re;
        port_sp.V_m.im - port_sn.V_m.im = V_mss.im;
      
        // Magneto motive force of stator
        port_rp.V_m.re - port_rn.V_m.re = V_mrr.re;
        port_rp.V_m.im - port_rn.V_m.im = V_mrr.im;
      
        // Transformation of fluxes between stator and rotor fixed frame
        // -- Phi_rs.re = + Phi_rr.re * cos(gamma) - Phi_rr.im * sin(gamma);
        // -- Phi_rs.im = + Phi_rr.re * sin(gamma) + Phi_rr.im * cos(gamma);
        // Phi_rr.re = + Phi_rs.re * cos(gamma) + Phi_rs.im * sin(gamma);
        // Phi_rr.im = - Phi_rs.re * sin(gamma) + Phi_rs.im * cos(gamma);
      
        // Transformed stator flux is not needed
        Phi_sr.re = + Phi_ss.re * cos(gamma) + Phi_ss.im * sin(gamma);
        Phi_sr.im = - Phi_ss.re * sin(gamma) + Phi_ss.im * cos(gamma);
        // Phi_ss.re = + Phi_sr.re * cos(gamma) - Phi_sr.im * sin(gamma);
        // Phi_ss.im = + Phi_sr.re * sin(gamma) + Phi_sr.im * cos(gamma);
      
        // Local balance of flux w.r.t. the rotor fixed frame
        0 = Phi_sr.re + Phi_rr.re;
        0 = Phi_sr.im + Phi_rr.im;
      
        // Transformation of magnetic potential difference between stator and rotor fixed frame
        // V_mrs.re = + V_mrr.re * cos(gamma) - V_mrr.im * sin(gamma);
        // V_mrs.im = + V_mrr.re * sin(gamma) + V_mrr.im * cos(gamma);
        // V_mrr.re = + V_mrs.re * cos(gamma) + V_mrs.im * sin(gamma);
        // V_mrr.im = - V_mrs.re * sin(gamma) + V_mrs.im * cos(gamma);
        V_msr.re = + V_mss.re * cos(gamma) + V_mss.im * sin(gamma);
        V_msr.im = - V_mss.re * sin(gamma) + V_mss.im * cos(gamma);
        // V_msr.re = + V_mss.re * cos(gamma) + V_mss.im * sin(gamma);
        // V_msr.im = - V_mss.re * sin(gamma) + V_mss.im * cos(gamma);
      
        // Local balance of maganeto motive force
        (pi/2) * (V_mrr.re - V_msr.re) = Phi_rr.re*R_m.d;
        (pi/2) * (V_mrr.im - V_msr.im) = Phi_rr.im*R_m.q;
      
        // Torque
        tauElectrical = - (pi*p/2)*(Phi_ss.re * V_mss.im - Phi_ss.im * V_mss.re);
      
        flange_a.tau = tauElectrical;
        support.tau = -tauElectrical;
      
        // Electrical angle between stator and rotor
        gamma = p*(flange_a.phi-support.phi);
      
        annotation (Icon(
            Text(
              extent=[0,20; 0,-20],
              string="%name",
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=45,
                rgbfillColor={255,128,0},
                fillPattern=1))));
      end RotorSaliencyAirGap;
    annotation (Documentation(info="<html>
<p>
This package contains components for 
<a href=\"Modelica://Modelica_FundamentalWave.Machines.AsynchronousInductionMachines\">asynchronous induction machines</a> and
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines\">synchronous induction machines</a>.
</p>
</html>"));
    end Components;
  
    package AsynchronousInductionMachines "Asynchronous inductioin machines" 
      model AIM_SquirrelCage 
      "Asynchronous induction machine with squirrel cage" 
        extends 
        Modelica.Electrical.Machines.Interfaces.PartialBasicInductionMachine(
          is(start=zeros(m)));
      
        parameter Real effectiveStatorTurns = 1 
        "Effective number of stator turns";
        parameter Modelica.SIunits.Resistance Rs "Stator resistance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lssigma 
        "Stator leakage inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lm "Main field inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lrsigma 
        "Rotor leakage inductance w.r.t. stator side" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Resistance Rr 
        "Rotor resistance w.r.t. stator side" 
           annotation(Dialog(group="Nominal resistances and inductances"));
      
        Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding
        statorWinding(
          final m=m,
          final R=Rs,
          final Lsigma=Lssigma,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric stator winding including resistances and stray inductances" 
          annotation (extent=[-10,20; 10,40], rotation=270);
        Components.RotorSaliencyAirGap airGap(
          final p=p, final L0(d=2*Lm/3/effectiveStatorTurns^2, q=2*Lm/3/
              effectiveStatorTurns^2)) 
          annotation (extent=[-10,-10; 10,10], rotation=270);
        Modelica_FundamentalWave.Components.Ground groundS 
        "Ground of stator magnetic circuit" 
          annotation (extent=[-40,0; -20,20]);
        Modelica_FundamentalWave.Components.Ground groundR 
        "Ground of rotor magnetic circuit" 
          annotation (extent=[-40,-30; -20,-10]);
        Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseCageWinding
        rotorCageWinding(
          final Lsigma=Lrsigma,
          final R=Rr,
          final m=m,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (extent=[10,-50; -10,-30]);
      
        annotation (Diagram, Icon,
        Documentation(info="<html>
<p>
Resistances and stray inductances of the machine refer to the stator phases. The symmetry of the stator and rotor is assumed. Only losses in stator and rotor resistance are taken into account. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.AsynchronousInductionMachines.AIM_SlipRing\">AIM_SlipRing</a>,
<a href=\"Modelica://Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage\">Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SquirrelCage</a>,
</p>
</html>"));
      equation 
        connect(statorWinding.plug_n, plug_sn) 
          annotation (points=[-10,40; -10,60; -60,60; -60,100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.support, internalSupport)         annotation (points=[-10,
            2.33651e-15; -10,0; -60,0; -60,-100; 20,-100],     style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.flange_a, inertiaRotor.flange_a)         annotation (
            points=[10,-1.33731e-15; 10,0; 60,0; 60,-1.72421e-15], style(color=0,
              rgbcolor={0,0,0}));
        connect(statorWinding.port_n, airGap.port_sn)     annotation (points=[-10,20;
            -10,17.5; -10,17.5; -10,15; -10,10; -10,10],   style(color=45,
              rgbcolor={255,128,0}));
        connect(groundS.port_p, statorWinding.port_n) 
                                                  annotation (points=[-30,20;
            -10,20],
                   style(color=45, rgbcolor={255,128,0}));
        connect(groundR.port_p, airGap.port_rn)         annotation (points=[-30,-10;
            -20,-10; -20,-10; -10,-10],   style(color=45, rgbcolor={255,128,0}));
        connect(statorWinding.port_p, airGap.port_sp)     annotation (points=[10,20;
            10,20; 10,10; 10,10],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0},
            fillPattern=1));
        connect(airGap.port_rn, rotorCageWinding.port_n) 
                                             annotation (points=[-10,-10; -10,
            -17.5; -10,-17.5; -10,-25; -10,-40; -10,-40],
                                                     style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rp, rotorCageWinding.port_p) 
                                             annotation (points=[10,-10; 10,-40],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_p, plug_sp) annotation (points=[10,40; 10,60;
            60,60; 60,100],style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end AIM_SquirrelCage;
    
      model AIM_SlipRing "Asynchronous induction machine with slip ring rotor" 
        extends 
        Modelica.Electrical.Machines.Interfaces.PartialBasicInductionMachine(        is(start=zeros(m)));
      
        Modelica.Electrical.MultiPhase.Interfaces.NegativePlug plug_rn(final m=m) 
          annotation (extent=[-110,-50; -90,-70]);
        Modelica.Electrical.MultiPhase.Interfaces.PositivePlug plug_rp(final m=m) 
          annotation (extent=[-110,70; -90,50]);
      
        parameter Real effectiveStatorTurns = 1 
        "Effective number of stator turns";
        parameter Modelica.SIunits.Resistance Rs "Stator resistance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lssigma 
        "Stator leakage inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lm "Main field inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lrsigma 
        "Rotor leakage inductance w.r.t. stator side" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Resistance Rr 
        "Rotor resistance w.r.t. stator side" 
           annotation(Dialog(group="Nominal resistances and inductances"));
      
        parameter Modelica.SIunits.Frequency fsNominal=50 "nominal frequency";
      
        parameter Boolean useTurnsRatio=true 
        "Use TurnsRatio or calculate from locked-rotor voltage?";
        parameter Real TurnsRatio(final min=Modelica.Constants.small)=1 
        "(ws*xis) / (wr*xir)" 
          annotation(Dialog(enable=useTurnsRatio));
        parameter Modelica.SIunits.Voltage VsNominal=100 
        "Nominal stator voltage per phase" 
          annotation(Dialog(enable=not useTurnsRatio));
        parameter Modelica.SIunits.Voltage VrLockedRotor=100*
          (2*pi*fsNominal*Lm)/sqrt(Rs^2+(2*pi*fsNominal*(Lm+Lssigma))^2) 
        "Locked-rotor voltage per phase" 
          annotation(Dialog(enable=not useTurnsRatio));
    protected 
        final parameter Real internalTurnsRatio=if useTurnsRatio then TurnsRatio else 
          VsNominal/VrLockedRotor*(2*pi*fsNominal*Lm)/sqrt(Rs^2+(2*pi*fsNominal*(Lm+Lssigma))^2);
    public 
        Components.RotorSaliencyAirGap airGap(
          final p=p, final L0(d=2*Lm/3/effectiveStatorTurns^2, q=2*Lm/3/
              effectiveStatorTurns^2)) 
          annotation (extent=[-10,-8; 10,12],  rotation=270);
        Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding
        rotorWinding(
          final m=3,
          final R=Rr,
          final Lsigma=Lrsigma,
          final effectiveTurns=effectiveStatorTurns/internalTurnsRatio) 
        "Symmetric rotor winding including resistances and stray inductances" 
          annotation (extent=[10,-40; -10,-20], rotation=90);
        Modelica_FundamentalWave.Components.Ground groundS 
        "Ground of stator magnetic circuit" 
          annotation (extent=[-40,0; -20,20]);
        Modelica_FundamentalWave.Components.Ground groundR 
        "Ground of rotor magnetic circuit" 
          annotation (extent=[-40,-30; -20,-10]);
        Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding
        statorWinding(
          final m=m,
          final R=Rs,
          final Lsigma=Lssigma,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric stator winding including resistances and stray inductances" 
          annotation (extent=[-10,20; 10,40], rotation=270);
      equation 
      
        annotation (Diagram, Icon(
               Line(points=[-100,50; -100,20; -60,20], style(color=3, rgbcolor={0,
                    0,255})), Line(points=[-100,-50; -100,-20; -60,-20], style(
                  color=3, rgbcolor={0,0,255}))),
        Documentation(info="<html>
<p>
Resistances and stray inductances of the machine always refer to either stator or rotor phases. The symmetry of the stator and rotor is assumed. Only losses in stator and rotor resistance are taken into account. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.AsynchronousInductionMachines.AIM_SquirrelCage\">AIM_SquirrelCage</a>,
<a href=\"Modelica://Modelica.Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing\">Electrical.Machines.BasicMachines.AsynchronousInductionMachines.AIM_SlipRing</a>,
</p>
</html>"));
        connect(airGap.support, internalSupport)         annotation (points=[-10,2;
            -10,0; -60,0; -60,-100; 20,-100],                  style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.flange_a, inertiaRotor.flange_a)         annotation (
            points=[10,2; 10,0; 60,0; 60,-1.72421e-15],            style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.port_rn, rotorWinding.port_n)      annotation (points=[-10,-8;
            -10,-11; -10,-11; -10,-14; -10,-20; -10,-20],       style(color=45,
              rgbcolor={255,128,0}));
        connect(airGap.port_rp, rotorWinding.port_p)      annotation (points=[10,-8;
            10,-20],   style(color=45, rgbcolor={255,128,0}));
        connect(groundR.port_p, airGap.port_rn)         annotation (points=[-30,-10;
            -20,-10; -20,-8; -10,-8],     style(color=45, rgbcolor={255,128,0}));
        connect(rotorWinding.plug_n, plug_rn) annotation (points=[-10,-40; -10,-60;
              -100,-60], style(color=3, rgbcolor={0,0,255}));
        connect(rotorWinding.plug_p, plug_rp) annotation (points=[10,-40; 10,-80; -80,
              -80; -80,60; -100,60], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_n, plug_sn) 
          annotation (points=[-10,40; -10,60; -60,60; -60,100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_p, plug_sp) annotation (points=[10,40; 10,60;
            60,60; 60,100],style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.port_n, airGap.port_sn) annotation (points=[-10,20;
            -10,18; -10,18; -10,16; -10,12; -10,12],   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.port_p, airGap.port_sp) annotation (points=[10,20;
            10,20; 10,12; 10,12],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(groundS.port_p, statorWinding.port_n) annotation (points=[-30,20;
            -10,20],
                   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end AIM_SlipRing;
    
    annotation (Documentation(info="<html>
<p>This package provides squirrel cage and slip ring induction machine models.</p>
</html>"));
    end AsynchronousInductionMachines;
  
    package SynchronousInductionMachines "Synchronous machines" 
      model SM_PermanentMagnet 
      "Permanent magnet synchronous machine with optional damper cage" 
        extends 
        Modelica.Electrical.Machines.Interfaces.PartialBasicInductionMachine(        is(start=zeros(m)));
      
        parameter Real effectiveStatorTurns = 1 
        "Effective number of stator turns";
        parameter Modelica.SIunits.Resistance Rs "Stator resistance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lssigma 
        "Stator leakage inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmd 
        "Main field inductance, d-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq 
        "Main field inductance, q-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
      
        // Rotor cage parameters
        parameter Boolean useDamperCage = true "Enable/disable damper cage" 
           annotation(Dialog(group="Damper cage"));
        parameter Modelica.SIunits.Inductance Lrsigmad 
        "Rotor leakage inductance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq 
        "Rotor leakage inductance, q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd 
        "Rotor resistance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq 
        "Rotor resistance , q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
      
        parameter Modelica.SIunits.Frequency fsNominal=50 
        "Nominal stator frequency" 
           annotation(Dialog(group="Excitation"));
        parameter Modelica.SIunits.Voltage V0=112.3 
        "No-load RMS voltage per phase @ fsNominal" 
           annotation(Dialog(group="Excitation"));
      
    protected 
        final parameter Modelica.SIunits.MagneticPotentialDifference V_mPM=
          (2/pi)*sqrt(2)*(m/2)*V0/effectiveStatorTurns/(Lmd/effectiveStatorTurns^2*2*pi*fsNominal) 
        "Equivalent excitation magnetic potential difference";
      
    public 
        Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap airGap(
          final p=p,
          final L0(d=2*Lmd/3/effectiveStatorTurns^2,
                   q=2*Lmq/3/effectiveStatorTurns^2)) 
          annotation (extent=[-10,-10; 10,10], rotation=270);
        Modelica_FundamentalWave.Components.Ground groundR 
        "Ground of rotor magnetic circuit" 
          annotation (extent=[-40,-30; -20,-10]);
        Modelica_FundamentalWave.Components.Short short if not useDamperCage 
        "Magnetic connection in case the damper cage is not present" 
          annotation (extent=[-10,-40; 10,-20], rotation=180);
        Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding 
        rotorCage(
        final R(d=Rrd, q=Rrq),
        final Lsigma(d=Lrsigmad, q=Lrsigmaq),
        final effectiveTurns=sqrt(3/2)*effectiveStatorTurns) if 
           useDamperCage 
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (extent=[10,-68; -10,-48]);
        Modelica_FundamentalWave.Sources.ConstantMagneticPotentialDifference 
        permanentMagnet(
          final V_m=Complex.Complex(V_mPM, 0)) 
        "Magnetic potential difference of permanent magnet" 
          annotation (extent=[20,-10; 40,-30], rotation=270);
        Components.SymmetricMultiPhaseWinding statorWinding(
          final m=m,
          final R=Rs,
          final Lsigma=Lssigma,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric stator winding including resistances and stray inductances" 
          annotation (extent=[-10,20; 10,40], rotation=270);
        Modelica_FundamentalWave.Components.Ground groundS 
        "Ground of stator magnetic circuit" 
          annotation (extent=[-40,0; -20,20]);
      equation 
      
        annotation (Diagram, Icon(
            Rectangle(extent=[-130,10; -100,-10],
                style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=2,
                rgbfillColor={0,255,0})),
            Rectangle(extent=[-100,10; -70,-10],
                style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=1,
                rgbfillColor={255,0,0})),
             Ellipse(extent=[-134,34; -66,-34], style(color=3, rgbcolor={0,0,255}))),
        Documentation(info="<html>
<p>
Resistances and stray inductances of the machine refer to the stator phases. The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. Only losses in stator and rotor resistance are taken into account. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ElectricalExcited\">SM_ElectricalExcited</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ReluctanceRotor\">SM_ReluctanceRotor</a>,
<a href=\"Modelica://Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnetDamperCage\">Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_PermanentMagnet</a>
</p>
</html>"));
        connect(airGap.support, internalSupport)         annotation (points=[-10,
            2.33651e-15; -10,0; -60,0; -60,-100; 20,-100],     style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.flange_a, inertiaRotor.flange_a)         annotation (
            points=[10,-1.33731e-15; 10,0; 60,0; 60,-1.72421e-15], style(color=0,
              rgbcolor={0,0,0}));
        connect(groundR.port_p, airGap.port_rn)         annotation (points=[-30,-10;
            -20,-10; -20,-10; -10,-10],   style(color=45, rgbcolor={255,128,0}));
        connect(airGap.port_rn, short.port_n)         annotation (points=[-10,-10;
            -10,-20; -10,-20; -10,-30],   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rn, rotorCage.port_n)            annotation (points=[-10,-10;
            -10,-22; -10,-22; -10,-34; -10,-58; -10,-58],            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rp, permanentMagnet.port_n) 
                                                    annotation (points=[10,-10;
            20,-10; 20,-10; 30,-10],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(permanentMagnet.port_p, short.port_p) 
                                                  annotation (points=[30,-30;
            10,-30],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(permanentMagnet.port_p, rotorCage.port_p) 
                                                 annotation (points=[30,-30; 30,
            -58; 10,-58],
                       style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_n, plug_sn) 
          annotation (points=[-10,40; -10,60; -60,60; -60,100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.port_n, airGap.port_sn)     annotation (points=[-10,20;
            -10,17.5; -10,17.5; -10,15; -10,10; -10,10],   style(color=45,
              rgbcolor={255,128,0}));
        connect(groundS.port_p,statorWinding. port_n) 
                                                  annotation (points=[-30,20;
            -10,20],
                   style(color=45, rgbcolor={255,128,0}));
        connect(statorWinding.port_p, airGap.port_sp)     annotation (points=[10,20;
            10,20; 10,10; 10,10],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0},
            fillPattern=1));
        connect(statorWinding.plug_p, plug_sp) annotation (points=[10,40; 10,60;
            60,60; 60,100],style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end SM_PermanentMagnet;
    
      model SM_ElectricalExcited 
      "Electrical excited synchronous machine with optional damper cage" 
        extends 
        Modelica.Electrical.Machines.Interfaces.PartialBasicInductionMachine(        is(start=zeros(m)));
      
        parameter Real effectiveStatorTurns = 1 
        "Effective number of stator turns";
        parameter Modelica.SIunits.Resistance Rs "Stator resistance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lssigma 
        "Stator leakage inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmd 
        "Main field inductance, d-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq 
        "Main field inductance, q-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
      
        // Rotor cage parameters
        parameter Boolean useDamperCage = true "Enable/disable damper cage" 
           annotation(Dialog(group="Damper cage"));
        parameter Modelica.SIunits.Inductance Lrsigmad 
        "Rotor leakage inductance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq 
        "Rotor leakage inductance, q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd 
        "Rotor resistance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq 
        "Rotor resistance , q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
      
        // Excitaiton parameters
        parameter Modelica.SIunits.Voltage VsNominal=100 
        "Nominal stator voltage" 
           annotation(Dialog(group="Excitation"));
        parameter Modelica.SIunits.Frequency fsNominal=50 
        "Nominal stator frequency" 
           annotation(Dialog(group="Excitation"));
        parameter Modelica.SIunits.Current Ie0=10 
        "no-load excitation current @ nominal voltage and frequency" 
           annotation(Dialog(group="Excitation"));
        parameter Modelica.SIunits.Resistance Re=2.5 
        "warm excitation resistance" 
           annotation(Dialog(group="Excitation"));
        parameter Real sigmae(min=0, max=1)=0.025 
        "stray fraction of total excitation inductance" 
           annotation(Dialog(group="Excitation"));
        output Modelica.SIunits.Voltage ve = pin_ep.v-pin_en.v 
        "Excitation voltage";
        output Modelica.SIunits.Current ie = pin_ep.i "Excitation current";
    protected 
        final parameter Real TurnsRatio=
          sqrt(2)*VsNominal/(2*pi*fsNominal*Lmd*Ie0) 
        "Stator current / excitation current";
        final parameter Modelica.SIunits.Inductance Lesigma=
           Lmd*TurnsRatio^2*3/2 * sigmae/(1-sigmae) 
        "Leakage inductance of the excitation winding";
      
    public 
        Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap airGap(
          final p=p,
          final L0(d=2*Lmd/3/effectiveStatorTurns^2,
                   q=2*Lmq/3/effectiveStatorTurns^2)) 
          annotation (extent=[-10,-10; 10,10], rotation=270);
        Modelica_FundamentalWave.Components.Ground groundS 
        "Ground of stator magnetic circuit" 
          annotation (extent=[-40,0; -20,20]);
        Modelica_FundamentalWave.Components.Ground groundR 
        "Ground of rotor magnetic circuit" 
          annotation (extent=[-40,-30; -20,-10]);
        Modelica_FundamentalWave.Components.Short short if not useDamperCage 
        "Magnetic connection in case the damper cage is not present" 
          annotation (extent=[-10,-70; 10,-50], rotation=180);
        Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding 
        rotorCage(
          final R(d=Rrd, q=Rrq),
          final Lsigma(d=Lrsigmad, q=Lrsigmaq),
        final effectiveTurns=sqrt(3/2)*effectiveStatorTurns) if 
           useDamperCage 
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (extent=[10,-100; -10,-80]);
        Modelica_FundamentalWave.Machines.Components.SinglePhaseWinding 
        excitationWinding(
          final windingAngle=0,
          final R=Re,
          final Lsigma=Lesigma,
          final effectiveTurns=effectiveStatorTurns*TurnsRatio*m/2) 
        "Excitation winding including resistance and stray inductance" 
          annotation (extent=[-30,-50; -10,-30]);
        Modelica.Electrical.Analog.Interfaces.PositivePin pin_ep 
          annotation (extent=[-110,70; -90,50]);
        Modelica.Electrical.Analog.Interfaces.NegativePin pin_en 
          annotation (extent=[-90,-50; -110,-70]);
        Modelica_FundamentalWave.Machines.Components.SymmetricMultiPhaseWinding
        statorWinding(
          final m=m,
          final R=Rs,
          final Lsigma=Lssigma,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric stator winding including resistances and stray inductances" 
          annotation (extent=[-10,20; 10,40], rotation=270);
      equation 
      
        annotation (Diagram, Icon(
             Ellipse(extent=[-134,34; -66,-34], style(color=3, rgbcolor={0,0,255})),
            Line(points=[-100,50; -100,20; -130,20; -130,-4], style(color=3,
                  rgbcolor={0,0,255})),
            Line(points=[-130,-4; -129,1; -125,5; -120,6; -115,5; -111,1; -110,-4],
                style(color=3, rgbcolor={0,0,255})),
            Line(points=[-110,-4; -109,1; -105,5; -100,6; -95,5; -91,1; -90,-4],
                style(color=3, rgbcolor={0,0,255})),
            Line(points=[-90,-4; -89,1; -85,5; -80,6; -75,5; -71,1; -70,-4],style(
                  color=3, rgbcolor={0,0,255})),
            Line(points=[-100,-50; -100,-20; -70,-20; -70,-2], style(color=3,
                  rgbcolor={0,0,255}))),
        Documentation(info="<html>
<p>
The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. Only losses in stator and rotor resistance are taken into account. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_PermanentMagnet\">SM_PermanentMagnet</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ReluctanceRotor\">SM_ReluctanceRotor</a>,
<a href=\"Modelica://Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcitedDamperCage\">Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ElectricalExcited</a>
</p>
</html>"));
        connect(airGap.support, internalSupport)         annotation (points=[-10,
            2.33651e-15; -10,0; -60,0; -60,-100; 20,-100],     style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.flange_a, inertiaRotor.flange_a)         annotation (
            points=[10,-1.33731e-15; 10,0; 60,0; 60,-1.72421e-15], style(color=0,
              rgbcolor={0,0,0}));
        connect(groundR.port_p, airGap.port_rn)         annotation (points=[-30,-10;
            -20,-10; -20,-10; -10,-10],   style(color=45, rgbcolor={255,128,0}));
        connect(airGap.port_rp, short.port_p) annotation (points=[10,-10; 10,
            -10; 10,-60; 10,-60],
                            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rp, rotorCage.port_p) 
                                             annotation (points=[10,-10; 10,-90],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(short.port_n, rotorCage.port_n) 
                                           annotation (points=[-10,-60; -10,-75;
            -10,-90; -10,-90],
                             style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rn, excitationWinding.port_p) annotation (points=[-10,-10;
            -10,-20; -10,-20; -10,-30],   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(excitationWinding.port_n, short.port_n) annotation (points=[-10,-50;
              -10,-60], style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(excitationWinding.port_n, rotorCage.port_n) 
                                                       annotation (points=[-10,-50;
            -10,-70; -10,-90; -10,-90],   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(pin_en, excitationWinding.pin_n) annotation (points=[-100,-60; -30,
              -60; -30,-50], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_n, plug_sn) 
          annotation (points=[-10,40; -10,60; -60,60; -60,100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_p, plug_sp) annotation (points=[10,40; 10,60;
            60,60; 60,100],style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.port_n, airGap.port_sn)     annotation (points=[-10,20;
            -10,17.5; -10,17.5; -10,15; -10,10; -10,10],   style(color=45,
              rgbcolor={255,128,0}));
        connect(statorWinding.port_p, airGap.port_sp)     annotation (points=[10,20;
            10,20; 10,10; 10,10],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0},
            fillPattern=1));
        connect(groundS.port_p,statorWinding. port_n) 
                                                  annotation (points=[-30,20;
            -10,20],
                   style(color=45, rgbcolor={255,128,0}));
        connect(excitationWinding.pin_p, pin_ep) annotation (points=[-30,-30; -80,-30;
              -80,60; -100,60], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
      end SM_ElectricalExcited;
    
      model SM_ReluctanceRotor "Reluctance machine with optional damper cage" 
        extends 
        Modelica.Electrical.Machines.Interfaces.PartialBasicInductionMachine(        is(start=zeros(m)));
      
        parameter Real effectiveStatorTurns = 1 
        "Effective number of stator turns";
        parameter Modelica.SIunits.Resistance Rs "Stator resistance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lssigma 
        "Stator leakage inductance" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmd 
        "Main field inductance, d-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
        parameter Modelica.SIunits.Inductance Lmq 
        "Main field inductance, q-axis" 
           annotation(Dialog(group="Nominal resistances and inductances"));
      
        // Rotor cage parameters
        parameter Boolean useDamperCage = true "Enable/disable damper cage" 
           annotation(Dialog(group="Damper cage"));
        parameter Modelica.SIunits.Inductance Lrsigmad 
        "Rotor leakage inductance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Inductance Lrsigmaq 
        "Rotor leakage inductance, q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrd 
        "Rotor resistance, d-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
        parameter Modelica.SIunits.Resistance Rrq 
        "Rotor resistance , q-axis, w.r.t. stator side" 
           annotation(Dialog(group="Damper cage",enable=useDamperCage));
      
        Modelica_FundamentalWave.Machines.Components.RotorSaliencyAirGap airGap(
          final p=p,
          final L0(d=2*Lmd/3/effectiveStatorTurns^2,
                   q=2*Lmq/3/effectiveStatorTurns^2)) 
          annotation (extent=[-10,-10; 10,10], rotation=270);
        Modelica_FundamentalWave.Components.Ground groundS 
        "Ground of stator magnetic circuit" 
          annotation (extent=[-40,0; -20,20]);
        Modelica_FundamentalWave.Components.Ground groundR 
        "Ground of rotor magnetic circuit" 
          annotation (extent=[-40,-30; -20,-10]);
        Modelica_FundamentalWave.Components.Short short if not useDamperCage 
        "Magnetic connection in case the damper cage is not present" 
          annotation (extent=[-10,-40; 10,-20], rotation=180);
        Modelica_FundamentalWave.Machines.Components.SaliencyCageWinding 
        rotorCage(
          final R(d=Rrd, q=Rrq),
          final Lsigma(d=Lrsigmad, q=Lrsigmaq),
        final effectiveTurns=sqrt(3/2)*effectiveStatorTurns) if 
           useDamperCage 
        "Symmetric rotor cage winding including resistances and stray inductances"
          annotation (extent=[10,-70; -10,-50]);
        Components.SymmetricMultiPhaseWinding statorWinding(
          final m=m,
          final R=Rs,
          final Lsigma=Lssigma,
          final effectiveTurns=effectiveStatorTurns) 
        "Symmetric stator winding including resistances and stray inductances" 
          annotation (extent=[-10,20; 10,40], rotation=270);
      equation 
      
        annotation (Diagram, Icon(
            Rectangle(extent=[-130,10; -100,-10], style(color=0, rgbcolor={0,0,0})),
            Rectangle(extent=[-100,10; -70,-10], style(color=0, rgbcolor={0,0,0})),
             Ellipse(extent=[-134,34; -66,-34], style(color=3, rgbcolor={0,0,255}))),
        Documentation(info="<html>
<p>
The symmetry of the stator is assumed. For rotor asymmetries can be taken into account by different resistances and stray inductances in the d- and q-axis. Only losses in stator and rotor resistance are taken into account. 
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_PermanentMagnet\">SM_PermanentMagnet</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Machines.SynchronousInductionMachines.SM_ElectricalExcited\">SM_ElectricalExcited</a>,
<a href=\"Modelica://Modelica.Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotorDamperCage\">Electrical.Machines.BasicMachines.SynchronousInductionMachines.SM_ReluctanceRotor</a>
</p>
</html>"));
        connect(airGap.support, internalSupport)         annotation (points=[-10,
            2.33651e-15; -10,0; -80,0; -80,-100; 20,-100],     style(color=0,
              rgbcolor={0,0,0}));
        connect(airGap.flange_a, inertiaRotor.flange_a)         annotation (
            points=[10,-1.33731e-15; 10,0; 60,0; 60,-1.72421e-15], style(color=0,
              rgbcolor={0,0,0}));
        connect(groundR.port_p, airGap.port_rn)         annotation (points=[-30,-10;
            -20,-10; -20,-10; -10,-10],   style(color=45, rgbcolor={255,128,0}));
        connect(airGap.port_rn, short.port_n)         annotation (points=[-10,-10;
            -10,-20; -10,-20; -10,-30],   style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rn, rotorCage.port_n)            annotation (points=[-10,-10;
            -10,-22.5; -10,-22.5; -10,-35; -10,-60; -10,-60],        style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rp, short.port_p) annotation (points=[10,-10; 10,
            -10; 10,-30; 10,-30],
                            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(airGap.port_rp, rotorCage.port_p) 
                                             annotation (points=[10,-10; 10,-60],
            style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_n, plug_sn) 
          annotation (points=[-10,40; -10,60; -60,60; -60,100], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(statorWinding.plug_p, plug_sp) annotation (points=[10,40; 10,60;
            60,60; 60,100],style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1));
        connect(groundS.port_p,statorWinding. port_n) 
                                                  annotation (points=[-30,20;
            -10,20],
                   style(color=45, rgbcolor={255,128,0}));
        connect(statorWinding.port_n, airGap.port_sn)     annotation (points=[-10,20;
            -10,17.5; -10,17.5; -10,15; -10,10; -10,10],   style(color=45,
              rgbcolor={255,128,0}));
        connect(statorWinding.port_p, airGap.port_sp)     annotation (points=[10,20;
            10,20; 10,10; 10,10],style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0},
            fillPattern=1));
      end SM_ReluctanceRotor;
    annotation (Documentation(info="<html>
<p>This package contains various synchronous induction machine models.</p>
</html>"));
    end SynchronousInductionMachines;
  annotation (Documentation(info="<html>
<p>
This package contains components for electric machines and electric machine models.
</p>
</html>"));
  end Machines;


  package Interfaces "Interfaces and partial models" 
    connector MagneticPort "Complex magnetic port" 
    
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      flow Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
    annotation (Documentation(info="<html>
<p>
The potential quantity of the magnetic port is the complex magnetic potential difference <img src=\"../Images/V_m.png\">. The corresponding flow quantity is the magnetic flux <img src=\"../Images/Phi.png\">.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">PositiveMagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">NegativeMagneticPort</a>
</p>

</html>"));
    end MagneticPort;
  
    connector PositiveMagneticPort "Positive complex magnetic port" 
    
      extends Modelica_FundamentalWave.Interfaces.MagneticPort;
      annotation (defaultName="port_p",
        Icon(Ellipse(extent=[-100,100; 100,-100], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=45,
              rgbfillColor={255,128,0}))),
                                Diagram(
                      Text(
            extent=[-60,100; -60,60],
            string="%name",
            style(color=45, rgbcolor={255,128,0})), Ellipse(extent=[-50,50; 50,
                -50], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=45,
              rgbfillColor={255,128,0}))),
      Documentation(info="<html>
<p>
Positive magnetic <a href=\"Modelica://Modelica_FundamentalWave.Interfaces.MagneticPort\">port</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.MagneticPort\">MagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">NegativeMagneticPort</a>
</p>

</html>"));
    end PositiveMagneticPort;
  
    connector NegativeMagneticPort "Negative complex magnetic port" 
    
      extends Modelica_FundamentalWave.Interfaces.MagneticPort;
    
      annotation (defaultName="port_n",
        Icon(Ellipse(extent=[-100,100; 100,-100], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=7,
              rgbfillColor={255,255,255}))),
                                Diagram(
                      Text(
            extent=[-60,100; -60,60],
            string="%name",
            style(color=45, rgbcolor={255,128,0})), Ellipse(extent=[-50,50; 50,
                -50], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1))),
      Documentation(info="<html>
<p>
Negative magnetic <a href=\"Modelica://Modelica_FundamentalWave.Interfaces.MagneticPort\">port</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.MagneticPort\">MagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">PositiveMagneticPort</a>
</p>
</html>"));
    end NegativeMagneticPort;
  
    partial model PartialTwoPortElementary 
    "Two magnetic ports for textual modeling" 
    
      Modelica_FundamentalWave.Interfaces.PositiveMagneticPort port_p 
      "Positive complex magnetic port" 
        annotation (extent=[-110,-10; -90,10]);
      Modelica_FundamentalWave.Interfaces.NegativeMagneticPort port_n 
      "Negative complex magnetic port" 
        annotation (extent=[90,-10; 110,10]);
    
    annotation (Documentation(info="<html>
<p>
This magnetic two port element only consists of a  
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">positive</a> and a
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">negative magnetic port</a>. 
This model is mainly used to extend from in order build more complex - equation based - models.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">PositiveMagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">NegativeMagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PartialTwoPort\">PartialTwoPort</a>
</p>
</html>"));
    end PartialTwoPortElementary;
  
    partial model PartialTwoPort "Two magnetic ports for textual modeling" 
    
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
    equation 
      // Flux into positive port 
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
    
      // Magneto motive force
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Local flux balance
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
    annotation (Documentation(info="<html>
<p>
This magnetic two port element consists of a  
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">positive</a> and a
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">negative magnetic port</a> and
considers the flux balance of the two ports. Additionally the magnetic potential difference (of the positive and the negative port) and the magnetic flux (into the positive magnetic port) are defined. This model is mainly to used to extend from in order build more complex - graphical - models.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PositiveMagneticPort\">PositiveMagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.NegativeMagneticPort\">NegativeMagneticPort</a>,
<a href=\"Modelica://Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary\">PartialTwoPortElementary</a>
</p></html>"));
    end PartialTwoPort;
  
  annotation (Documentation(info="<html>
<p>
This package contains interface definitions of the magnetic ports as well as partial models.
</p>
</html>"));
  end Interfaces;


  package Sources "Sources" 
    model ConstantMagneticPotentialDifference 
    "Source with constant magnetic potential difference" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
      parameter Complex.ComplexMagneticPotentialDifference V_m=
        Complex.Complex(re=1, im=0);
    
      Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
    equation 
      // Flux into positive port 
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
    
      // Magneto motive force
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Local flux balance
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      annotation (Diagram, Icon(
          Text(
            extent=[-80,-20; -80,-40],
            string="+",
            style(color=45, rgbcolor={255,128,0})),
          Text(
            extent=[80,-20; 80,-40],
            string="-",
            style(color=45, rgbcolor={255,128,0})),
          Ellipse(extent=[-50,-50; 50,50], style(
              color=45,
              rgbcolor={255,127,0},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,0; 50,0],   style(color=45)),
          Line(points=[-50,0; -100,0],   style(color=45)),
          Line(points=[-50,0; 50,0],   style(color=45))),
        Documentation(info="<html>
<p>
Source of constant magneto motive force.
</p>
</html>"));
    end ConstantMagneticPotentialDifference;
  
    model SignalMagneticPotentialDifference 
    "Source of magnetic potential difference with signal input" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
      Complex.ComplexInput V_m 
      "Complex signal input of magnetic potential difference" 
        annotation (extent=[-20,80; 20,120], rotation=270);
      Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
    equation 
      // Flux into positive port 
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
    
      // Magneto motive force
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Local flux balance
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      annotation (Diagram, Icon(
          Text(
            extent=[80,-20; 80,-40],
            string="-",
            style(color=45, rgbcolor={255,128,0})),
          Ellipse(extent=[-50,-50; 50,50], style(
              color=45,
              rgbcolor={255,127,0},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,0; 50,0],   style(color=45)),
          Line(points=[-50,0; -100,0],   style(color=45)),
          Line(points=[-50,0; 50,0],   style(color=45)),
          Line(points=[0,100; 0,50],       style(color=45))),
      Documentation(info="<html>
<p>
Source of magneto motive force with complex signal input.
</p>
</html>"));
    end SignalMagneticPotentialDifference;
  
    model ConstantFlux "Source of constant magnetic flux" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      parameter Complex.ComplexMagneticFlux Phi=
        Complex.Complex(re=1, im=0);
    
    equation 
      // Flux into positive port 
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
    
      // Magneto motive force
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Local flux balance
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      annotation (Diagram, Icon(
          Ellipse(extent=[-50,-50; 50,50], style(
              color=45,
              rgbcolor={255,127,0},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,0; 50,0],   style(color=45)),
          Line(points=[-50,0; -100,0],   style(color=45)),
          Line(points=[0,50; 0,-50],   style(color=45)),
        Polygon(points=[80,0; 60,6; 60,-6; 80,0],    style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0}))),
        Documentation(info="<html>
<p>
Source of constant magnetic flux.
</p>
</html>"));
    end ConstantFlux;
  
    model SignalFlux "Source of constant magnetic flux" 
      extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
      Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
      Complex.ComplexInput Phi "Complex signal input of magnetic flux" 
        annotation (extent=[-20,80; 20,120], rotation=270);
    
    equation 
      // Flux into positive port 
      port_p.V_m.re - port_n.V_m.re = V_m.re;
      port_p.V_m.im - port_n.V_m.im = V_m.im;
    
      // Magneto motive force
      port_p.Phi.re = Phi.re;
      port_p.Phi.im = Phi.im;
    
      // Local flux balance
      port_p.Phi.re + port_n.Phi.re = 0;
      port_p.Phi.im + port_n.Phi.im = 0;
    
      annotation (Diagram, Icon(
          Ellipse(extent=[-50,-50; 50,50], style(
              color=45,
              rgbcolor={255,127,0},
              fillColor=7,
              rgbfillColor={255,255,255},
              fillPattern=1)),
          Line(points=[100,0; 50,0],   style(color=45)),
          Line(points=[-50,0; -100,0],   style(color=45)),
          Line(points=[0,50; 0,-50],   style(color=45)),
        Polygon(points=[80,0; 60,6; 60,-6; 80,0], style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0})),
          Line(points=[0,100; 0,50],   style(color=45))),
        Documentation(info="<html>
<p>
Source of magnetic flux with complex signal input.
</p>
</html>"));
    end SignalFlux;
  annotation (Documentation(info="<html>
<p>This package provides ources of magnetic potential difference and magnetic flux.</p>
</html>"));
  end Sources;


  package Sensors "Sensors to measure variables in magnetic networks" 
  
  model MagneticPotentialDifferenceSensor 
    "Sensor to measure magnetic potential difference" 
    extends Modelica.Icons.RotationalSensor;
    extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
    Complex.ComplexOutput V_m 
      "Complex magnetic potential difference between port_p and port_n as output signal"
       annotation (extent=[-10, -90; 10, -110],
        rotation=90);
    Complex.ComplexMagneticFlux Phi "Complex magnetic flux";
    
  equation 
    // Flux into positive port 
    port_p.V_m.re - port_n.V_m.re = V_m.re;
    port_p.V_m.im - port_n.V_m.im = V_m.im;
    
    // Magneto motive force
    port_p.Phi.re = Phi.re;
    port_p.Phi.im = Phi.im;
    
    // Local flux balance
    port_p.Phi.re + port_n.Phi.re = 0;
    port_p.Phi.im + port_n.Phi.im = 0;
    
    // No magnetic flux through sensor
    Phi.re = 0;
    Phi.im = 0;
    
    annotation (
      Coordsys(
        extent=[-100, -100; 100, 100],
        grid=[2,2],
        component=[20, 20],
          scale=0),
      Window(
        x=0.28,
        y=0.29,
        width=0.6,
        height=0.6),
      Icon(Text(
          extent=[-52,1; 48,-57],
          string="V_m",
          style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0})),
        Line(points=[-70,0; -90,0], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0})),
        Line(points=[70,0; 90,0], style(
            color=0,
            rgbcolor={0,0,0},
            fillColor=0,
            rgbfillColor={0,0,0})),
        Line(points=[0,-90; 0,-70]),
        Text(extent=[-140,120; 140,80],   string="%name")),
      Diagram(
        Line(points=[-70,0; -100,0],  style(color=0)),
        Line(points=[70,0; 100,0],  style(color=0)),
        Line(points=[0,-90; 0,-70])),
      Documentation(info="<html>
<p>Sensor for magnetic potential difference.</p>
</html>"));
  end MagneticPotentialDifferenceSensor;
  
  model MagneticFluxSensor "Sensor to measure magnetic flux" 
    extends Modelica.Icons.RotationalSensor;
    extends Modelica_FundamentalWave.Interfaces.PartialTwoPortElementary;
    
    Complex.ComplexMagneticPotentialDifference V_m 
      "Complex magnetic potential difference";
    Complex.ComplexOutput Phi 
      "Complex magnetic flux from por_ p to port_n as output signal" 
       annotation (extent=[-10, -90; 10, -110],
        rotation=90);
    
  equation 
    // Flux into positive port 
    port_p.V_m.re - port_n.V_m.re = V_m.re;
    port_p.V_m.im - port_n.V_m.im = V_m.im;
    
    // Magneto motive force
    port_p.Phi.re = Phi.re;
    port_p.Phi.im = Phi.im;
    
    // Local flux balance
    port_p.Phi.re + port_n.Phi.re = 0;
    port_p.Phi.im + port_n.Phi.im = 0;
    
    // No magnetic potential difference at sensor
    V_m.re = 0;
    V_m.im = 0;
    
    annotation (
      Coordsys(
        extent=[-100, -100; 100, 100],
        grid=[2,2],
        component=[20, 20],
          scale=0),
      Window(
        x=0.23,
        y=0.07,
        width=0.6,
        height=0.6),
      Icon(Text(
          extent=[-29, -11; 30, -70],
          style(color=0),
          string="Phi"),
        Line(points=[-70,0; -90,0],   style(color=0)),
        Text(extent=[-140,120; 140,80],   string="%name"),
        Line(points=[70,0; 90,0],   style(color=0)),
        Line(points=[0,-90; 0,-70])),
      Diagram(
        Line(points=[-70,0; -100,0],  style(color=0)),
        Line(points=[70,0; 100,0],  style(color=0)),
        Line(points=[0,-90; 0,-70])),
      Documentation(info="<html>
<p>Sensor for magnetic flux.</p>
</html>"));
  end MagneticFluxSensor;
    annotation (Documentation(info="<html>
<p>
This package provides sensors for the magnetic potential difference and the magnetic flux in magnetic circuit.
</p>
</html>"));
  end Sensors;


  package Math "Complex and salient definitions" 
    record Salient "Base record of saliency with d and q component" 
      Real d "Component of d (direct) axis, aligned to real part";
      Real q "Component of q (quadrature) axis, aligned to imaginary part";
    annotation (Documentation(info="<html>
<p>
Definition of saliency with respect to the orthogonal d- and q-axis. Saliency, however, refers to different properties in d- and q-axis and thus considers the anisotropic behavior.
</p>

<h4>See also</h4>
<p>

</p>
</html>"));
    end Salient;
  
    package SIunits "Redeclared complex and salient types" 
      record SalientReluctance = Salient (
        redeclare Modelica.SIunits.Reluctance d,
        redeclare Modelica.SIunits.Reluctance q) "Salient reluctance";
      record SalientInductance = Salient (
        redeclare Modelica.SIunits.Inductance d,
        redeclare Modelica.SIunits.Inductance q) "Salient inductance";
      record SalientResistance = Salient (
        redeclare Modelica.SIunits.Resistance d,
        redeclare Modelica.SIunits.Resistance q) "Salient resistance";
      record SalientCurrent = Salient (
        redeclare Modelica.SIunits.Current d,
        redeclare Modelica.SIunits.Current q) "Salient current";
      record SalientVoltage = Salient (
        redeclare Modelica.SIunits.Voltage d,
        redeclare Modelica.SIunits.Voltage q) "Salient voltage";
    annotation (Documentation(info="<html>
<p>
Units for quantities with saliency until a better solution is provided in the Modelica specficiation
</p>
</html>"));
    end SIunits;
  
    function symmetricWindingAngle "Winding angles of symmetric phase winding" 
    
    import Modelica.Constants.pi;
    
      input Integer m "Number of phases";
      output Modelica.SIunits.Angle windingAngle[m] "Angle of wining axis";
    
    algorithm 
      if mod(m,2) == 0 then
        // Even number of phases
        for k in 1:integer(m/2) loop
          windingAngle[k] :=(k - 1)*4*pi/m;
          windingAngle[k+integer(m/2)] := windingAngle[k] + 2*pi/m;
        end for;
      else
        // Odd number of phases 
        windingAngle :={(k - 1)*2*pi/m for k in 1:m};
      end if;
    annotation (Documentation(info="<html>
<p>  
This function determines the winding angles of a symmetrical winding with <img src=\"../Images/m.png\"> phases. For an odd number of phases the difference of the windings angles of two adjacent phases is 
<img src=\"../Images/2pi_over_m.png\">. In case of an even number of phases aligned winding angles are not modeled since they do not add any information. Instead the <img src=\"../Images/m.png\"> windings are divided into two different groups. The first group refers to the indices <img src=\"../Images/k_le_m_over_2.png\">. The second group covers the indices <img src=\"../Images/k_gt_m_over_2.png\">. The difference of the windings angles of two adjacent phases - of both the first and the second group, respectively - is <img src=\"../Images/4pi_over_m.png\">. The phase shift of the two groups <img src=\"../Images/pi_over_2m.png\">.
</p>
</html>"));
    end symmetricWindingAngle;
  end Math;


  annotation (uses(Modelica(version="2.2.2"),
    Complex(version="0.2.0")),
    version="0.X.0",versionDate="2009-11-XX",
    Documentation(revisions="<html>
<table border=\"1\" rules=\"groups\">
<thead>
<tr><td>Version</td> <td>Revision</td> <td>Date</td> <td>Authors</td> <td>Comments</td></tr>
</thead>
<tbody>
 
<tr><td>0.1.0</td> <td>215</td> <td>2009-07-22</td>  <td>C. Kral</td>  <td>First version based on the concept of the FluxTubes library and the Magnetics library of Michael Beuschel</td></tr>
<tr><td>0.2.0</td> <td>234</td> <td>2009-10-20</td>  <td>C. Kral</td>  <td>Added idle model and converter model for FluxTubes</td></tr>
<tr><td>0.3.0</td> <td>XXX</td> <td>2009-10-28</td>  <td>C. Kral<br>A.&nbsp;Haumer</td>  <td>Renamed number of turns and winding angles</td></tr>
<tr><td>0.4.0</td> <td>XXX</td> <td>2009-10-29</td>  <td>C. Kral<br>A.&nbsp;Haumer</td>  <td>Corrected bug in magnetic potential calculation</td></tr>
</tbody>
</table>
</html>"));
end Modelica_FundamentalWave;
