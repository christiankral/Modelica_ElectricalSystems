within ;
package Modelica_QuasiStationary "Library for quasi-stationary electrical singlephase and multiphase AC simulation"
  package UsersGuide "User's guide" 
    annotation (DocumentationClass=true);
    package Overview "Overview" 
      annotation (Documentation(info="<html>
<p>
The <a href=\"Modelica://Modelica_QuasiStationary\">Modelica_QuasiStationary</a>
library addresses the analysis of electrical circuits with purely sinusoidal 
voltages and currents. The main characteristics of the library are:
</p>
 
<ul> 
  <li>Only pure sinusoidal voltages and currents are taken into account. 
      Higher harmonic voltages and currents are not considered.</li>
  <li>Any electrical transient effects are negelcted.</li>
  <li>The electrical components of this library are strictly linear.</li>
  <li>The angular frequency <eq>omega</eq> of the voltages and currents of 
      a circuit are
      determined from a reference angle <eq>gamma</eq> by means of  
      <eq>omega = der(gamma)</eq>. </li>
  <li>The reference angle <eq>gamma</eq> is not a global quantity 
      since it propagated through the connector. 
      Therefore, independent circuits of different frequencies can be modeled in one model.</li>
  <li>The connectors contain the real and the imaginary part of the voltage and the current 
      <a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Phasor\">RMS phasors</a></li>
 
</ul>
 
<p>
The main intention of this libirary is the modeling of quasi stationary behavior 
of single and multi phase 
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.ACCircuit\">AC circuits</a>
with fixed and variable frequency. Quasi stationary theory and applications can be 
found in 
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Dorf1993</a>],
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Burton1994</a>],
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Landolt1936</a>],
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Philippow1967</a>],
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Weyh1967</a>],
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Vaske1973</a>].
</p>
 
 
</html>"));
      class Introduction "Introduction to phasors" 
      
        annotation (Documentation(info="<html>
 
<p>
The purely sinusoidal voltage 
</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/Introduction/img1.png\"
 ALT=\"
v=\\sqrt{2}V_{\\mathrm{RMS}}\\cos(\\omega t+\\varphi_{v})\">
</p>
 
<p>
in the time domain can be represented by a complex 
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Glossar\">rms</a> phasor
</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/Introduction/img2.png\"
 ALT=\"
\\underline{v}=V_{\\mathrm{RMS}}e^{j\\varphi_{v}}.\">
</p>
 
<p>For these quasi stationary
phasor the following relationship applies:</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/Introduction/img3.png\"
 ALT=\"\\begin{displaymath}
v=\\mathrm{Re}(\\sqrt{2}\\underline{v}e^{j\\omega t})\\end{displaymath}\">
</p>
 
<p>
This equation is also illustrated in Fig. 1.
</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/Introduction/phasor_voltage.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 1: Relationship between voltage phasor and time domain voltage</caption>
</table>
 
<p>
From the above equation it is obvious that for <i>t</i> = 0
the time domain voltage is <i>v</i> = cos(<i>&phi;<sub>v</sub></i>).  
The complex representation of the phasor corresponds with this instance, too, since
the phasor is leading the real axis by the angle <i>&phi;<sub>v</sub></i>.
</p>
 
<p>
The explanation given for sinusoidal voltages can certainly also be applied 
to sinusoidal currents.</p>
 
<h4>See also</h4>
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.ACCircuit\">
          AC circuit</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.Power\">
          Power</a>
 
 
</html>"));
      end Introduction;
    
      class ACCircuit "AC circuit" 
      
        annotation (Documentation(info="<html>
<p>
A simple 
<a href=\"Modelica://Modelica_QuasiStationary.Examples.SeriesResonance\">
          example</a> of a series connection of a resistor, an inductor and a capacitor
as depicted in Fig. 1 should be explained in the following. For various frequencies, 
the voltage drops across the resistor, the inductor and the capacitor should be determined. 
</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/ACCircuit/resonance_circuit.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 1: Series AC circuit of a resistor and an inductor at variable frequency</caption>
</table>
 
<p>
The voltage drop across the resistor 
</p>
 
<p> 
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/ACCircuit/img1.png\"
 ALT=\"
\\underline{v}_{r}=R\\underline{i}\">
</p>
 
<p>
and the inductor
</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/ACCircuit/img2.png\"
 ALT=\"
\\underline{v}_{l}=j\\omega L\\underline{i}\">
</p>
 
<p>
and the capacitor
</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/ACCircuit/img3.png\"
 ALT=\"
\\underline{v}_{l}=j\\omega L\\underline{i}\">
</p>
 
<p>
add up to the total voltage
</p>
 
<p>
<IMG
 BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/ACCircuit/img4.png\"
 ALT=\"
\\underline{v}=\\underline{v}_{r}+\\underline{v}_{l}\">
</p>
 
<p>
as illustraed in the phasor diagram of Fig. 2. 
</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/ACCircuit/phasor_diagram.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 2: Phasor diagram of a resistor and inductance series connection</caption>
</table>
 
<p>Due to the series connection of the resistor, inductor and capacitor, the three currents are all equal:</p>
 
<IMG BORDER=\"0\"
 SRC=\"../Images/UsersGuide/Overview/ACCircuit/img5.png\">
</p>
 
 
<h4>See also</h4>
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.Introduction\">
          Introduction</a>,
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.Power\">
          Power</a>
 
</html>"));
      end ACCircuit;
    
      class Power "Real and reactive power" 
        annotation (Documentation(info="<html>
 
<p>For periodic waveforms, the average value of the instantaneous power is <b>real power</b> <i>P</i>. 
<b>Reactive power</b> <i>Q</i> is a term 
associated with inductors and capacitors. For pure inductors and capacitors, real power is equal to zero. 
Yet, there is instantaneous power exchanged with connecting network. 
</p> 
 
The 
<a href=\"Modelica://Modelica_QuasiStationary.Examples.SeriesResonance\">
          series resonance circuit</a> which was also adressed in the
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.ACCircuit\">
          AC circuit</a>
will be investigated.
 
<h5>Power of a resistor</h5>
 
<p>
The instantaneous voltage and current are in phase:
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/v_r.png\"> <br>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/i_r.png\"> 
</p>
 
<p>
Therefore, the instantaneous power is 
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/power_r.png\"> 
</p>
 
<p>A graphical representation of these equations is depicted in Fig. 1</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/Power/power_resistor.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 1: Instantaneous voltage, current of power of a resistor</caption>
</table>
 
<p>Real power of the resistor is the average of instantaneous power:</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/p_r.png\"> 
</p>
 
 
<h5>Power of an inductor</h5>
 
<p>
The instantaneous voltage leads the current by a quarter of the period:
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/v_l.png\"> <br>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/i_l.png\"> 
</p>
 
<p>
Therefore, the instantaneous power is 
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/power_l.png\"> 
</p>
 
<p>A graphical representation of these equations is depicted in Fig. 2</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/Power/power_inductor.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 2: Instantaneous voltage, current of power of an inductor</caption>
</table>
 
<p>Reqactive power of the inductor is:</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/q_l.png\"> 
</p>
 
<h5>Power of a capacitor</h5>
 
<p>
The instantaneous voltage lags the current by a quarter of the period:
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/v_c.png\"> <br>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/i_c.png\"> 
</p>
 
<p>
Therefore, the instantaneous power is 
</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/power_c.png\"> 
</p>
 
<p>A graphical representation of these equations is depicted in Fig. 3</p>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td>
      <img src=\"../Images/UsersGuide/Overview/Power/power_capacitor.png\">
    </td>
  </tr>
  <caption align=\"bottom\">Fig. 3: Instantaneous voltage, current of power of a capacitor</caption>
</table>
 
<p>Reqactive power of the capacitor is:</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/q_c.png\"> 
</p>
 
<h5>Complex apparent power</h5>
 
<p>For an arbitrary component with two pins, real and reactive power can be determined by the complex phasors:</p>
<p>
<IMG BORDER=\"0\"  SRC=\"../Images/UsersGuide/Overview/Power/s.png\"> 
</p>
 
<p>
In this equation <sup>*</sup> represents the conjugate complex operator
</p>
 
<h4>See also</h4>
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.Indtroduction\">
          Introduction</a>,
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.Overview.ACCircuit\">
          AC circuit</a>
</html>"));
      end Power;
    end Overview;
  
    class ReleaseNotes "Release notes" 
      annotation (Documentation(revisions="<html>
<h4>Version 1.0.0</h4>
 
<p>First official release</p>
</html>"));
    end ReleaseNotes;
  
    class References 
      annotation (Documentation(info="<html>
 
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
    <tr>
      <td valign=\"top\">[Dorf1993]</td>
      <td valign=\"top\">R. C. Dorf
        <i>The Electrical Engineering</i>,
        VDE, 1993.</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Boas1966]</td>
      <td valign=\"top\">M. L. Boas
        <i>Mathematical Methods in the Physical Sciences</i>,
        J. Wiley & Sons, New York, 1966.</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Burton1994]</td>
      <td valign=\"top\">T. Burton
        <i>Introduction to Dynamic Systems Analysis</i>,
        McGraw Hill, New York, 1994.</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Landolt1936]</td>
      <td valign=\"top\">M. Landolt
        <i>Komplexe Zahlen und Zeiger in der Wechselstromlehre</i>,
        Springer Verlag, Berlin, 1936</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Philippow1967]</td>
      <td valign=\"top\">E. Philippow
        <i>Grundlagen der Elektrotechnik</i>,
       Akademischer Verlag, Leipzig, 1967.</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Weyh1967]</td>
      <td valign=\"top\">Weyh and Benzinger
        <i>Die Grundlagen der Wechselstromlehre</i>,
       R. Oldenbourg Verlag, 1967.</td>
    </tr>
 
    <tr>
      <td valign=\"top\">[Vaske1973]</td>
      <td valign=\"top\">P. Vaske
        <i>Berechnung von Drehstromschaltungen</i>,
       B.G. Teubner Verlag, 1973.</td>
    </tr>
 
</table>
 
</html>"));
    end References;
  
    class Contact 
      annotation (Documentation(info="<html>
 
<p>This package is developed an maintained by the following contributors</p>
 
  <table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
    <tr>
      <th></th>
      <th>Name</th>
      <th>Affiliation</th>
    </tr>
    <tr>
      <td valign=\"top\">Library officer</td>
      <td valign=\"top\">
      <a href=\"mailto:a.haumer@haumer.at\">A. Haumer</a>
      </td>
      <td valign=\"top\">
        <a href=\"http://www.haumer.at\">Technical Consulting &amp; Electrical Engineering</a><br>
        3423 St.Andrae-Woerdern<br>
        Austria
      </td>
    </tr>
    <tr>
      <td valign=\"top\">Library officer</td>
      <td valign=\"top\">
        <a href=\"mailto:christian.kral@ait.ac.at\">C. Kral</a>
      </td>
      <td valign=\"top\">
        <a href=\"http://www.ait.ac.at\">Austrian Institute of Technology, AIT</a>, Mobility Department<br>
        1210 Vienna<br>
        Austria
      </td>
    </tr>
  </table>
 
</html>"));
    end Contact;
  
    class Glossar 
      annotation (Documentation(info="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <th>Abbreviation</th>
    <th>Comment</th> 
  </tr>
  <tr>
    <td>AC</td> 
    <td>alternating current</td> 
  </tr>
  <tr>
    <td>RMS</td>
    <td>root mean square</td> 
  </tr>
</table>
</html>"));
    end Glossar;
  end UsersGuide;


  extends Modelica.Icons.Library2;


    annotation (uses(Modelica(version="2.2.2")),
                version="0.9.3",
                versionDate="2009-12-04",
                preferedView="info", Documentation(info="<HTML>
<p>
<dl>
  <dt><b>Main Authors:</b></dt>
  <dd>
 <p>
  Anton Haumer<br>
  <a href=\"http://www.haumer.at/\">Technical Consulting &amp; Electrical Engineering</a><br>
  A-3423 St.Andrae-Woerdern, Austria<br>
  email: <a href=\"mailto:a.haumer@haumer.at\">a.haumer@haumer.at</a>
  </p>
  <p>
  Dr. Christian Kral<br>
  <a href=\"http://www.ait.ac.at/\">Austrian Institute of Technology, AIT</a><br>
  Mobility Department<br>
  Giefinggasse 2<br>
  A-1210 Vienna, Austria<br>
  email: <a href=\"mailto:christian.kral@ait.ac.at\">christian.kral@ait.ac.at</a>
  </p>
  </dd>
</dl>
</p>
<p>
Copyright &copy; 2009 Anton Haumer &amp; Christian Kral.
</p>
</HTML>", revisions="<HTML>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
    <tr>
      <th>Version</th>
      <th>Revision</th>
      <th>Date</th>
      <th>Author</th>
      <th>Comment</th>
    </tr>
    <tr>
      <td valign=\"top\">1.0.0</td>
      <td valign=\"top\">XXX</td>
      <td valign=\"top\">2009-XX-XX</td>
      <td valign=\"top\">A. Haumer<br>C. Kral</td>
      <td valign=\"top\"></td>
    </tr>
</table>>
</HTML>"),     Icon(       Line(points=[-90,-20; -78.7,14.2; -71.5,33.1; -65.1,46.4; -59.4,54.6;             -53.8,59.1; -48.2,59.8; -42.6,56.6; -36.9,49.7; -31.3,39.4; -24.9,             24.1; -16.83,1.2; 0.1,-50.8; 7.3,-70.2; 13.7,-84.2; 19.3,-93.1; 25,             -98.4; 30.6,-100; 36.2,-97.6; 41.9,-91.5; 47.5,-81.9; 53.9,-67.2;             62,-44.8; 70,-20],           style(color=0))));
                                                                                                    import 
  Modelica_QuasiStationary.Types.Complex;


  package Examples "Test examples" 
    extends Modelica.Icons.Library2;
  
    model TestConversionBlock "Test the conversion blocks" 
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.Ramp rms(duration=1, offset=1E-6) 
        annotation (extent=[-80,10; -60,30]);
      Modelica.Blocks.Sources.Ramp phi(height=4*Modelica.Constants.pi, duration=1) 
        annotation (extent=[-80,-30; -60,-10]);
      annotation (Diagram);
      Blocks.PolarToComplex polarToComplex annotation (extent=[-40,-10; -20,10]);
      Blocks.ComplexToReal complexToReal annotation (extent=[0,-10; 20,10]);
    equation 
      connect(phi.y, polarToComplex.phi) annotation (points=[-59,-20; -50,-20;
          -50,-7.8; -40,-7.8],   style(color=74, rgbcolor={0,0,127}));
      connect(polarToComplex.y, complexToReal.u) 
        annotation (points=[-19,0; 0,0], style(color=58, rgbcolor={0,127,0}));
      connect(rms.y, polarToComplex.rms) annotation (points=[-59,20; -50,20;
            -50,8; -40,8], style(color=74, rgbcolor={0,0,127}));
    end TestConversionBlock;
  
    model SeriesResonance "Series resonance circuit" 
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.Constant V 
        annotation (extent=[-50,40; -30,60], rotation=270);
      Modelica.Blocks.Sources.Constant phi(k=0) 
        annotation (extent=[-90,40; -70,60], rotation=270);
      Modelica.Blocks.Sources.Ramp f(
        height=2,
        duration=1,
        offset=1e-6) annotation (extent=[-70,-60; -50,-40], rotation=90);
      SinglePhase.Sources.VariableVoltageSource voltageSource 
        annotation (extent=[-20,-30; -40,-10],rotation=-90);
      Modelica_QuasiStationary.SinglePhase.Basic.Ground ground 
        annotation (extent=[-40,-60; -20,-40]);
      Modelica_QuasiStationary.SinglePhase.Basic.Resistor resistor(R_ref=0.1) 
        annotation (extent=[10,-10; 30,10]);
      Modelica_QuasiStationary.SinglePhase.Basic.Inductor inductor(L=1/(2*Modelica.Constants.pi)) 
        annotation (extent=[40,-10; 60,10]);
      Modelica_QuasiStationary.SinglePhase.Basic.Capacitor capacitor(C=1/(2*Modelica.Constants.pi)) 
        annotation (extent=[70,-10; 90,10], rotation=0);
      Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor currentSensor 
                                          annotation (extent=[-20,10; 0,-10]);
      annotation (Diagram);
      Blocks.PolarToComplex polarToComplex 
        annotation (extent=[-70,0; -50,20], rotation=270);
      Blocks.ComplexToPolar complexToPolar 
        annotation (extent=[-20,20; 0,40], rotation=90);
    equation 
      connect(f.y, voltageSource.f) annotation (points=[-60,-39; -60,-24; -40,
            -24], style(color=74, rgbcolor={0,0,127}));
      connect(polarToComplex.y, voltageSource.V) annotation (points=[-60,-1;
            -60,-16; -40,-16], style(color=58, rgbcolor={0,127,0}));
      connect(ground.pin, voltageSource.pin_n) annotation (points=[-30,-40; -30,
            -30], style(color=58, rgbcolor={0,127,0}));
      connect(voltageSource.pin_p, currentSensor.pin_p) annotation (points=[-30,-10;
            -30,0; -20,0],      style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.pin_n, resistor.pin_p) 
        annotation (points=[0,0; 2.5,0; 2.5,-3.36456e-022; 5,-3.36456e-022; 5,0; 10,
            0],                         style(color=58, rgbcolor={0,127,0}));
      connect(resistor.pin_n, inductor.pin_p) 
        annotation (points=[30,0; 32.5,0; 32.5,1.22125e-015; 35,1.22125e-015; 35,0;
            40,0],                       style(color=58, rgbcolor={0,127,0}));
      connect(inductor.pin_n, capacitor.pin_p) 
        annotation (points=[60,0; 62.5,0; 62.5,1.22125e-015; 65,1.22125e-015; 65,0;
            70,0],                       style(color=58, rgbcolor={0,127,0}));
      connect(capacitor.pin_n, ground.pin) annotation (points=[90,0; 90,-40; -30,
            -40],     style(color=58, rgbcolor={0,127,0}));
      connect(complexToPolar.u, currentSensor.y) annotation (points=[-10,20;
            -10,11], style(color=58, rgbcolor={0,127,0}));
      connect(phi.y, polarToComplex.phi) annotation (points=[-80,39; -80,30;
            -67.8,30; -67.8,20], style(color=74, rgbcolor={0,0,127}));
      connect(V.y, polarToComplex.rms) annotation (points=[-40,39; -40,30; -52,
            30; -52,20], style(color=74, rgbcolor={0,0,127}));
    end SeriesResonance;
  
    model ParallelResonance "Parallel resonance circuit" 
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.Constant I 
        annotation (extent=[-90,-60; -70,-40],rotation=90);
      Modelica.Blocks.Sources.Constant phi(k=0) 
        annotation (extent=[-50,-60; -30,-40], rotation=90);
      Modelica.Blocks.Sources.Ramp f(
        height=2,
        duration=1,
        offset=1e-6) annotation (extent=[-70,40; -50,60], rotation=270);
      SinglePhase.Sources.VariableCurrentSource currentSource 
        annotation (extent=[-20,30; -40,10],  rotation=-90);
      Modelica_QuasiStationary.SinglePhase.Basic.Ground ground 
        annotation (extent=[-40,-20; -20,0]);
      Modelica_QuasiStationary.SinglePhase.Basic.Resistor resistor(R_ref=10) 
        annotation (extent=[-20,10; 0,30],    rotation=-90);
      Modelica_QuasiStationary.SinglePhase.Basic.Inductor inductor(L=1/(2*Modelica.Constants.pi)) 
        annotation (extent=[0,10; 20,30],    rotation=-90);
      Modelica_QuasiStationary.SinglePhase.Basic.Capacitor capacitor(C=1/(2*Modelica.Constants.pi)) 
        annotation (extent=[20,10; 40,30],  rotation=-90);
      Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor voltageSensor 
                                          annotation (extent=[40,30; 60,10],  rotation=90);
      annotation (Diagram);
      Blocks.PolarToComplex polarToComplex 
        annotation (extent=[-70,-20; -50,0], rotation=90);
      Blocks.ComplexToPolar complexToPolar annotation (extent=[70,10; 90,30]);
    equation 
      connect(currentSource.pin_n, resistor.pin_p) annotation (points=[-30,30;
            -30,40; -10,40; -10,30], style(color=58, rgbcolor={0,127,0}));
      connect(currentSource.pin_n, inductor.pin_p) annotation (points=[-30,30;
            -30,40; 10,40; 10,30], style(color=58, rgbcolor={0,127,0}));
      connect(currentSource.pin_n, capacitor.pin_p) annotation (points=[-30,30;
            -30,40; 30,40; 30,30], style(color=58, rgbcolor={0,127,0}));
      connect(currentSource.pin_n, voltageSensor.pin_p) annotation (points=[-30,
            30; -30,40; 50,40; 50,30], style(color=58, rgbcolor={0,127,0}));
      connect(currentSource.pin_p, ground.pin) annotation (points=[-30,10; -30,
            0], style(color=58, rgbcolor={0,127,0}));
      connect(resistor.pin_n, ground.pin) annotation (points=[-10,10; -10,0;
            -30,0], style(color=58, rgbcolor={0,127,0}));
      connect(inductor.pin_n, ground.pin) annotation (points=[10,10; 10,0; -30,
            0], style(color=58, rgbcolor={0,127,0}));
      connect(capacitor.pin_n, ground.pin) annotation (points=[30,10; 30,0; -30,
            0], style(color=58, rgbcolor={0,127,0}));
      connect(voltageSensor.pin_n, ground.pin) annotation (points=[50,10; 50,0;
            -30,0], style(color=58, rgbcolor={0,127,0}));
      connect(f.y, currentSource.f) annotation (points=[-60,39; -60,24; -40,24],
          style(color=74, rgbcolor={0,0,127}));
      connect(polarToComplex.y, currentSource.I) annotation (points=[-60,1; -60,
            16; -40,16], style(color=58, rgbcolor={0,127,0}));
      connect(voltageSensor.y, complexToPolar.u) annotation (points=[61,20; 70,
            20], style(color=58, rgbcolor={0,127,0}));
      connect(phi.y, polarToComplex.phi) annotation (points=[-40,-39; -40,-32;
            -52.2,-32; -52.2,-20], style(color=74, rgbcolor={0,0,127}));
      connect(I.y, polarToComplex.rms) annotation (points=[-80,-39; -80,-32;
            -68,-32; -68,-20], style(color=74, rgbcolor={0,0,127}));
    end ParallelResonance;
  
    annotation (Icon(
                Ellipse(extent=[-80,44; 60,-96], style(color=10, rgbcolor={95,
                95,95})), Polygon(points=[-40,36; -40,-88; 60,-26; -40,36],
            style(
            color=10,
            rgbcolor={95,95,95},
            fillColor=10,
            rgbfillColor={95,95,95}))));
    model BalancingStar "Balancing an unsymmetrical star-connected load" 
    //P.Vaske, Berechnung von Drehstromschaltungen, Teubner 1973, Seite 42, Beispiel 18
      extends Modelica.Icons.Example;
      parameter Integer m=3;
      parameter Modelica.SIunits.Voltage V = 100;
      parameter Modelica.SIunits.Frequency f = 50;
      parameter Modelica.SIunits.Resistance R = 10;
      parameter Modelica.SIunits.Inductance L = (R*sqrt(3))/(2*Modelica.Constants.pi*f);
      parameter Modelica.SIunits.Capacitance C = 1/(R*sqrt(3))/(2*Modelica.Constants.pi*f);
      MultiPhase.Sources.VoltageSource voltageSource(
        m=m,
        f=f,
        V=fill(V, m),
        phi={-(j - 1)*2*Modelica.Constants.pi/m for j in 1:m}) 
        annotation (extent=[-90,-30; -70,-10], rotation=270);
      MultiPhase.Basic.Star star(m=m) 
        annotation (extent=[-90,-60; -70,-40], rotation=270);
      SinglePhase.Basic.Ground ground 
        annotation (extent=[-90,-90; -70,-70]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p1(m=m, k=1) 
        annotation (extent=[-10,40; 10,60]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p2(k=2, m=m) 
        annotation (extent=[-10,0; 10,20]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p3(k=3, m=m) 
        annotation (extent=[-10,-40; 10,-20]);
      MultiPhase.Sensors.PowerSensor powerSensor(m=m) 
        annotation (extent=[-70,0; -50,20]);
      MultiPhase.Sensors.CurrentSensor currentSensor(m=m) 
        annotation (extent=[-40,0; -20,20]);
      SinglePhase.Basic.Resistor resistor(R_ref=R) 
        annotation (extent=[20,-40; 40,-20], rotation=0);
      SinglePhase.Basic.Capacitor capacitor(C=C) 
        annotation (extent=[20,0; 40,20],  rotation=0);
      SinglePhase.Basic.Inductor inductor(L=L) 
        annotation (extent=[20,40; 40,60],   rotation=0);
      annotation (Diagram(Text(
            extent=[-100,-80; 100,-100],
            style(color=3, rgbcolor={0,0,255}),
            string="L and C are choosen such way that the neutral current is 0")));
      MultiPhase.Basic.Star star2(m=m) 
        annotation (extent=[70,-60; 90,-40],   rotation=270);
      MultiPhase.Basic.PlugToPin_n plugToPin_n1(m=m, k=1) 
        annotation (extent=[50,40; 70,60], rotation=180);
      MultiPhase.Basic.PlugToPin_n plugToPin_n2(k=2, m=m) 
        annotation (extent=[50,0; 70,20], rotation=180);
      MultiPhase.Basic.PlugToPin_n plugToPin_n3(k=3, m=m) 
        annotation (extent=[50,-40; 70,-20], rotation=180);
      SinglePhase.Sensors.CurrentSensor currentSensor0 
        annotation (extent=[-40,-70; -20,-50], rotation=180);
    equation 
      connect(ground.pin, star.pin_n) annotation (points=[-80,-70; -80,-60], style(
            color=58, rgbcolor={0,127,0}));
      connect(star.plug_p, voltageSource.plug_n) annotation (points=[-80,-40; -80,
            -30], style(color=58, rgbcolor={0,127,0}));
      connect(voltageSource.plug_p, powerSensor.currentP) annotation (points=[-80,-10;
            -80,10; -70,10],      style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.currentN, currentSensor.plug_p) 
        annotation (points=[-50,10; -40,10], style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.voltageP, powerSensor.currentP) annotation (points=[-60,20;
            -70,20; -70,10],     style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.voltageN, star.plug_p) annotation (points=[-60,0; -60,-40;
            -80,-40], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p2.plug_p) 
        annotation (points=[-20,10; -2,10], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p3.plug_p) annotation (points=[-20,10;
            -20,-30; -2,-30], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p1.plug_p) annotation (points=[-20,10;
            -20,50; -2,50], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_p1.pin_p, inductor.pin_p) 
        annotation (points=[2,50; 20,50], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_p2.pin_p, capacitor.pin_p) 
        annotation (points=[2,10; 20,10], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_p3.pin_p, resistor.pin_p) 
        annotation (points=[2,-30; 20,-30], style(color=58, rgbcolor={0,127,0}));
      connect(inductor.pin_n, plugToPin_n1.pin_n) 
        annotation (points=[40,50; 58,50], style(color=58, rgbcolor={0,127,0}));
      connect(capacitor.pin_n, plugToPin_n2.pin_n) 
        annotation (points=[40,10; 58,10], style(color=58, rgbcolor={0,127,0}));
      connect(resistor.pin_n, plugToPin_n3.pin_n) 
        annotation (points=[40,-30; 58,-30], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_n1.plug_n, star2.plug_p) annotation (points=[62,50; 80,50;
            80,-40], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_n2.plug_n, star2.plug_p) annotation (points=[62,10; 80,10;
            80,-40], style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_n3.plug_n, star2.plug_p) annotation (points=[62,-30; 80,-30;
            80,-40], style(color=58, rgbcolor={0,127,0}));
      connect(star2.pin_n, currentSensor0.pin_p) 
        annotation (points=[80,-60; -20,-60], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor0.pin_n, star.pin_n) annotation (points=[-40,-60; -80,
            -60], style(color=58, rgbcolor={0,127,0}));
    end BalancingStar;
  
    model BalancingDelta "Balancing an unsymmetrical delta-connected load" 
    //P.Vaske, Berechnung von Drehstromschaltungen, Teubner 1973, Seite 43, Beispiel 23
      extends Modelica.Icons.Example;
      parameter Integer m=3;
      parameter Modelica.SIunits.Voltage V_LL = 100;
      parameter Modelica.SIunits.Frequency f = 50;
      parameter Modelica.SIunits.Resistance R = 10;
      parameter Modelica.SIunits.Inductance L = (R*sqrt(3))/(2*Modelica.Constants.pi*f);
      parameter Modelica.SIunits.Capacitance C = 1/(R*sqrt(3))/(2*Modelica.Constants.pi*f);
      MultiPhase.Sources.VoltageSource voltageSource(
        m=m,
        f=f,
        V=fill(V_LL, m),
        phi={-(j - 1)*2*Modelica.Constants.pi/m for j in 1:m}) 
        annotation (extent=[-80,-30; -60,-10], rotation=270);
      MultiPhase.Basic.Star star(m=m) 
        annotation (extent=[-80,-60; -60,-40], rotation=270);
      SinglePhase.Basic.Ground ground 
        annotation (extent=[-80,-90; -60,-70]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p1(m=m, k=1) 
        annotation (extent=[12,70; 32,90]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p2(k=2, m=m) 
        annotation (extent=[10,0; 30,20]);
      MultiPhase.Basic.PlugToPin_p plugToPin_p3(k=3, m=m) 
        annotation (extent=[10,-70; 30,-50]);
      MultiPhase.Sensors.PowerSensor powerSensor(m=m) 
        annotation (extent=[-60,0; -40,20]);
      MultiPhase.Sensors.CurrentSensor currentSensor(m=m) 
        annotation (extent=[-30,0; -10,20]);
      SinglePhase.Basic.Resistor resistor(R_ref=R) 
        annotation (extent=[60,20; 80,40], rotation=90);
      SinglePhase.Basic.Capacitor capacitor(C=C) 
        annotation (extent=[30,22; 50,42], rotation=270);
      SinglePhase.Basic.Inductor inductor(L=L) 
        annotation (extent=[30,-48; 50,-28], rotation=270);
      SinglePhase.Sensors.CurrentSensor currentSensor12 
        annotation (extent=[50,52; 30,72], rotation=270);
      SinglePhase.Sensors.CurrentSensor currentSensor23 
        annotation (extent=[50,-18; 30,2],   rotation=270);
      SinglePhase.Sensors.CurrentSensor currentSensor31 
        annotation (extent=[60,-20; 80,0],   rotation=90);
      annotation (Diagram(Text(
            extent=[-100,-80; 100,-100],
            style(color=3, rgbcolor={0,0,255}),
            string=
                "L and C are choosen such way that the 3 source currents are balanced")));
      Blocks.ComplexToPolar complexToPolar[m] 
        annotation (extent=[-30,-30; -10,-10], rotation=270);
    equation 
      connect(ground.pin, star.pin_n) annotation (points=[-70,-70; -70,-60], style(color=58,
            rgbcolor={0,127,0}));
      connect(star.plug_p, voltageSource.plug_n) annotation (points=[-70,-40;
            -70,-30],
                  style(color=58, rgbcolor={0,127,0}));
      connect(voltageSource.plug_p, powerSensor.currentP) annotation (points=[-70,-10;
            -70,10; -60,10],      style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.currentP, powerSensor.voltageP) annotation (points=[-60,10;
            -60,20; -50,20],     style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.currentN, currentSensor.plug_p) 
        annotation (points=[-40,10; -30,10], style(color=58, rgbcolor={0,127,0}));
      connect(powerSensor.voltageN, star.plug_p) annotation (points=[-50,0; -50,
            -40; -70,-40],
                      style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p2.plug_p) 
        annotation (points=[-10,10; 18,10], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p1.plug_p) annotation (points=[-10,10;
            0,10; 0,80; 20,80],
                            style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.plug_n, plugToPin_p3.plug_p) annotation (points=[-10,10;
            0,10; 0,-60; 18,-60],
                              style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor12.pin_p, plugToPin_p1.pin_p) annotation (points=[40,72;
            40,80; 24,80],style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor12.pin_n, capacitor.pin_p) annotation (points=[40,52;
            40,42],     style(color=58, rgbcolor={0,127,0}));
      connect(capacitor.pin_n, plugToPin_p2.pin_p) annotation (points=[40,22;
            40,10; 22,10],
                   style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_p2.pin_p, currentSensor23.pin_p) annotation (points=[22,10;
            40,10; 40,2], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor23.pin_n, inductor.pin_p) annotation (points=[40,-18;
            40,-28],                              style(color=58, rgbcolor={0,127,0}));
      connect(inductor.pin_n, plugToPin_p3.pin_p) annotation (points=[40,-48;
            40,-60; 22,-60],
                         style(color=58, rgbcolor={0,127,0}));
      connect(plugToPin_p1.pin_p, resistor.pin_n) annotation (points=[24,80; 70,
            80; 70,40],
                    style(color=58, rgbcolor={0,127,0}));
      connect(resistor.pin_p, currentSensor31.pin_n) 
        annotation (points=[70,20; 70,0], style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor31.pin_p, plugToPin_p3.pin_p) annotation (points=[70,-20;
            70,-60; 22,-60],style(color=58, rgbcolor={0,127,0}));
      connect(currentSensor.y, complexToPolar.u) annotation (points=[-20,-1;
          -20,-5.5; -20,-10; -20,-10],   style(color=58, rgbcolor={0,127,0}));
    end BalancingDelta;
  end Examples;


  package Blocks "Blocks for complex signals" 
    extends Modelica.Icons.Library2;
  
    block RealToComplex "Convert (re, im) to Complex" 
      extends Modelica.Blocks.Interfaces.BlockIcon;
      Modelica.Blocks.Interfaces.RealInput re 
        annotation (extent=[-120,60; -80,100]);
      Modelica.Blocks.Interfaces.RealInput im 
        annotation (extent=[-120,-98; -80,-58]);
      Interfaces.ComplexOutput y annotation (extent=[100,-10; 120,10]);
      annotation (Diagram, Icon(Text(
            extent=[-78,100; 22,40],
            style(color=3, rgbcolor={0,0,255}),
            string="re"), Text(
            extent=[-80,-40; 20,-100],
            style(color=3, rgbcolor={0,0,255}),
            string="im")));
    equation 
      y.re = re;
      y.im = im;
    end RealToComplex;
  
    block ComplexToReal "Convert Complex to (re, im)" 
      extends Modelica.Blocks.Interfaces.BlockIcon;
      annotation (Diagram, Icon(Text(
            extent=[0,100; 100,40],
            style(color=3, rgbcolor={0,0,255}),
            string="re"), Text(
            extent=[0,-40; 100,-100],
            style(color=3, rgbcolor={0,0,255}),
            string="im")));
      Modelica.Blocks.Interfaces.RealOutput re annotation (extent=[100,70; 120,90]);
      Modelica.Blocks.Interfaces.RealOutput im annotation (extent=[100,-90; 120,-70]);
      Interfaces.ComplexInput u 
        annotation (extent=[-120,-20; -80,20]);
    equation 
      re  = u.re;
      im  = u.im;
    end ComplexToReal;
  
    block PolarToComplex "Convert (len, phi) to Complex" 
      extends Modelica.Blocks.Interfaces.BlockIcon;
      Modelica.Blocks.Interfaces.RealInput rms 
        annotation (extent=[-120,60; -80,100]);
      Modelica.Blocks.Interfaces.RealInput phi 
        annotation (extent=[-120,-98; -80,-58]);
      Interfaces.ComplexOutput y annotation (extent=[100,-10; 120,10]);
      annotation (Diagram, Icon(Text(
            extent=[-80,100; 20,40],
            style(color=3, rgbcolor={0,0,255}),
            string="rms"), Text(
            extent=[-80,-40; 20,-100],
            style(color=3, rgbcolor={0,0,255}),
            string="phi")));
    equation 
      y.re = rms*cos(phi);
      y.im = rms*sin(phi);
    end PolarToComplex;
  
    block ComplexToPolar "Convert Complex to (len, phi)" 
      extends Modelica.Blocks.Interfaces.BlockIcon;
      annotation (Diagram, Icon(Text(
            extent=[0,100; 100,40],
            style(color=3, rgbcolor={0,0,255}),
            string="rms"), Text(
            extent=[0,-40; 100,-100],
            style(color=3, rgbcolor={0,0,255}),
            string="phi")));
      Modelica.Blocks.Interfaces.RealOutput rms 
        annotation (extent=[100,70; 120,90]);
      Modelica.Blocks.Interfaces.RealOutput phi 
        annotation (extent=[100,-90; 120,-70]);
      Interfaces.ComplexInput u 
        annotation (extent=[-120,-20; -80,20]);
    equation 
      rms = sqrt(u.re^2 + u.im^2);
      phi = Modelica.Math.atan2(u.im, u.re);
    end ComplexToPolar;
  
        block Sum "Output the sum of the elements of the input vector" 
          extends Modelica.Blocks.Interfaces.BlockIcon;
          parameter Integer nin=1 "Number of inputs";
          annotation (
            Icon(Line(points=[26, 42; -34, 42; 6, 2; -34, -38; 26, -38], style(
                    color=0, thickness=1)), Text(extent=[-150, 150; 150, 110],
                  string="%name")),
            Diagram);
    Interfaces.ComplexInput u[nin] annotation (extent=[-140,-20; -100,20]);
    Interfaces.ComplexOutput y annotation (extent=[100,-10; 120,10]);
        equation 
        //y = Complex.'sum'(u);
          y.re = sum(u.re);
          y.im = sum(u.im);
        end Sum;
  
    package Interfaces "Complex signal interfaces" 
    
    connector ComplexSignal = Modelica_QuasiStationary.Types.Complex;
    
    connector ComplexInput = input ComplexSignal 
      annotation (defaultComponentName="u",
      Coordsys(extent=[-100, -100; 100, 100],
        grid=[1,1],
        component=[20,20],
          scale=0.2),
      Icon(coordinateSystem(extent=[-100,-100; 100,100]),
           Polygon(points=[-100,100; 100,0; -100,-100; -100,100], style(
              color=58,
              rgbcolor={0,127,0},
              fillColor=58,
              rgbfillColor={0,127,0}))),
      Diagram(Polygon(points=[0,50; 100,0; 0,-50; 0,50], style(
              color=58,
              rgbcolor={0,127,0},
              fillColor=58,
              rgbfillColor={0,127,0})),
                                      Text(
          extent=[-100,100; 99,60],
          string="%name",
            style(color=74, rgbcolor={0,0,127}))));
    
    connector ComplexOutput = output ComplexSignal 
      annotation (defaultComponentName="y",
      Coordsys(extent=[-100, -100; 100, 100],
        grid=[1,1],
        component=[20,20]),
      Icon(Polygon(points=[-100, 100; 100, 0; -100, -100; -100, 100], style(
            color=58,
            rgbcolor={0,127,0},
            fillColor=7,
            rgbfillColor={255,255,255}))),
      Diagram(Polygon(points=[-100,50; 0,0; -100,-50; -100,50], style(
            color=58,
            rgbcolor={0,127,0},
            fillColor=7,
            rgbfillColor={255,255,255})),
          Text(
          extent=[-100,100; 100,60],
          string="%name",
            style(color=74, rgbcolor={0,0,127}))));
    
    end Interfaces;
    annotation (Icon(
      Line(points=[-32,-62; -64,-62; -64,-11; -32,-11],     style(color=0)),
      Polygon(points=[-32,-11; -46,-7; -46,-15; -32,-11],      style(
          color=0,
          fillColor=0,
          fillPattern=1)),
      Rectangle(extent=[-32,4; 16,-25],    style(color=0)),
      Line(points=[16,-10; 49,-10; 49,-61; 16,-61],     style(color=0)),
      Polygon(points=[16,-61; 29,-57; 29,-64; 16,-61],     style(
          color=0,
          fillColor=0,
          fillPattern=1)),
      Rectangle(extent=[-32,-46; 16,-75],   style(color=0))));
  end Blocks;


  package SinglePhase "Single phase AC components" 
    extends Modelica.Icons.Library2;
  
    package Basic "Basic components for AC singlephase models" 
      extends Modelica.Icons.Library2;
    
      model Ground "Electrical ground" 
      
        Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin pin 
          annotation (extent=[-10,90; 10,110]);
        annotation (Icon(
            Line(points=[-60,50; 60,50]),
            Line(points=[-40,30; 40,30]),
            Line(points=[-20,10; 20,10]),
            Line(points=[0,90; 0,50]),
            Text(extent=[100,-40; -100,0], string="%name")), Diagram,
          Documentation(info="<html>
<p>
Ground of a single phase circuit. The potential at the ground node is zero. 
Every electrical circuit, e.g. a series resonance
<a href=\"Modelica://Modelica_QuasiStationary.Examples.SeriesResonance\">
          example</a>, has to contain at least one ground object. 
</p>
 
</html>"));
      equation 
        pin.v.re = 0;
        pin.v.im = 0;
      end Ground;
    
      model Resistor "Singlephase linear resistor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Resistance R_ref(start=1);
        parameter Modelica.SIunits.Temperature T_ref=293.15 
        "Reference temperature";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref
        =                                                                       0 
        "Temperature coefficient of resistance (R_actual = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        extends MoveToModelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
        Modelica.SIunits.Resistance R_actual 
        "Resistance = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Text(extent=[100,-80; -100,-40], string="R=%R_ref")),
            Diagram,
          Documentation(info="<html>
<p>
The linear resistor connects the complex voltage <i><u>v</u></i> with the complex
current <i><u>i</u></i> by <i><u>i</u>*R = <u>v</u></i>.
The resistance <i>R</i> is allowed to be positive, zero, or negative.
</p>

<p>
The resistor model also has an optional 
<a href=\"Modelica://Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort\">conditional heat port</a>. 
A linear temperature dependency of the resistance for an enabled heat port is also taken into account.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
      equation 
        assert((1 + alpha_ref*(T_heatPort - T_ref)) >= Modelica.Constants.eps, "Temperature outside scope of model!");
        R_actual = R_ref*(1 + alpha_ref*(T_heatPort - T_ref));
        v.re = R_actual*i.re;
        v.im = R_actual*i.im;
        LossPower = v.re*i.re + v.im*i.im;
      end Resistor;
    
      model Conductor "Singlephase linear conductor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Conductance G_ref(start=1);
        parameter Modelica.SIunits.Temperature T_ref=293.15 
        "Reference temperature";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref
        =                                                                       0 
        "Temperature coefficient of conductance (G_actual = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        extends MoveToModelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
        Modelica.SIunits.Conductance G_actual 
        "Conductance = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Text(extent=[100,-80; -100,-40], string="G=%G_ref")),
                                                            Diagram,
          Documentation(info="<html>
 
<p>
The linear conductor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by <i><u>i</u> = <u>v</u>*G</i>.
The conductance <i>G</i> is allowed to be positive, zero, or negative.
</p>
 
<p>
The conductor model also has an optional 
<a href=\"Modelica://Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort\">conditional heat port</a>. 
A linear temperature dependency of the resistance for an enabled heat port is also taken into account.
</p>
 
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
      equation 
        assert((1 + alpha_ref*(T_heatPort - T_ref)) >= Modelica.Constants.eps, "Temperature outside scope of model!");
        G_actual = G_ref/(1 + alpha_ref*(T_heatPort - T_ref));
        i.re = G_actual*v.re;
        i.im = G_actual*v.im;
        LossPower = v.re*i.re + v.im*i.im;
      end Conductor;
    
      model Capacitor "Singlephase linear capacitor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Capacitance C(start=1);
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Line(points=[-14,28; -14,-28],   style(thickness=2)),
            Line(points=[14,28; 14,-28],   style(thickness=2)),
            Line(points=[-90,0; -14,0]),
            Line(points=[14,0; 90,0]),
            Text(extent=[100,-80; -100,-40], string="C=%C")),
                                        Diagram,
          Documentation(info="<html>
 
<p>
The linear capacitor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by <i><u>i</u> = j*&omega;*C*<u>v</u></i>.
The capacitance <i>C</i> is allowed to be positive, zero, or negative.
</p>
 
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
      equation 
        i.re = -omega*C*v.im;
        i.im =  omega*C*v.re;
      end Capacitor;
    
      model Inductor "Singlephase linear inductor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Inductance L(start=1);
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Ellipse(extent=[-60,-15; -30,15]),
            Ellipse(extent=[-30,-15; 0,15]),
            Ellipse(extent=[0,-15; 30,15]),
            Ellipse(extent=[30,-15; 60,15]),
            Rectangle(extent=[-60,-30; 60,0], style(color=7, fillColor=7)),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Text(extent=[100,-80; -100,-40], string="L=%L")),
                                          Diagram,
          Documentation(info="<html>
 
<p>
The linear inductor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by  <i><u>v</u> = j*&omega;*L*<u>i</u></i>.
The Inductance <i>L</i> is allowed to be positive, zero, or negative.
</p>
 
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
      equation 
        v.re = -omega*L*i.im;
        v.im =  omega*L*i.re;
      end Inductor;
    
      model VariableResistor "Singlephase variable resistor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Temperature T_ref=293.15 
        "Reference temperature";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref
        =                                                                       0 
        "Temperature coefficient of resistance (R_actual = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        extends MoveToModelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
        Modelica.SIunits.Resistance R_actual 
        "Resistance = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Line(points=[0,90; 0,30], style(
                color=3,
                rgbcolor={0,0,255},
                smooth=0))), Diagram,
          Documentation(info="<html>
 
<p>
The linear resistor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by <i><u>i</u>*R = <u>v</u></i>.
The resistance <i>R</i> is given as input signal.
<p>
 
<p>
The variable resistor model also has an optional 
<a href=\"Modelica://Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort\">conditional heat port</a>. 
A linear temperature dependency of the resistance for an enabled heat port is also taken into account.
</p>
 
<h4>Note</h4>
<p>
A zero crossing of the R signal could cause singularities due to the actual structure of the connected network.
</p>
 
<p>
The variable resistor model also has an optional 
<a href=\"Modelica://Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort\">conditional heat port</a>. 
A linear temperature dependency of the resistance for an enabled heat port is also taken into account.
</p>
 
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput R_ref 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
      equation 
        assert((1 + alpha_ref*(T_heatPort - T_ref)) >= Modelica.Constants.eps, "Temperature outside scope of model!");
        R_actual = R_ref*(1 + alpha_ref*(T_heatPort - T_ref));
        v.re = R_actual*i.re;
        v.im = R_actual*i.im;
        LossPower = v.re*i.re + v.im*i.im;
      end VariableResistor;
    
      model VariableConductor "Singlephase variable conductor" 
        extends Interfaces.OnePort;
        parameter Modelica.SIunits.Temperature T_ref=293.15 
        "Reference temperature";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref
        =                                                                       0 
        "Temperature coefficient of conductance (G_actual = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        extends MoveToModelica.Electrical.Analog.Interfaces.ConditionalHeatPort(T = T_ref);
        Modelica.SIunits.Conductance G_actual 
        "Conductance = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Line(points=[0,90; 0,30], style(
                color=3,
                rgbcolor={0,0,255},
                smooth=0))), Diagram,
          Documentation(info="<html>
 
<p>
The linear conductor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by <i><u>i</u> = G*<u>v</u></i>.
The conductance <i>G</i> is given as input signal.
</p>
 
<p>
The variable conductor model also has an optional 
<a href=\"Modelica://Modelica.Electrical.Analog.Interfaces.ConditionalHeatPort\">conditional heat port</a>. 
A linear temperature dependency of the resistance for an enabled heat port is also taken into account.
</p>
 
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput G_ref 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
      equation 
        assert((1 + alpha_ref*(T_heatPort - T_ref)) >= Modelica.Constants.eps, "Temperature outside scope of model!");
        G_actual = G_ref/(1 + alpha_ref*(T_heatPort - T_ref));
        i.re = G_actual*v.re;
        i.im = G_actual*v.im;
        LossPower = v.re*i.re + v.im*i.im;
      end VariableConductor;
    
      model VariableCapacitor "Singlephase variable capacitor" 
        extends Interfaces.OnePort;
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[-14,28; -14,-28],   style(thickness=2)),
            Line(points=[14,28; 14,-28],   style(thickness=2)),
            Line(points=[-90,0; -14,0]),
            Line(points=[14,0; 90,0]),
            Line(points=[0,90; 0,30],   style(color=73))),
            Diagram,
          Documentation(info="<html>

<p>
The linear capacitor connects the voltage <i><u>v</u></i> with the
current <i><u>i</u></i> by <i><u>i</u> = j*&omega;*C*<u>v</u></i>.
The capacitance <i>C</i> is given as input signal.
</p>

<h4>Note</h4>
<p>
The abstraction of a variable capacitor at quasi stationary operation assumes:<br>
<img src=\"../Images/SinglePhase/Basic/dc_dt.png\">.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput C 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
      equation 
        i.re = -omega*C*v.im;
        i.im =  omega*C*v.re;
      end VariableCapacitor;
    
      model VariableInductor "Singlephase variable inductor" 
        extends Interfaces.OnePort;
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Ellipse(extent=[-60,-15; -30,15]),
            Ellipse(extent=[-30,-15; 0,15]),
            Ellipse(extent=[0,-15; 30,15]),
            Ellipse(extent=[30,-15; 60,15]),
            Rectangle(extent=[-60,-30; 60,0], style(color=7, fillColor=7)),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Line(points=[0,90; 0,8],   style(color=73))),
            Diagram,
          Documentation(info="<html>

<p>
The linear inductor connects the branch voltage <i><u>v</u></i> with the
branch current <i><u>i</u></i> by <i><u>v</u> = j*&omega;*L*<u>i</u></i>. The inductance <i>L</i> is given as input signal.
</p>

<h4>Note</h4>
<p>
The abstraction of a variable inductor at quasi stationary operation assumes:<br>
<img src=\"../Images/SinglePhase/Basic/dl_dt.png\">
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>Variable capacitor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput L 
          annotation (extent=[-20,88; 20,128],   rotation=-90);
      equation 
        v.re = -omega*L*i.im;
        v.im =  omega*L*i.re;
      end VariableInductor;
      annotation (Icon(
          Line(points=[-100,-40; -80,-40]),
          Line(points=[60,-40; 80,-40]),
          Rectangle(extent=[-80,-10; 60,-70],  style(
              color=3,
              fillColor=7,
              fillPattern=1))));
    end Basic;
  
    package Ideal 
      extends Modelica.Icons.Library2;
    
      annotation (Icon(
           Ellipse(extent=[-54,-56; -46,-64]),
           Line(points=[-100,-60; -54,-60]),
           Line(points=[-47,-58; 30,-10]),
           Line(points=[30,-60; 80,-60]),
           Line(points=[30,-40; 30,-60])));
    
      model Idle "Idle branch" 
        extends Interfaces.OnePort;
        annotation (
          Icon(
            Rectangle(extent=[-60, 60; 60, -60], style(fillColor=7)),
            Line(points=[-90, 0; -41, 0]),
            Line(points=[91, 0; 40, 0]),
            Text(extent=[-100, 100; 100, 70], string="%name")),
          Diagram,
          Window(
            x=0.36,
            y=0.16,
            width=0.6,
            height=0.6),
          Documentation(info="<html>
<p>
This model is a simple idle branch considering the complex current <i><u>i</u></i> = 0.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Short\">Short</a>
</p>
</html>"));
      equation 
        i.re = 0;
        i.im = 0;
      end Idle;
    
      model Short "Short cut branch" 
        extends Interfaces.OnePort;
        annotation (
          Icon(
            Rectangle(extent=[-60, 60; 60, -60], style(fillColor=7)),
            Line(points=[91, 0; -90, 0]),
            Text(extent=[-100, 100; 100, 70], string="%name")),
          Window(
            x=0.31,
            y=0.14,
            width=0.6,
            height=0.6),
          Documentation(info="<html>
<p>
This model is a simple short cut branch considering the complex voltage <i><u>v</u></i> = 0.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Idle\">Idle</a>
</p>
</html>"));
      equation 
        v.re = 0;
        v.im = 0;
      end Short;
    
    end Ideal;
  
    package Interfaces "Interfaces for AC singlephase models" 
      extends Modelica.Icons.Library2;
    
      connector Pin "Basic connector" 
        Types.ComplexVoltage v "Complex potential at the node";
        flow Types.ComplexCurrent i "Complex current flowing into the pin";
        annotation (Documentation(info="<html>
<p>
The potential of this connector is the complex voltage and the flow variable is the complex current.
The <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive</a> and
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">negative pin</a> are 
derived from this base connector.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>

</html>"));
      end Pin;
    
      connector PositivePin "Positive connector" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Pin;
        Types.Reference reference "Reference";
        annotation (Diagram(Text(
              extent=[-100,100; 100,60],
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=3,
                rgbfillColor={0,0,255},
                fillPattern=1),
              string="%name"), Rectangle(extent=[-40,40; 40,-40], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=58,
                rgbfillColor={0,127,0}))),
                                  Icon(Rectangle(extent=[-100,100; 100,-100],
                style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=58,
                rgbfillColor={0,127,0},
                fillPattern=1))),
        Documentation(info="<html>

<p>
The positive pin is based on <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>. 
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of the quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">negative pin</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
      end PositivePin;
    
      connector NegativePin "Negative Connector" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Pin;
        Types.Reference reference "Reference";
        annotation (Diagram(Text(
              extent=[-100,100; 100,60],
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=3,
                rgbfillColor={0,0,255},
                fillPattern=1),
              string="%name"), Rectangle(extent=[-40,40; 40,-40], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))), Icon(Rectangle(extent=[-100,100; 100,-100],
                style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
        Documentation(info="<html>

<p>
The negative pin is based on <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>. 
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of the quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive pin</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
      end NegativePin;
    
      partial model TwoPin "Two pins" 
        Types.ComplexVoltage v;
        Types.ComplexCurrent i;
        Modelica.SIunits.AngularVelocity omega = der(pin_p.reference.gamma);
        Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin pin_p 
        "Positive pin" 
          annotation (extent=[-110,-10; -90,10]);
        Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin pin_n 
        "Negative pin" 
          annotation (extent=[90,-10; 110,10]);
        annotation (Diagram, Documentation(info="<html>
<p>
This partial model uses a <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive</a>
and <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">negative pin</a> and defines the complex voltage difference as well as the complex current (into the positive pin). Additionally, the angular velocity of the quasi stationary system is explicitely defined as variable. This model is mainly intended to be used with graphical representation of user models.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.OnePort\">OnePort</a>
</p>
</html>"));
      equation 
        Connections.branch(pin_p.reference, pin_n.reference);
        pin_p.reference.gamma = pin_n.reference.gamma;
        i = pin_p.i;
        v.re = pin_p.v.re - pin_n.v.re;
        v.im = pin_p.v.im - pin_n.v.im;
      end TwoPin;
    
      partial model OnePort "Two pins, current through" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.TwoPin;
      equation 
        pin_p.i.re + pin_n.i.re = 0;
        pin_p.i.im + pin_n.i.im = 0;
        annotation (Diagram, Documentation(info="<html>
<p>
This partial model is based on <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.TwoPin\">TwoPin</a> and 
additionally considers the complex current balance of the  
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive</a> and the
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">negative pin</a>. 
This model is intended to be used with textual representation of user models.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.TwoPin\">TwoPin</a>
</p>
</html>"));
      end OnePort;
    
      partial model AbsoluteSensor "Partial potential sensor" 
        extends Modelica.Icons.RotationalSensor;
        Modelica.SIunits.AngularVelocity omega = der(pin.reference.gamma);
        Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin pin "Pin" 
          annotation (extent=[-110, -10; -90, 10]);
        annotation (Diagram, Icon(
          Line(points=[-70,0; -94,0], style(color=0)),
            Text(
              extent=[-100,100; 100,70],
              string="%name",
              style(
                pattern=0,
                fillColor=79,
                rgbfillColor={170,85,255},
                fillPattern=1)),
            Line(points=[70,0; 80,0; 90,0; 100,0],     style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
        Documentation(info="<html>
<p>
The absolute sensor partial model provides a single 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">positive pin</a> to measure the complex voltage. Additionally this model contains a proper icon and a definition of the angular velocity. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.RelativeSensor\">RelativeSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">PotentialSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.AbsoluteSensor\">MultiPhase.Interfaces.AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.RelativeSensor\">MultiPhase.Interfaces.RelativeSensor</a>
</p>

</html>"));
        Blocks.Interfaces.ComplexOutput y annotation (extent=[100,-10; 120,10]);
      equation 
        pin.i.re=0;
        pin.i.im=0;
      end AbsoluteSensor;
    
      partial model RelativeSensor "Partial voltage / current sensor" 
        extends Modelica.Icons.RotationalSensor;
        extends OnePort;
        annotation (Diagram, Icon(
            Line(points=[-70,0; -94,0],   style(color=0)),
            Line(points=[70,0; 94,0],   style(color=0)),
            Text(
              extent=[-100,100; 100,70],
              string="%name",
              style(
                pattern=0,
                fillColor=79,
                rgbfillColor={170,85,255},
                fillPattern=1)),
            Line(points=[0,-70; 0,-80; 0,-90; 0,-100],     style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
        Documentation(info="<html>
<p>
The relative sensor partial model relies on the 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.OnePort\">OnePort</a> to measure the complex voltage, current or power. Additionally this model contains a proper icon and a definition of the angular velocity. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.AbsoluteSensor\">AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">VoltageSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">CurrentSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">PowerSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.AbsoluteSensor\">MultiPhase.Interfaces.AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.RelativeSensor\">MultiPhase.Interfaces.RelativeSensor</a>
</p>

</html>"));
        Blocks.Interfaces.ComplexOutput y annotation (extent=[-10,-120; 10,-100],rotation=-90);
      end RelativeSensor;
    
      partial model Source "Partial voltage / current source" 
        extends OnePort;
        annotation (Icon(
            Ellipse(extent=[-50,50; 50,-50], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255})),
            Text(extent=[100,-100; -100,-60], string="%name"),
            Line(points=[-90,0; -50,0],style(color=0, rgbcolor={0,0,0})),
            Line(points=[50,0; 90,0],  style(color=0, rgbcolor={0,0,0}))), Diagram,
        Documentation(info="<html>
<p>
The source partial model relies on the 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.OnePort\">OnePort</a> and contains a proper icon. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Source\">MultiPhase.Interfaces.Source</a>.
</p>
</html>"));
      equation 
        Connections.root(pin_p.reference);
      end Source;
      annotation (Icon(Rectangle(extent=[-30,-20; 10,-60], style(
              color=0,
              rgbcolor={0,0,0},
              gradient=2,
              fillColor=3,
              rgbfillColor={0,0,255})), Rectangle(extent=[-60,10; 40,-90],
              style(color=3, rgbcolor={0,0,255}))), Documentation(info="<html>
<p>This package contains connector specifications and partial models for more complex components.</p>
</html>"));
    end Interfaces;
  
    package Sensors "AC singlephase sensors" 
      extends Modelica.Icons.Library2;
    
      model PotentialSensor "Potential sensor" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.AbsoluteSensor;
        annotation (Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="V",
              style(color=0))), Diagram,
        Documentation(info="<html>

<p>
This sensor can be used to measure the complex potential.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">VoltageSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">CurrentSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
      equation 
        y = pin.v;
      end PotentialSensor;
    
      model VoltageSensor "Voltage sensor" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.RelativeSensor;
        annotation (Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="V",
              style(color=0))), Diagram,
        Documentation(info="<html>
<p>
This sensor can be used to measure the complex voltage.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">PotentialSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">CurrentSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
      equation 
        i.re = 0;
        i.im = 0;
        y = v;
      end VoltageSensor;
    
      model CurrentSensor "Current sensor" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.RelativeSensor;
        annotation (Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="I",
              style(color=0))), Diagram,
        Documentation(info="<html>
<p>
This sensor can be used to measure the complex current.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">PotentialSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">VoltageSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
      equation 
        v.re = 0;
        v.im = 0;
        y = i;
      end CurrentSensor;
    
      model PowerSensor "Power sensor" 
      
        annotation (Diagram, Icon(
            Ellipse(extent=[-70,70; 70,-70],   style(color=0, fillColor=7)),
            Line(points=[0,100; 0,70], style(color=3, rgbcolor={0,0,255})),
            Line(points=[0,-70; 0,-100], style(color=3, rgbcolor={0,0,255})),
            Line(points=[-100,0; 100,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[0,70; 0,40],   style(color=0)),
            Line(points=[22.9,32.8; 40.2,57.3],   style(color=0)),
            Line(points=[-22.9,32.8; -40.2,57.3],   style(color=0)),
            Line(points=[37.6,13.7; 65.8,23.9],   style(color=0)),
            Line(points=[-37.6,13.7; -65.8,23.9],   style(color=0)),
            Line(points=[0,0; 9.02,28.6],   style(color=0)),
            Polygon(points=[-0.48,31.6; 18,26; 18,57.2; -0.48,31.6],     style(
                color=0,
                fillColor=0,
                fillPattern=1)),
            Ellipse(extent=[-5,5; 5,-5],   style(
                color=0,
                gradient=0,
                fillColor=0,
                fillPattern=1)),
               Text(
              extent=[-29,-11; 30,-70],
              style(color=0),
              string="P"),
            Line(points=[-80,-100; -80,0],   style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
        Documentation(info="<html>

<p>
This sensor can be used to measure the complex apparent power.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">PotentialSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">VoltageSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">CurrentSensor</a>,
</p>

</html>"));
        Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin currentP 
          annotation (extent=[-110,-10; -90,10]);
        Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin currentN 
          annotation (extent=[90,-10; 110,10]);
        Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin voltageP 
          annotation (extent=[-10,90; 10,110]);
        Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin voltageN 
          annotation (extent=[-10,-110; 10,-90]);
        output Types.ComplexCurrent i;
        output Types.ComplexVoltage v;
        Blocks.Interfaces.ComplexOutput y 
          annotation (extent=[-90,-120; -70,-100], rotation=-90);
      equation 
        Connections.branch(currentP.reference, currentN.reference);
        currentP.reference.gamma = currentN.reference.gamma;
        Connections.branch(voltageP.reference, voltageN.reference);
        voltageP.reference.gamma = voltageN.reference.gamma;
        currentP.i.re + currentN.i.re = 0;
        currentP.i.im + currentN.i.im = 0;
        currentP.v.re - currentN.v.re = 0;
        currentP.v.im - currentN.v.im = 0;
        i = currentP.i;
        voltageP.i.re + voltageN.i.re = 0;
        voltageP.i.im + voltageN.i.im = 0;
        voltageP.i.re = 0;
        voltageP.i.im = 0;
        v.re = voltageP.v.re - voltageN.v.re;
        v.im = voltageP.v.im - voltageN.v.im;
      //P + j*Q = v * conj(i);
        y.re =  v.re*i.re + v.im*i.im;
        y.im = -v.re*i.im + v.im*i.re;
      end PowerSensor;
      annotation (Icon(
          Ellipse(extent=[-60,10; 40,-90], style(
              color=0,
              rgbcolor={0,0,0},
              fillColor=7,
              rgbfillColor={255,255,255})),
          Line(points=[-50,-16; -36,-25],
                                        style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[-35,0; -25,-14], style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[-10,7; -10,-10],
                                    style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Polygon(points=[-12,-24; -0.5,-27; 2,1.5; -12,-24],          style(
              color=0,
              fillColor=0,
              fillPattern=1)),
          Line(points=[-10,-40; -6,-26],  style(color=0)),
          Ellipse(extent=[-15,-35; -5,-45],
                                         style(
              color=0,
              gradient=0,
              fillColor=0,
              fillPattern=1)),
          Line(points=[15,0; 5,-14],  style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[30,-15; 16,-25],
                                      style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1))));
    end Sensors;
  
    package Sources "AC singlephase sources" 
      extends Modelica.Icons.Library2;
    
      model VoltageSource "Constant AC voltage" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Source;
        parameter Modelica.SIunits.Frequency f(start=1) 
        "frequency of the source";
        parameter Modelica.SIunits.Voltage V(start=1) 
        "RMS voltage of the source";
        parameter Modelica.SIunits.Angle phi(start=0) 
        "phase shift of the source";
        annotation (Icon(
            Text(string="+",
              extent=[-120,50; -20,0], style(color=3, rgbcolor={0,0,255})),
            Text(string="-",
              extent=[20,50; 120,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[50,0; -50,0],   style(color=0, rgbcolor={0,0,0}))),
            Documentation(info="<html>

<p>
This is a constant voltage source, specifying the complex voltage by the RMS voltage and the phase shift.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
      equation 
        omega = 2*Modelica.Constants.pi*f;
        v.re = V*cos(phi);
        v.im = V*sin(phi);
      end VoltageSource;
    
      model VariableVoltageSource "Variable AC voltage" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Source;
        annotation (Icon(
            Text(string="+",
              extent=[-120,50; -20,0], style(color=3, rgbcolor={0,0,255})),
            Text(string="-",
              extent=[20,50; 120,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[50,0; -50,0],   style(color=0, rgbcolor={0,0,0}))),
            Diagram,
          Documentation(info="<html>

<p>
This is a voltage source with a complex signal input, specifying the complex voltage by the complex RMS voltage components. 
Additionally, the frequency of the voltage source is defined by a real signal input. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput f 
          annotation (extent=[20,80; 60,120], rotation=-90);
        Blocks.Interfaces.ComplexInput V 
          annotation (extent=[-60,80; -20,120], rotation=270);
      equation 
        omega = 2*Modelica.Constants.pi*f;
        v = V;
      end VariableVoltageSource;
    
      model CurrentSource "Constant AC current" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Source;
        parameter Modelica.SIunits.Frequency f(start=1) 
        "frequency of the source";
        parameter Modelica.SIunits.Current I(start=1) 
        "RMS current of the source";
        parameter Modelica.SIunits.Angle phi(start=0) 
        "phase shift of the source";
        annotation (Icon(
            Line(points=[0,-50; 0,50],   style(color=0, rgbcolor={0,0,0})),
            Line(points=[-60,60; 60,60],   style(color=3, rgbcolor={0,0,255})),
            Polygon(points=[60,60; 30,70; 30,50; 60,60],
              style(color=3, rgbcolor={0,0,255}, fillColor=3, rgbfillColor={0,0,255}))),
          Documentation(info="<html>

<p>
This is a constant current source, specifying the complex current by the RMS current and the phase shift.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
      equation 
        omega = 2*Modelica.Constants.pi*f;
        i.re = I*cos(phi);
        i.im = I*sin(phi);
      end CurrentSource;
    
      model VariableCurrentSource "Variable AC current" 
        extends Modelica_QuasiStationary.SinglePhase.Interfaces.Source;
        annotation (Icon(
            Line(points=[0,-50; 0,50],   style(color=0, rgbcolor={0,0,0})),
            Line(points=[-60,60; 60,60],   style(color=3, rgbcolor={0,0,255})),
            Polygon(points=[60,60; 30,70; 30,50; 60,60],
              style(color=3, rgbcolor={0,0,255}, fillColor=3, rgbfillColor={0,0,255}))),
            Diagram,
        Documentation(info="<html>

<p>
This is a current source with a complex signal input, specifying the complex current by the complex RMS current components. 
Additionally, the frequency of the voltage source is defined by a real signal input.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">CurrentSource</a>,
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput f 
          annotation (extent=[20,80; 60,120], rotation=-90);
        Blocks.Interfaces.ComplexInput I 
          annotation (extent=[-60,80; -20,120], rotation=270);
      equation 
        omega = 2*Modelica.Constants.pi*f;
        i = I;
      end VariableCurrentSource;
      annotation (Icon(
          Line(points=[-100,-40; -60,-40]),
          Ellipse(extent=[-60,10; 40,-90],   style(color=3, fillColor=7)),
          Line(points=[40,-40; 80,-40])), Documentation(info="<html>
<p>
This package contains voltage and current sources.
</p>
</html>"));
    end Sources;
    annotation (Icon(Rectangle(extent=[-60,10; 40,-90], style(color=3, rgbcolor=
               {0,0,255})), Rectangle(extent=[-30,-20; 10,-60], style(
            color=0,
            rgbcolor={0,0,0},
            gradient=2,
            fillColor=3,
            rgbfillColor={0,0,255}))), Documentation(info="<html>
<p>This package hosts models for quasi stationary single phase circuits. 
Quasi stationary theory for single phase circuits can be found in the
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">references</a>.
</p>
<h4>See also</h4>

<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase\">MultiPhase</a>

</html>"));
  end SinglePhase;


  package MultiPhase 
    package Basic 
      extends Modelica.Icons.Library2;
      annotation (Icon(
          Line(points=[-100,-40; -80,-40]),
          Line(points=[60,-40; 80,-40]),
          Rectangle(extent=[-80,-10; 60,-70],  style(
              color=3,
              fillColor=7,
              fillPattern=1))));
    
      model Star "Star connection" 
        parameter Integer m(final min=1) = 3 "number of phases";
        Interfaces.PositivePlug plug_p(final m=m) 
          annotation (extent=[-110, -10; -90, 10]);
        SinglePhase.Interfaces.NegativePin pin_n 
          annotation (extent=[90, -10; 110, 10]);
        annotation (Icon(
            Text(extent=[-150,60; 150,120],  string="%name"),
            Line(points=[80, 0; 0, 0], style(thickness=2, fillPattern=1)),
            Line(points=[0, 0; -39, 68], style(thickness=2, fillPattern=1)),
            Line(points=[0, 0; -38, -69], style(thickness=2, fillPattern=1)),
            Text(extent=[-100,-110; 100,-70],   string="m=%m",
              style(color=0, rgbcolor={0,0,0})),
            Line(points=[-90,0; -40,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[80,0; 90,0], style(color=3, rgbcolor={0,0,255}))), Diagram,
        Documentation(info="<html>
<p>
Star (wye) connection of a multi phase circuit. The potentials at the star points are the same. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Delta>Delta</a>
</p> 
</html>"));
        PlugToPins_p plugToPins_p annotation (extent=[-80,-10; -60,10]);
      equation 
        for j in 1:m loop
          connect(plugToPins_p.pin_p[j], pin_n);
        end for;
        connect(plug_p, plugToPins_p.plug_p) 
          annotation (points=[-100,0; -72,0], style(color=58, rgbcolor={0,127,0}));
      end Star;
    
      model Delta "Delta (polygon) connection" 
        parameter Integer m(final min=2) = 3 "number of phases";
        Interfaces.PositivePlug plug_p(final m=m) 
          annotation (extent=[-110, -10; -90, 10]);
        Interfaces.NegativePlug plug_n(final m=m) 
          annotation (extent=[90, -10; 110, 10]);
        annotation (Icon(
            Text(extent=[-150,60; 150,120],  string="%name",
              style(color=3, rgbcolor={0,0,255})),
            Line(points=[-40, 68; -40, -70; 79, 0; -40, 68; -40, 67], style(
                  thickness=2, fillPattern=1)),
            Text(extent=[-100,-110; 100,-70],   string="m=%m",
              style(color=0, rgbcolor={0,0,0})),
            Line(points=[-90,0; -40,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[80,0; 90,0], style(color=3, rgbcolor={0,0,255}))), Diagram,
        Documentation(info="<html>
<p>
Delta (polygon) connection of a multi phase circuit. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Star>Star</a>
</p> 
</html>"));
      
        PlugToPins_p plugToPins_p annotation (extent=[-80,-10; -60,10]);
        PlugToPins_n plugToPins_n annotation (extent=[80,-10; 60,10]);
      equation 
        for j in 1:m loop
          if j<m then
            connect(plugToPins_p.pin_p[j], plugToPins_n.pin_n[j+1]);
          else
            connect(plugToPins_p.pin_p[j], plugToPins_n.pin_n[1]);
          end if;
        end for;
        connect(plug_p, plugToPins_p.plug_p) 
          annotation (points=[-100,0; -72,0], style(color=58, rgbcolor={0,127,0}));
        connect(plugToPins_n.plug_n, plug_n) 
          annotation (points=[72,0; 100,0], style(color=58, rgbcolor={0,127,0}));
      end Delta;
    
      model PlugToPin_p "Connect one (positive) pin" 
        parameter Integer m(final min=1) = 3 "number of phases";
        parameter Integer k(
          final min=1,
          final max=m) = 1 "phase index";
        Interfaces.PositivePlug plug_p(final m=m) 
          annotation (extent=[-30,-10; -10,10]);
        SinglePhase.Interfaces.PositivePin pin_p 
          annotation (extent=[10,-10; 30,10]);
        annotation (Icon(
            Rectangle(extent=[-20,20; 40,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=30,
                rgbfillColor={215,215,215},
                fillPattern=1)),
            Ellipse(extent=[-40,20; 0,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=30,
                rgbfillColor={215,215,215},
                fillPattern=1)),
            Text(extent=[-100,100; 100,40],string="%name"),
            Text(extent=[-100,-60; 100,-100],string="k = %k",
              style(color=0, rgbcolor={0,0,0}))), Diagram,
        Documentation(info="<html>
<p>
Connects the single phase (positive) pin <i>k</i> of the multi phase (positive) plug to a single phase (positive) pin. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_n>PlugToPin_n</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_p>PlutToPins_p</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_n>PlugToPins_n</a>
</p> 
</html>"));
      equation 
        Connections.branch(plug_p.reference, pin_p.reference);
        plug_p.reference.gamma = pin_p.reference.gamma;
        pin_p.v = plug_p.pin[k].v;
        for j in 1:m loop
          plug_p.pin[j].i.re = if j == k then -pin_p.i.re else 0;
          plug_p.pin[j].i.im = if j == k then -pin_p.i.im else 0;
        end for;
      end PlugToPin_p;
    
      model PlugToPin_n "Connect one (negative) pin" 
        parameter Integer m(final min=1) = 3 "number of phases";
        parameter Integer k(
          final min=1,
          final max=m) = 1 "phase index";
        Interfaces.NegativePlug plug_n(final m=m) 
          annotation (extent=[-30,-10; -10,10]);
        SinglePhase.Interfaces.NegativePin pin_n 
          annotation (extent=[10,-10; 30,10]);
        annotation (Icon(
            Rectangle(extent=[-20,20; 40,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=30,
                rgbfillColor={215,215,215},
                fillPattern=1)),
            Ellipse(extent=[-40,20; 0,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=30,
                rgbfillColor={215,215,215},
                fillPattern=1)),
            Text(extent=[-100,100; 100,40],string="%name"),
            Text(extent=[-100,-60; 100,-100],string="k = %k",
              style(color=0, rgbcolor={0,0,0}))), Diagram,
        Documentation(info="<html>
<p>
Connects the single phase (negative) pin <i>k</i> of the multi phase (negative) plug to a single phase (negative) pin. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_p>PlugToPin_p</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_p>PlutToPins_p</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_n>PlugToPins_n</a>
</p> 
</html>"));
      equation 
        Connections.branch(plug_n.reference, pin_n.reference);
        plug_n.reference.gamma = pin_n.reference.gamma;
        pin_n.v = plug_n.pin[k].v;
        for j in 1:m loop
          plug_n.pin[j].i.re = if j == k then -pin_n.i.re else 0;
          plug_n.pin[j].i.im = if j == k then -pin_n.i.im else 0;
        end for;
      end PlugToPin_n;
    
      model PlugToPins_p "Connect all (positive) pins" 
        parameter Integer m(final min=1) = 3 "number of phases";
        Interfaces.PositivePlug plug_p(final m=m) 
          annotation (extent=[-30,-10; -10,10]);
        SinglePhase.Interfaces.PositivePin pin_p[m] 
          annotation (extent=[10,-10; 30,10]);
        annotation (Icon(
            Rectangle(extent=[-20,20; 40,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=31,
                rgbfillColor={170,255,255},
                fillPattern=1)),
            Ellipse(extent=[-40,20; 0,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=31,
                rgbfillColor={170,255,255},
                fillPattern=1)),
            Text(extent=[-100,100; 100,40],string="%name")), Diagram,
        Documentation(info="<html>
<p>
Connects all <i>m</i> single phase (positive) pins of the multi phase (positive) plug to an array of <i>m</i> single phase (positive) pins. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_p>PlugToPin_p</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_n>PlugToPin_n</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_n>PlugToPins_n</a>
</p> 
</html>"));
      equation 
        for j in 1:m loop
          Connections.branch(plug_p.reference, pin_p[j].reference);
          plug_p.reference.gamma = pin_p[j].reference.gamma;
          plug_p.pin[j].v = pin_p[j].v;
          plug_p.pin[j].i.re = -pin_p[j].i.re;
          plug_p.pin[j].i.im = -pin_p[j].i.im;
        end for;
      end PlugToPins_p;
    
      model PlugToPins_n "Connect all (negative) pins" 
        parameter Integer m(final min=1) = 3 "number of phases";
        Interfaces.NegativePlug plug_n(final m=m) 
          annotation (extent=[-30,-10; -10,10]);
        SinglePhase.Interfaces.NegativePin pin_n[m] 
          annotation (extent=[10,-10; 30,10]);
        annotation (Icon(
            Rectangle(extent=[-20,20; 40,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=31,
                rgbfillColor={170,255,255},
                fillPattern=1)),
            Ellipse(extent=[-40,20; 0,-20], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=31,
                rgbfillColor={170,255,255},
                fillPattern=1)),
            Text(extent=[-100,100; 100,40],string="%name")), Diagram,
        Documentation(info="<html>
<p>
Connects all <i>m</i> single phase (negative) pins of the multi phase (negative) plug to an array of <i>m</i> single phase (negative) pins. 
</p>
<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_p>PlugToPin_p</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPin_n>PlugToPin_n</a>,
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_p>PlugToPins_p</a>
</p> 
</html>"));
      equation 
        for j in 1:m loop
          Connections.branch(plug_n.reference, pin_n[j].reference);
          plug_n.reference.gamma = pin_n[j].reference.gamma;
          plug_n.pin[j].v = pin_n[j].v;
          plug_n.pin[j].i.re = -pin_n[j].i.re;
          plug_n.pin[j].i.im = -pin_n[j].i.im;
        end for;
      end PlugToPins_n;
    
      model Resistor "Multiphase linear resistor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Resistance R_ref[m](start=fill(1,m));
        parameter Modelica.SIunits.Temperature T_ref[m]=fill(293.15,m) 
        "Reference temperatures";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref[m]=zeros(m) 
        "Temperature coefficient of resistance (R_actual = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        extends 
        MoveToModelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort(        final mh=m, T=T_ref);
        annotation (Icon(
            Text(extent=[100,60; -100,100], string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Text(extent=[100,-80; -100,-40],
              string="m=%m",
              style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=0,
                rgbfillColor={0,0,0}))),
            Diagram,
        Documentation(info="<html>
<p>
The linear resistor connects the complex voltages <i><u>v</u></i> with the complex
currents <i><u>i</u></i> by <i><u>i</u>*R = <u>v</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>single phase Resistors</a>. 
</p>

<p>
The resistor model also has <i>m</i> optional 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort\">conditional heat ports</a>. 
A linear temperature dependency of the resistances for enabled heat ports is also taken into account.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        SinglePhase.Basic.Resistor resistor[m](
          final R_ref=R_ref,
          final T_ref=T_ref,
          final alpha_ref=alpha_ref,
          each final useHeatPort=useHeatPort,
          final T=T) 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(plugToPins_p.pin_p, resistor.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(resistor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(resistor.heatPort, heatPort) 
          annotation (points=[0,-10; 0,-100], style(color=42, rgbcolor={191,0,0}));
      end Resistor;
    
      model Conductor "Multiphase linear conductor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Conductance G_ref[m](start=fill(1,m));
        parameter Modelica.SIunits.Temperature T_ref[m]=fill(293.15,m) 
        "Reference temperatures";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref[m]=zeros(m) 
        "Temperature coefficient of conductance (G_actual = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        extends 
        MoveToModelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort(        final mh=m, T=T_ref);
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Text(extent=[100,-80; -100,-40],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),           Diagram,
        Documentation(info="<html>
<p>
The linear resistor connects the complex currents <i><u>i</u></i> with the complex
voltages <i><u>v</u></i> by <i><u>v</u>*G = <u>i</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>single phase Conductors</a>.
</p>

<p>
The conductor model also has <i>m</i> optional 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort\">conditional heat ports</a>. 
A linear temperature dependency of the conductances for enabled heat ports is also taken into account.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        SinglePhase.Basic.Conductor conductor[m](
          final G_ref=G_ref,
          final T_ref=T_ref,
          final alpha_ref=alpha_ref,
          each final useHeatPort=useHeatPort,
          final T=T) 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(plugToPins_p.pin_p, conductor.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(conductor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(conductor.heatPort, heatPort) 
          annotation (points=[0,-10; 0,-100], style(color=42, rgbcolor={191,0,0}));
      end Conductor;
    
      model Capacitor "Multiphase linear capacitor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Capacitance C[m](start=fill(1,m));
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Line(points=[-14,28; -14,-28],   style(thickness=2)),
            Line(points=[14,28; 14,-28],   style(thickness=2)),
            Line(points=[-90,0; -14,0]),
            Line(points=[14,0; 90,0]),
            Text(extent=[100,-80; -100,-40],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
            Diagram,
        Documentation(info="<html>
<p>
The linear capacitor connects the complex currents <i><u>i</u></i> with the complex
voltages <i><u>v</u></i> by <i><u>v</u>*j*&omega;*C = <u>i</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>single phase Capacitors</a>.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        SinglePhase.Basic.Capacitor capacitor[m](final C=C) 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(plugToPins_p.pin_p, capacitor.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(capacitor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
      end Capacitor;
    
      model Inductor "Multiphase linear inductor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Inductance L[m](start=fill(1,m));
        annotation (Icon(
            Text(extent=[100,60; -100,100],   string="%name"),
            Ellipse(extent=[-60,-15; -30,15]),
            Ellipse(extent=[-30,-15; 0,15]),
            Ellipse(extent=[0,-15; 30,15]),
            Ellipse(extent=[30,-15; 60,15]),
            Rectangle(extent=[-60,-30; 60,0], style(color=7, fillColor=7)),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Text(extent=[100,-80; -100,-40],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
            Diagram,
        Documentation(info="<html>
<p>
The linear inductor connects the complex voltages <i><u>v</u></i> with the complex
currents <i><u>i</u></i> by <i><u>i</u>*j*&omega;*L = <u>v</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>single phase Inductors</a>.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        SinglePhase.Basic.Inductor inductor[m](final L=L) 
          annotation (extent=[-10,-10; 10,10]);
      equation 
      
        connect(plugToPins_p.pin_p, inductor.pin_p) annotation (points=[-68,0;
              -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(inductor.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,
            0; 39,2.44921e-016; 68,2.44921e-016],      style(color=58, rgbcolor=
               {0,127,0}));
      end Inductor;
    
      model VariableResistor "Multiphase variable resistor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Temperature T_ref[m]=fill(293.15,m) 
        "Reference temperatures";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref[m]=zeros(m) 
        "Temperature coefficient of resistance (R_actual = R_ref*(1 + alpha_ref*(heatPort.T - T_ref))";
        extends 
        MoveToModelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort(        final mh=m, T=T_ref);
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Line(points=[0,90; 0,30], style(
                color=3,
                rgbcolor={0,0,255},
                smooth=0)),
            Text(extent=[100,40; -100,80],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
                             Diagram,
        Documentation(info="<html>
<p>
The linear resistor connects the complex voltages <i><u>v</u></i> with the complex
currents <i><u>i</u></i> by <i><u>i</u>*R = <u>v</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>single phase variable Resistors</a>. 
The resistances <i>R</i> are given as <i>m</i> input signals.
</p>

<p>
The resistor model also has <i>m</i> optional 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort\">conditional heat ports</a>. 
A linear temperature dependency of the resistances for enabled heat ports is also taken into account.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableResistor>VariableResistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput R_ref[m] 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
        SinglePhase.Basic.VariableResistor variableResistor[m](
          final T_ref=T_ref,
          final alpha_ref=alpha_ref,
          each final useHeatPort=useHeatPort,
          final T=T) annotation (extent=[-10,-10; 10,10]);
      equation 
      
        connect(variableResistor.R_ref, R_ref) 
          annotation (points=[0,11; 0,110], style(color=74, rgbcolor={0,0,127}));
        connect(variableResistor.pin_p, plugToPins_p.pin_p) 
          annotation (points=[-10,0; -68,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableResistor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(variableResistor.heatPort, heatPort) 
          annotation (points=[0,-10; 0,-100], style(color=42, rgbcolor={191,0,0}));
      end VariableResistor;
    
      model VariableConductor "Multiphase variable conductor" 
        extends Interfaces.TwoPlug;
        parameter Modelica.SIunits.Temperature T_ref[m]=fill(293.15,m) 
        "Reference temperatures";
        parameter MoveToModelica.SIunits.LinearTemperatureCoefficient alpha_ref[m]=zeros(m) 
        "Temperature coefficient of resistance (G_actual = G_ref/(1 + alpha_ref*(heatPort.T - T_ref))";
        extends 
        MoveToModelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort(        final mh=m, T=T_ref);
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Rectangle(extent=[-70,30; 70,-30],
              style(color=3, fillColor=7, fillPattern=1)),
            Line(points=[0,90; 0,30], style(
                color=3,
                rgbcolor={0,0,255},
                smooth=0)),
            Text(extent=[100,40; -100,80],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
          Diagram,
        Documentation(info="<html>
<p>
The linear resistor connects the complex currents <i><u>i</u></i> with the complex
voltages <i><u>v</u></i> by <i><u>v</u>*G = <u>i</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>single phase variable Conductors</a>. 
The conductances <i>G</i> are given as <i>m</i> input signals.
</p>

<p>
The conductor model also has <i>m</i> optional 
<a href=\"Modelica://Modelica.Electrical.MultiPhase.Interfaces.ConditionalHeatPort\">conditional heat ports</a>. 
A linear temperature dependency of the conductances for enabled heat ports is also taken into account.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableConductor>VariableConductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput G_ref[m] 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
        SinglePhase.Basic.VariableConductor variableResistor[m](
          final T_ref=T_ref,
          final alpha_ref=alpha_ref,
          each final useHeatPort=useHeatPort,
          final T=T) annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(variableResistor.pin_p, plugToPins_p.pin_p) 
          annotation (points=[-10,0; -68,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableResistor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(variableResistor.heatPort, heatPort) 
          annotation (points=[0,-10; 0,-100], style(color=42, rgbcolor={191,0,0}));
        connect(G_ref, variableResistor.G_ref) 
          annotation (points=[0,110; 0,11], style(color=74, rgbcolor={0,0,127}));
      end VariableConductor;
    
      model VariableCapacitor "Multiphase variable capacitor" 
        extends Interfaces.TwoPlug;
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Line(points=[-14,28; -14,-28],   style(thickness=2)),
            Line(points=[14,28; 14,-28],   style(thickness=2)),
            Line(points=[-90,0; -14,0]),
            Line(points=[14,0; 90,0]),
            Line(points=[0,90; 0,30],   style(color=73)),
            Text(extent=[100,40; -100,80],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
            Diagram,
        Documentation(info="<html>
<p>
The linear capacitor connects the complex currents <i><u>i</u></i> with the complex
voltages <i><u>v</u></i> by <i><u>v</u>*j*&omega;*C = <u>i</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>single phase variable Capacitors</a>. 
The capacitances <i>C</i> are given as <i>m</i> input signals.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableCapacitor>VariableCapacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableInductor>Variable inductor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput C[m] 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
        SinglePhase.Basic.VariableCapacitor variableCapacitor[m] 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(C, variableCapacitor.C) 
          annotation (points=[0,110; 0,11], style(color=74, rgbcolor={0,0,127}));
        connect(variableCapacitor.pin_p, plugToPins_p.pin_p) 
          annotation (points=[-10,0; -68,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableCapacitor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
      end VariableCapacitor;
    
      model VariableInductor "Multiphase variable inductor" 
        extends Interfaces.TwoPlug;
        annotation (Icon(
            Text(extent=[100,-80; -100,-40],  string="%name"),
            Ellipse(extent=[-60,-15; -30,15]),
            Ellipse(extent=[-30,-15; 0,15]),
            Ellipse(extent=[0,-15; 30,15]),
            Ellipse(extent=[30,-15; 60,15]),
            Rectangle(extent=[-60,-30; 60,0], style(color=7, fillColor=7)),
            Line(points=[60,0; 90,0]),
            Line(points=[-90,0; -60,0]),
            Line(points=[0,90; 0,8],   style(color=73)),
            Text(extent=[100,40; -100,80],
              string="m=%m",
              style(color=0, rgbcolor={0,0,0}))),
            Diagram,
        Documentation(info="<html>
<p>
The linear inductor connects the complex voltages <i><u>v</u></i> with the complex
currents <i><u>i</u></i> by <i><u>i</u>*j*&omega;*L = <u>v</u></i>, 
using <i>m</i> <a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.VariableInductor>single phase variable Inductors</a>. 
The inductances <i>L</i> are given as <i>m</i> input signals.
</p>

<h4>See also</h4>
<p>
<a href=Modelica://Modelica_QuasiStationary.SinglePhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Resistor>Resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Conductor>Conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Capacitor>Capacitor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.Inductor>Inductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableResistor>Variable resistor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableConductor>Variable conductor</a>, 
<a href=Modelica://Modelica_QuasiStationary.MultiPhase.Basic.VariableCapacitor>Variable capacitor</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput L[m] 
          annotation (extent=[-20,90; 20,130],   rotation=-90);
        SinglePhase.Basic.VariableInductor variableInductor[m] 
          annotation (extent=[-10,-10; 10,10]);
      equation 
        connect(variableInductor.pin_p, plugToPins_p.pin_p) 
          annotation (points=[-10,0; -68,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableInductor.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(variableInductor.L, L) 
          annotation (points=[0,10.8; 0,110],
                                            style(color=74, rgbcolor={0,0,127}));
      end VariableInductor;
    end Basic;
    extends Modelica.Icons.Library2;
  
    annotation (Icon(
        Ellipse(extent=[-60,10; 40,-90], style(color=3, rgbcolor={0,0,255})),
        Ellipse(extent=[-40,-14; -20,-34], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=3,
            rgbfillColor={0,0,255})),
        Ellipse(extent=[0,-14; 20,-34], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=3,
            rgbfillColor={0,0,255})),
        Ellipse(extent=[-20,-54; 0,-74], style(
            color=3,
            rgbcolor={0,0,255},
            fillColor=3,
            rgbfillColor={0,0,255}))), Documentation(info="<html>
<p>This package hosts models for quasi stationary multi phase circuits. 
Quasi stationary theory can be found in 
[<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">Vaske1973</a>]
and other
<a href=\"Modelica://Modelica_QuasiStationary.UsersGuide.References\">references</a>.
</p>
<h4>See also</h4>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase\">SinglePhase</a>
 
</html>"));
    package Ideal 
      extends Modelica.Icons.Library2;
    
    annotation (Icon(
           Line(points=[-100,-60; -54,-60]),
           Ellipse(extent=[-54,-56; -46,-64]),
           Line(points=[-47,-58; 30,-10]),
           Line(points=[30,-40; 30,-60]),
           Line(points=[30,-60; 80,-60])));
      model Idle "Idle branch" 
        extends Interfaces.TwoPlug;
      
      SinglePhase.Ideal.Idle idle[m] annotation (extent=[-10,-10; 10,10]);
      annotation (
        Diagram,
        Icon(
            Rectangle(extent=[-60,60; 60,-60],   style(fillColor=7)),
            Line(points=[-90,0; -41,0]),
            Line(points=[91,0; 40,0]),
            Text(extent=[-100,100; 100,70],   string="%name")),
        Documentation(info="<html>
<p>
This model describes <i>m</m> simple idle branches considering the complex currents <i><u>i</u></i> = 0; 
it uses <i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Idle\">single phase idle branches</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Idle\">Idle</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Ideal.Short\">Short</a>
</p>
</html>"));
      equation 
      connect(plugToPins_p.pin_p, idle.pin_p) annotation (points=[-68,0; -10,0],
          style(color=58, rgbcolor={0,127,0}));
      connect(idle.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,0;
            39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,
              0}));
      end Idle;
    
      model Short "Short cut branch" 
        extends Interfaces.TwoPlug;
      
      SinglePhase.Ideal.Short short[m] annotation (extent=[-10,-10; 10,10]);
      annotation (
        Diagram,
        Icon(
            Rectangle(extent=[-60,60; 60,-60],   style(fillColor=7)),
            Text(extent=[-100,100; 100,70],   string="%name"),
            Line(points=[91,0; -90,0])),
        Documentation(info="<html>
<p>
This model describes <i>m</m> simple short branches considering the complex voltages <i><u>v</u></i> = 0; 
it uses <i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Short\">single phase short branches</a>.
</p>

<h4>See also</h4>
<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Ideal.Short\">Short</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Ideal.Idle\">Idle</a>
</p>
</html>"));
      equation 
      connect(plugToPins_p.pin_p, short.pin_p) annotation (points=[-68,0; -10,0],
          style(color=58, rgbcolor={0,127,0}));
      connect(short.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,0;
            39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,
              0}));
      end Short;
    end Ideal;
  
    package Interfaces 
      extends Modelica.Icons.Library2;
      annotation (Icon(
          Ellipse(extent=[-60,10; 40,-90], style(color=3, rgbcolor={0,0,255})),
          Ellipse(extent=[-40,-14; -20,-34], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=3,
              rgbfillColor={0,0,255})),
          Ellipse(extent=[0,-14; 20,-34], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=3,
              rgbfillColor={0,0,255})),
          Ellipse(extent=[-20,-54; 0,-74], style(
              color=3,
              rgbcolor={0,0,255},
              fillColor=3,
              rgbfillColor={0,0,255}))));
    
      connector Plug "Basic multiphase plug" 
        parameter Integer m=3 "number of phases";
        SinglePhase.Interfaces.Pin pin[m];
        annotation (Documentation(info="<html>

<p>
This multiphase plug contains a vector of <i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">single phase pins</a>.
The <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">positive</a> and
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">negative plug</a> are 
derived from this base connector.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
      end Plug;
    
      connector PositivePlug "Positive multiphase connector" 
        extends Plug;
        Types.Reference reference;
        annotation (Icon(Ellipse(extent=[-100,100; 100,-100], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=58,
                rgbfillColor={0,127,0},
                fillPattern=1))), Diagram(Ellipse(extent=[-40,40; 40,-40], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=58,
                rgbfillColor={0,127,0},
                fillPattern=1)),          Text(
              extent=[-100,100; 100,60],
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=3,
                rgbfillColor={0,0,255},
                fillPattern=1),
              string="%name")),
        Documentation(info="<html>

<p>
The positive plug is based on <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>. 
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of each quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">negative plug</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>
</p>
</html>"));
      end PositivePlug;
    
      connector NegativePlug "Negative multiphase connector" 
        extends Plug;
        Types.Reference reference;
        annotation (Icon(Ellipse(extent=[-100,100; 100,-100], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))), Diagram(Ellipse(extent=[-40,40; 40,-40], style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),          Text(
              extent=[-100,100; 100,60],
              style(
                color=3,
                rgbcolor={0,0,255},
                fillColor=3,
                rgbfillColor={0,0,255},
                fillPattern=1),
              string="%name")),
        Documentation(info="<html>

<p>
The negative plug is based on <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>. 
Additionally the reference angle is specified in the connector. The time derivative of the reference angle is the actual angluar velocity of each quasi stationary voltage and current. The symbol is also designed such way to look different than the <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">positive plug</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Pin\">Pin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.PositivePin\">PositivePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.NegativePin\">NegativePin</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.Plug\">Plug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
</p>
</html>"));
      end NegativePlug;
    
      partial model TwoPlug "Two plugs with pin-adapter" 
        parameter Integer m(min=1) = 3 "number of phases";
        Types.ComplexVoltage v[m];
        Types.ComplexCurrent i[m];
        Modelica.SIunits.AngularVelocity omega = der(plug_p.reference.gamma);
        PositivePlug plug_p(final m=m) 
          annotation (extent=[-110,-10; -90,10]);
        NegativePlug plug_n(final m=m) 
          annotation (extent=[90,-10; 110,10]);
        Basic.PlugToPins_p plugToPins_p(final m=m) 
          annotation (extent=[-80,-10; -60,10]);
        Basic.PlugToPins_n plugToPins_n(final m=m) 
          annotation (extent=[60,-10; 80,10], rotation=180);
        annotation (Diagram, Documentation(info="<html>
<p>
This partial model uses a <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">positive</a>
and <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">negative plug</a> and defines the complex voltage differences as well as the complex currents (into the positive plug). A <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_p\">positive</a> and 
a <a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Basic.PlugToPins_n\">negative adapter</a> are used to give easy acces to the single pins of both plugs. Additionally, the angular velocity of the quasi stationary system is explicitely defined as variable. This model is mainly intended to be used with graphical representation of user models.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">PositivePlug</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.NegativePlug\">NegativePlug</a>,
</p>
</html>"));
      equation 
        v.re = plug_p.pin.v.re - plug_n.pin.v.re;
        v.im = plug_p.pin.v.im - plug_n.pin.v.im;
        i = plug_p.pin.i;
        connect(plug_p, plugToPins_p.plug_p) 
          annotation (points=[-100,0; -72,0], style(color=58, rgbcolor={0,127,0}));
        connect(plugToPins_n.plug_n, plug_n) 
          annotation (points=[72,-2.44921e-016; 86,-2.44921e-016; 86,0; 100,0], style(color=58, rgbcolor={0,127,0}));
      end TwoPlug;
    
      partial model AbsoluteSensor "Partial potential sensor" 
        extends Modelica.Icons.RotationalSensor;
        parameter Integer m(min=1) = 3 "number of phases";
        Modelica.SIunits.AngularVelocity omega = der(plug_p.reference.gamma);
        PositivePlug plug_p(final m=m) 
          annotation (extent=[-110, -10; -90, 10]);
        annotation (Diagram, Icon(
          Line(points=[-70,0; -94,0], style(color=0)),
            Text(
              extent=[-100,100; 100,70],
              string="%name",
              style(
                pattern=0,
                fillColor=79,
                rgbfillColor={170,85,255},
                fillPattern=1)),
            Line(points=[70,0; 80,0; 90,0; 100,0],   style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(extent=[100,-100; -100,-70],
              string="m=%m",
              style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=0,
                rgbfillColor={0,0,0}))),
        Documentation(info="<html>

<p>
The absolute sensor partial model relies on the a 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.PositivePlug\">positive plug</a> to measure the complex potential. Additionally this model contains a proper icon and a definition of the angular velocity. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.RelativeSensor\">RelativeSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.AbsoluteSensor\">SinglePhase.Interfaces.AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.RelativeSensor\">SinglePhase.Interfaces.RelativeSensor</a>
</p>

</html>"));
        Blocks.Interfaces.ComplexOutput y[m] 
          annotation (extent=[100,-10; 120,10]);
        Basic.PlugToPins_p plugToPins_p(final m=m) 
          annotation (extent=[-80,-10; -60,10]);
      equation 
        connect(plug_p, plugToPins_p.plug_p) 
            annotation (points=[-100,0; -72,0], style(
            color=58,
            rgbcolor={0,127,0},
            smooth=0));
      end AbsoluteSensor;
    
      partial model RelativeSensor "Partial voltage / current sensor" 
        extends Modelica.Icons.RotationalSensor;
        extends TwoPlug;
        annotation (Diagram, Icon(
            Line(points=[-70,0; -94,0],   style(color=0)),
            Line(points=[70,0; 94,0],   style(color=0)),
            Text(
              extent=[-100,100; 100,70],
              string="%name",
              style(
                pattern=0,
                fillColor=79,
                rgbfillColor={170,85,255},
                fillPattern=1)),
            Line(points=[0,-70; 0,-80; 0,-90; 0,-100],   style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1)),
            Text(extent=[100,-100; -100,-70],
              string="m=%m",
              style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=0,
                rgbfillColor={0,0,0}))),
        Documentation(info="<html>
<p>
The relative sensor partial model relies on the 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.TwoPlug\">TwoPlug</a> to measure the complex voltages, currents or power. Additionally this model contains a proper icon and a definition of the angular velocity. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.AbsoluteSensor\">AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.AbsoluteSensor\">SinglePhase.Interfaces.AbsoluteSensor</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.RelativeSensor\">SinglePhase.Interfaces.RelativeSensor</a>
</p>

</html>"));
        Blocks.Interfaces.ComplexOutput y[m] 
          annotation (extent=[-10,-120; 10,-100], rotation=-90);
      end RelativeSensor;
    
      partial model Source "Partial voltage / current source" 
        extends TwoPlug;
        constant Modelica.SIunits.Angle pi=Modelica.Constants.pi;
        annotation (Icon(
            Ellipse(extent=[-50,50; 50,-50], style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=7,
                rgbfillColor={255,255,255})),
            Text(extent=[100,-100; -100,-60], string="%name"),
            Line(points=[-90,0; -50,0],style(color=0, rgbcolor={0,0,0})),
            Line(points=[50,0; 90,0],  style(color=0, rgbcolor={0,0,0})),
            Text(extent=[100,60; -100,100],
              string="m=%m",
              style(
                color=0,
                rgbcolor={0,0,0},
                fillColor=0,
                rgbfillColor={0,0,0}))), Diagram,
        Documentation(info="<html>
<p>
The source partial model relies on the 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Interfaces.TwoPlug\">TwoPlug</a> and contains a proper icon. 
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MutliPhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Interfaces.Source\">SinglePhase.Interfaces.Source</a>.
</p>
</html>"));
      end Source;
    end Interfaces;
  
    package Sensors 
      extends Modelica.Icons.Library2;
      annotation (Icon(
          Ellipse(extent=[-60,10; 40,-90], style(
              color=0,
              rgbcolor={0,0,0},
              fillColor=7,
              rgbfillColor={255,255,255})),
          Line(points=[-50,-16; -36,-25],
                                        style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[-35,0; -25,-14], style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[-10,7; -10,-10],
                                    style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[15,0; 5,-14],  style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Line(points=[30,-15; 16,-25],
                                      style(
              color=0,
              rgbcolor={0,0,0},
              fillPattern=1)),
          Polygon(points=[-12,-24; -0.5,-27; 2,1.5; -12,-24],          style(
              color=0,
              fillColor=0,
              fillPattern=1)),
          Line(points=[-10,-40; -6,-26],  style(color=0)),
          Ellipse(extent=[-15,-35; -5,-45],
                                         style(
              color=0,
              gradient=0,
              fillColor=0,
              fillPattern=1))));
      model PotentialSensor 
        extends Interfaces.AbsoluteSensor;
        SinglePhase.Sensors.PotentialSensor potentialSensor[m] 
          annotation (extent=[-10,-10; 10,10]);
      equation 
      
        annotation (Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="V",
              style(color=0))), Diagram,
        Documentation(info="<html>

<p>
This sensor can be used to measure <i>m</i> complex potentials, using <i>m</i> 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">single phase PotentialSensors</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PotentialSensor\">SinglePhase.PotentialSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.VoltageSensor\">VoltageSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.CurrentSensor\">CurrentSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
        connect(plugToPins_p.pin_p, potentialSensor.pin) annotation (points=[
              -68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(potentialSensor.y, y) annotation (points=[11,0; 110,0], style(
              color=58, rgbcolor={0,127,0}));
      end PotentialSensor;
    
      model VoltageSensor 
        extends Interfaces.RelativeSensor;
        SinglePhase.Sensors.VoltageSensor voltageSensor[m] 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Diagram, Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="V",
              style(color=0))),
        Documentation(info="<html>

<p>
This sensor can be used to measure <i>m</i> complex voltages, using <i>m</i> 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">single phase VoltageSensors</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.VoltageSensor\">SinglePhase.VoltageSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PotentialSensor\">PotentialSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.CurrentSensor\">CurrentSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
      equation 
        connect(plugToPins_p.pin_p, voltageSensor.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(voltageSensor.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,
            0; 39,2.44921e-016; 68,2.44921e-016],   style(color=58, rgbcolor={0,127,0}));
        connect(voltageSensor.y, y) 
          annotation (points=[0,-11; 0,-110], style(color=58, rgbcolor={0,127,0}));
      end VoltageSensor;
    
      model CurrentSensor 
        extends Interfaces.RelativeSensor;
        SinglePhase.Sensors.CurrentSensor currentSensor[m] 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Diagram, Icon(
               Text(
              extent=[-29,-11; 30,-70],
              string="I",
              style(color=0))),
        Documentation(info="<html>

<p>
This sensor can be used to measure <i>m</i> complex currents, using <i>m</i> 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">single phase CurrentSensors</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.CurrentSensor\">SinglePhase.CurrentSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PotentialSensor\">PotentialSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.VoltageSensor\">VoltageSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PowerSensor\">PowerSensor</a>
</p>

</html>"));
      equation 
        connect(plugToPins_p.pin_p,currentSensor. pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(currentSensor.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,
            0; 39,2.44921e-016; 68,2.44921e-016],   style(color=58, rgbcolor={0,127,0}));
        connect(currentSensor.y, y) 
          annotation (points=[0,-11; 0,-110], style(color=58, rgbcolor={0,127,0}));
      end CurrentSensor;
    
      model PowerSensor 
        parameter Integer m(min=1) = 3 "number of phases";
        Modelica.SIunits.AngularVelocity omega = der(currentP.reference.gamma);
        annotation (Diagram, Icon(
            Ellipse(extent=[-70,70; 70,-70],   style(color=0, fillColor=7)),
            Line(points=[0,100; 0,70], style(color=3, rgbcolor={0,0,255})),
            Line(points=[0,-70; 0,-100], style(color=3, rgbcolor={0,0,255})),
            Line(points=[-100,0; 100,0], style(color=3, rgbcolor={0,0,255})),
            Line(points=[0,70; 0,40],   style(color=0)),
            Line(points=[22.9,32.8; 40.2,57.3],   style(color=0)),
            Line(points=[-22.9,32.8; -40.2,57.3],   style(color=0)),
            Line(points=[37.6,13.7; 65.8,23.9],   style(color=0)),
            Line(points=[-37.6,13.7; -65.8,23.9],   style(color=0)),
            Line(points=[0,0; 9.02,28.6],   style(color=0)),
            Polygon(points=[-0.48,31.6; 18,26; 18,57.2; -0.48,31.6],     style(
                color=0,
                fillColor=0,
                fillPattern=1)),
            Ellipse(extent=[-5,5; 5,-5],   style(
                color=0,
                gradient=0,
                fillColor=0,
                fillPattern=1)),
               Text(
              extent=[-29,-11; 30,-70],
              style(color=0),
              string="P"),
            Line(points=[-80,-100; -80,0],   style(
                color=58,
                rgbcolor={0,127,0},
                fillColor=7,
                rgbfillColor={255,255,255},
                fillPattern=1))),
        Documentation(info="<html>

<p>
This sensor can be used to measure <i>m</i> complex apparent power values, using <i>m</i> 
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">single phase PowerSensors</a>.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sensors.PowerSensor\">SinglePhase.PowerSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.PotentialSensor\">PotentialSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.VoltageSensor\">VoltageSensor</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sensors.CurrentSensor\">CurrentSensor</a>
</p>

</html>"));
        Interfaces.PositivePlug currentP 
          annotation (extent=[-110,-10; -90,10]);
        Interfaces.NegativePlug currentN 
          annotation (extent=[90,-10; 110,10]);
        Interfaces.PositivePlug voltageP 
          annotation (extent=[-10,90; 10,110]);
        Interfaces.NegativePlug voltageN 
          annotation (extent=[-10,-110; 10,-90]);
        Blocks.Interfaces.ComplexOutput y 
          annotation (extent=[-90,-120; -70,-100], rotation=-90);
        Basic.PlugToPins_p plugToPinsCurrentP(final m=m) 
          annotation (extent=[-80,-10; -60,10]);
        Basic.PlugToPins_p plugToPinsVoltageP(final m=m) 
          annotation (extent=[-10,60; 10,80], rotation=270);
        Basic.PlugToPins_n plugToPinsCurrentN(final m=m) 
          annotation (extent=[80,-10; 60,10]);
        Basic.PlugToPins_n plugToPinsVoltageN(final m=m) 
          annotation (extent=[10,-80; -10,-60], rotation=90);
        SinglePhase.Sensors.PowerSensor powerSensor[m] 
          annotation (extent=[-10,-10; 10,10]);
        Blocks.Sum sum(final nin=m) 
          annotation (extent=[-90,-80; -70,-60], rotation=270);
      equation 
        connect(plugToPinsCurrentP.plug_p, currentP) 
          annotation (points=[-72,0; -100,0], style(color=58, rgbcolor={0,127,0}));
        connect(currentN, plugToPinsCurrentN.plug_n) 
          annotation (points=[100,0; 72,0], style(color=58, rgbcolor={0,127,0}));
        connect(voltageP, plugToPinsVoltageP.plug_p) annotation (points=[0,100; 0,
            86; 0,72; 3.67382e-016,72],
                                      style(color=58, rgbcolor={0,127,0}));
        connect(plugToPinsVoltageN.plug_n, voltageN) annotation (points=[
            1.22461e-016,-72; 0,-72; 0,-100],
                                   style(color=58, rgbcolor={0,127,0}));
        connect(plugToPinsCurrentP.pin_p, powerSensor.currentP) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(powerSensor.currentN, plugToPinsCurrentN.pin_n) 
          annotation (points=[10,0; 68,0], style(color=58, rgbcolor={0,127,0}));
        connect(powerSensor.voltageP, plugToPinsVoltageP.pin_p) annotation (points=[0,10; 0,
            39; 0,68; -3.67382e-016,68],         style(color=58, rgbcolor={0,127,0}));
        connect(powerSensor.voltageN, plugToPinsVoltageN.pin_n) annotation (points=[0,-10; 0,
            -39; 0,-68; -1.22461e-016,-68],          style(color=58, rgbcolor={0,127,
                0}));
        connect(powerSensor.y, sum.u) annotation (points=[-8,-11; -8,-40; -80,-40;
              -80,-58], style(color=58, rgbcolor={0,127,0}));
        connect(sum.y, y) annotation (points=[-80,-81; -80,-110], style(color=58,
              rgbcolor={0,127,0}));
        connect(currentP, currentP) annotation (points=[-100,0; -100,0; -100,0],
            style(color=58, rgbcolor={0,127,0}));
      end PowerSensor;
    end Sensors;
  
    package Sources 
      extends Modelica.Icons.Library2;
    
      annotation (Icon(
          Line(points=[-100,-40; -60,-40]),
          Ellipse(extent=[-60,10; 40,-90],   style(color=3, fillColor=7)),
          Line(points=[40,-40; 80,-40])));
      model VoltageSource 
        extends Interfaces.Source;
        parameter Modelica.SIunits.Frequency f(start=1) 
        "frequency of the source";
        parameter Modelica.SIunits.Voltage V[m](start=fill(1,m)) 
        "RMS voltage of the source";
        parameter Modelica.SIunits.Angle phi[m]={0 - (j-1)*2*pi/m for j in 1:m} 
        "phase shift of the source";
        SinglePhase.Sources.VoltageSource voltageSource[m](
          each final f=f,
          final V=V,
          final phi=phi) 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Icon(
            Line(points=[50,0; -50,0],   style(color=0, rgbcolor={0,0,0})),
            Text(string="+",
              extent=[-120,50; -20,0], style(color=3, rgbcolor={0,0,255})),
            Text(string="-",
              extent=[20,50; 120,0], style(color=3, rgbcolor={0,0,255}))), Diagram,
        Documentation(info="<html>

<p>
This model describes <i>m</i> constant voltage sources, specifying the complex voltages by the RMS voltages and the phase shifts. 
<i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">single phase VoltageSources</a> are used.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">SinglePhase.VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
      equation 
        connect(plugToPins_p.pin_p, voltageSource.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(voltageSource.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,
            0; 39,2.44921e-016; 68,2.44921e-016],   style(color=58, rgbcolor={0,127,0}));
      end VoltageSource;
    
      model VariableVoltageSource 
        extends Interfaces.Source;
        SinglePhase.Sources.VariableVoltageSource variableVoltageSource[
                                m] 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Icon(
            Line(points=[50,0; -50,0],   style(color=0, rgbcolor={0,0,0})),
            Text(string="+",
              extent=[-120,50; -20,0], style(color=3, rgbcolor={0,0,255})),
            Text(string="-",
              extent=[20,50; 120,0], style(color=3, rgbcolor={0,0,255}))), Diagram,
        Documentation(info="<html>

<p>
This model describes <i>m</i> variable voltage sources, with <i>m</i> complex signal inputs, 
specifying the complex voltages by the complex RMS voltage components. 
Additionally, the frequency of the voltage source is defined by a real signal input. 
<i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableVoltageSource\">single phase VariableVoltageSources</a> are used.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">SinglePhase.VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VoltageSource\">VoltageSource</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.CurrentSource\">CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput f 
          annotation (extent=[20,80; 60,120], rotation=-90);
        Blocks.Interfaces.ComplexInput V[m] 
          annotation (extent=[-60,80; -20,120], rotation=270);
      equation 
        for j in 1:m loop
          connect(f, variableVoltageSource[j].f) 
            annotation (points=[40,100; 40,60; 4,60; 4,10], style(color=74, rgbcolor={0,0,127}));
        end for;
        connect(plugToPins_p.pin_p, variableVoltageSource.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableVoltageSource.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(V, variableVoltageSource.V) 
          annotation (points=[-40,100; -40,60; -4,60; -4,10], style(color=58, rgbcolor={0,127,0}));
      end VariableVoltageSource;
    
      model CurrentSource 
        extends Interfaces.Source;
        parameter Modelica.SIunits.Frequency f(start=1) 
        "frequency of the source";
        parameter Modelica.SIunits.Current I[m](start=fill(1,m)) 
        "RMS current of the source";
        parameter Modelica.SIunits.Angle phi[m]={0 - (j-1)*2*pi/m for j in 1:m} 
        "phase shift of the source";
        SinglePhase.Sources.CurrentSource currentSource[m](
          each final f=f,
          final phi=phi,
          final I=I) 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Icon(
            Line(points=[-60,60; 60,60],   style(color=3, rgbcolor={0,0,255})),
            Polygon(points=[60,60; 30,70; 30,50; 60,60],
              style(color=3, rgbcolor={0,0,255}, fillColor=3, rgbfillColor={0,0,255})),
            Line(points=[0,-50; 0,50],   style(color=0, rgbcolor={0,0,0}))),
                                                                           Diagram,
        Documentation(info="<html>

<p>
This model describes <i>m</i> constant current sources, specifying the complex currents by the RMS currents and the phase shifts. 
<i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">single phase CurrentSources</a> are used.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.CurrentSource\">SinglePhase.CurrentSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VoltageSource\">VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableCurrentSource\">VariableCurrentSource</a>
</p>
</html>"));
      equation 
        connect(plugToPins_p.pin_p,currentSource. pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(currentSource.pin_n, plugToPins_n.pin_n) annotation (points=[10,0; 39,
            0; 39,2.44921e-016; 68,2.44921e-016],   style(color=58, rgbcolor={0,127,0}));
      end CurrentSource;
    
      model VariableCurrentSource 
        extends Interfaces.Source;
        SinglePhase.Sources.VariableCurrentSource variableCurrentSource[
                                m] 
          annotation (extent=[-10,-10; 10,10]);
        annotation (Icon(
            Line(points=[-60,60; 60,60],   style(color=3, rgbcolor={0,0,255})),
            Polygon(points=[60,60; 30,70; 30,50; 60,60],
              style(color=3, rgbcolor={0,0,255}, fillColor=3, rgbfillColor={0,0,255})),
            Line(points=[0,-50; 0,50],   style(color=0, rgbcolor={0,0,0}))),
            Diagram,
        Documentation(info="<html>

<p>
This model describes <i>m</i> variable current sources, with <i>m</i> complex signal inputs, 
specifying the complex current by the complex RMS voltage components. 
Additionally, the frequency of the current source is defined by a real signal input. 
<i>m</i> <a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VariableCurrentSource\">single phase VariableCurrentSources</a> are used.
</p>

<h4>See also</h4>

<p>
<a href=\"Modelica://Modelica_QuasiStationary.SinglePhase.Sources.VoltageSource\">SinglePhase.VoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VoltageSource\">VoltageSource</a>, 
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.VariableVoltageSource\">VariableVoltageSource</a>,
<a href=\"Modelica://Modelica_QuasiStationary.MultiPhase.Sources.CurrentSource\">CurrentSource</a>.
</p>
</html>"));
        Modelica.Blocks.Interfaces.RealInput f 
          annotation (extent=[20,80; 60,120], rotation=-90);
        Blocks.Interfaces.ComplexInput I[m] 
          annotation (extent=[-60,80; -20,120], rotation=270);
      equation 
        for j in 1:m loop
          connect(f, variableCurrentSource[j].f) 
            annotation (points=[40,100; 40,60; 4,60; 4,10], style(color=74, rgbcolor={0,0,127}));
        end for;
        connect(plugToPins_p.pin_p, variableCurrentSource.pin_p) 
          annotation (points=[-68,0; -10,0], style(color=58, rgbcolor={0,127,0}));
        connect(variableCurrentSource.pin_n, plugToPins_n.pin_n) 
          annotation (points=[10,0; 39,0; 39,2.44921e-016; 68,2.44921e-016], style(color=58, rgbcolor={0,127,0}));
        connect(I, variableCurrentSource.I) annotation (points=[-40,100; -40,60; -4,60; -4,10], style(color=58, rgbcolor=
               {0,127,0}));
      end VariableCurrentSource;
    end Sources;
  end MultiPhase;


  package Types "Definiton of complex data and operations" 
    extends Modelica.Icons.Library2;
  
    record Reference 
      Modelica.SIunits.Angle gamma;
      function equalityConstraint 
        input Reference reference1;
        input Reference reference2;
        output Real residue[0];
      algorithm 
        assert(abs(reference1.gamma - reference2.gamma) < 1E-6*2*Modelica.Constants.pi, "Reference angles should be equal!");
      end equalityConstraint;
    end Reference;
  
    record Complex "Record defining a Complex number" 
      Real re "Real part of complex number";
      Real im "Imaginary part of complex number";
    
      annotation (Documentation(info="<html>
<p>
This record defines a complex number consisting of a real
and an imaginary part.
</html>"));
    end Complex;
  
    record ComplexVoltage = Complex (
        redeclare Modelica.SIunits.Voltage re,
        redeclare Modelica.SIunits.Voltage im);
  
    record ComplexCurrent = Complex (
        redeclare Modelica.SIunits.Current re,
        redeclare Modelica.SIunits.Current im);
  
    record ComplexImpedance=Complex (
        redeclare Modelica.SIunits.Resistance re,
        redeclare Modelica.SIunits.Reactance im);
  
    record ComplexAdmittance=Complex (
        redeclare Modelica.SIunits.Conductance re,
        redeclare Modelica.SIunits.Susceptance im);
  
    record ComplexPower=Complex (
        redeclare Modelica.SIunits.ActivePower re,
        redeclare Modelica.SIunits.ReactivePower im);
  end Types;


  package MoveToModelica 
  
    package Electrical 
      package Analog 
        package Interfaces 
        partial model ConditionalHeatPort 
          "Partial model to include a conditional HeatPort in order to describe the power loss via a thermal network" 
          parameter Boolean useHeatPort = false "=true, if HeatPort is enabled"
          annotation(Evaluate=true, HideResult=true);
          parameter Modelica.SIunits.Temperature T=293.15 
            "Fixed device temperature if useHeatPort = false" annotation(Dialog(enable=not useHeatPort));
          Modelica.SIunits.Power LossPower 
            "Loss power leaving component via HeatPort";
          Modelica.SIunits.Temperature T_heatPort "Temperature of HeatPort";
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort(T(start=T)=T_heatPort, Q_flow=-LossPower) if useHeatPort 
            annotation (extent=[-10,-110; 10,-90]);
        equation 
          if not useHeatPort then
             T_heatPort = T;
          end if;
        end ConditionalHeatPort;
        end Interfaces;
      end Analog;
    
      package MultiPhase 
        package Interfaces 
        partial model ConditionalHeatPort 
          "Partial model to include conditional HeatPorts in order to describe the power loss via a thermal network" 
          parameter Integer mh(min=1)=3 "Number of heatPorts=number of phases";
          parameter Boolean useHeatPort = false "=true, if HeatPort is enabled"
          annotation(Evaluate=true, HideResult=true);
          parameter Modelica.SIunits.Temperature T[mh]=fill(293.15, mh);
          Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort[mh] if useHeatPort 
            annotation (extent=[-10,-110; 10,-90]);
          annotation (Diagram);
        end ConditionalHeatPort;
        end Interfaces;
      end MultiPhase;
    end Electrical;
  
    package SIunits 
      type LinearTemperatureCoefficient = Real(final quantity = "LinearTemperatureCoefficient",
        final unit="1/K");
    end SIunits;

    package Math "Definiton of complex data and operations" 
      extends Modelica.Icons.Library2;
    
      annotation (
          version="0.2.0",
          versionDate="2009-10-28",
          Documentation(info="<html>
<p>This library is intended to be integrated into the MSL anytime soon. Until then it serves as a common basis for various libraries, e.g. QuasiStationary, FundamentalWave.</p>
</html>",     revisions="<html>
<table border=1>
 
<thead>
<tr><td>Version</td> <td>Date</td>  <td>Author(s)</td> <td>Comments</td></tr>
</thead>
 
<tbody>
<tr><td>0.1.0</td>  <td>2009-10-26</td>  <td>Christian Kral<br>Anton Haumer</td> <td></td> </tr>
</tbody>
 
</table>
 
</html>"),
        uses(Modelica(version="2.2.2")));
    
      record Complex "Record defining a complex number" 
        Real re "Real part of complex number";
        Real im "Imaginary part of complex number";
        annotation (Documentation(info=
                       "<html>
<p>
This record defines a complex number consisting of a real
and an imaginary part.
</html>"));
      
      end Complex;
    
    connector ComplexSignal = Complex;
    connector ComplexInput = input ComplexSignal 
      annotation (defaultComponentName="u",
      Coordsys(extent=[-100, -100; 100, 100],
        grid=[1,1],
        component=[20,20],
          scale=0.2),
      Icon(coordinateSystem(extent=[-100,-100; 100,100]),
           Polygon(points=[-100,100; 100,0; -100,-100; -100,100], style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=45,
            rgbfillColor={255,128,0})),
        Text(
          extent=[100,101; 101,65],
          style(
            color=45,
            rgbcolor={255,128,0},
            fillColor=7,
            rgbfillColor={255,255,255},
            fillPattern=1),
          string="%name")),
      Diagram(Polygon(points=[0,50; 100,0; 0,-50; 0,50], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=45,
              rgbfillColor={255,128,0})),
          Text(
          extent=[-100,100; 100,60],
          string="%name",
            style(color=45, rgbcolor={255,128,0}))));
    connector ComplexOutput = output ComplexSignal 
      annotation (defaultComponentName="y",
      Coordsys(extent=[-100, -100; 100, 100],
        grid=[1,1],
        component=[20,20]),
      Icon(Polygon(points=[-100,100; 100,0; -100,-100; -100,100], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=7,
              rgbfillColor={255,255,255}))),
      Diagram(Polygon(points=[-100,50; 0,0; -100,-50; -100,50], style(
              color=45,
              rgbcolor={255,128,0},
              fillColor=7,
              rgbfillColor={255,255,255})),
          Text(
          extent=[-100,100; 100,60],
          string="%name",
            style(color=45, rgbcolor={255,128,0}))));
      record ComplexVoltage = Complex (
          redeclare Modelica.SIunits.Voltage re,
          redeclare Modelica.SIunits.Voltage im) "Complex voltage";
    
      record ComplexCurrent = Complex (
          redeclare Modelica.SIunits.Current re,
          redeclare Modelica.SIunits.Current im) "Complex current";
    
      record ComplexImpedance=Complex (
          redeclare Modelica.SIunits.Resistance re,
          redeclare Modelica.SIunits.Reactance im) "Complex impedance";
    
      record ComplexAdmittance=Complex (
          redeclare Modelica.SIunits.Conductance re,
          redeclare Modelica.SIunits.Susceptance im) "Complex admittance";
    
      record ComplexPower=Complex (
          redeclare Modelica.SIunits.ActivePower re,
          redeclare Modelica.SIunits.ReactivePower im) "Complex power";
      record ComplexMagneticFlux = Complex (
        redeclare Modelica.SIunits.MagneticFlux re,
        redeclare Modelica.SIunits.MagneticFlux im) "Complex magnetic flux";
      record ComplexMagneticPotentialDifference = Complex (
        redeclare Modelica.SIunits.MagneticPotentialDifference re,
        redeclare Modelica.SIunits.MagneticPotentialDifference im) 
      "Complex magnetic potential difference";
      record ComplexReluctance = Complex (
        redeclare Modelica.SIunits.Reluctance re,
        redeclare Modelica.SIunits.Reluctance im) "Complex reluctance";
    end Math;
  end MoveToModelica;
end Modelica_QuasiStationary;
