within PowerFlow;
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
