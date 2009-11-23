within ;
package PowerFlow "Library for electrical power flow calculations"


  annotation (version="0.3", uses(Modelica(version="3.0")));


  replaceable package PackagePhaseSystem = 
      PowerFlow.PhaseSystems.ThreePhaseSymmetric "Default phase system" 
    annotation (choicesAllMatching=true);








end PowerFlow;
