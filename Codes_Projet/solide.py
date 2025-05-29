class SolideRigide():
    from vector3D import Vector3D as V3D
    from torseur import Torseur as T
    
    
    def __init__(self,CentreDeMasse=V3D(),PosLiaison=V3D(),masse=0):
        
        self.G = CentreDeMasse
        self.L0 = PosLiaison
        self.masse = masse
        self.EffortsExt = [T(self.G,self.masse*g)] #poids, g doit être défini avant
        self.EffortLiaison = T(P=self.L0)
        self.enfants = []
        
    def __str__(self):
        return('SolideRigide(%s,%s,%g)' % (self.G,self.L0,self.masse))

    def __repr__(self):
        return(str(self))
    
    def appEffort(self,T_ext=T()):
        self.EffortsExt.append(T_ext)
    
    def enfant(self,Sol_enf):
        self.enfants.append(Sol_enf)

    def equilibre(self):

        self.EffortLiaison = T(self.L0,V3D(),V3D())
        EffortsEnfants = []

        for s in self.enfants:            
            s.equilibre()
            EffortsEnfants.append(s.EffortLiaison)

        for t in self.EffortsExt:
            self.EffortLiaison = self.EffortLiaison + t
        
        for t in EffortsEnfants:
            self.EffortLiaison = self.EffortLiaison + t
        
        
    
    
if __name__ == "__main__": # false lors d'un import
    from vector3D import Vector3D as V3D
    from torseur import Torseur as T

    g = V3D(0,-9.8,0)
    S1 = SolideRigide(masse=10)
    print(S1)
    
    S2 = SolideRigide(V3D(1,0,0),masse=5)
    S1.enfant(S2)
    S1.equilibre()
    print('Avec S1+S2 ',S1.EffortLiaison)
    S3 = SolideRigide(V3D(-1,0,0),V3D(-1,0,0),masse=5)
    S1.enfant(S3)
    S1.equilibre()
    print('Avec S1+S2+S3', S1.EffortLiaison)
    
