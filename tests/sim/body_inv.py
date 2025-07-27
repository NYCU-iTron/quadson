import pybullet as p
import pybullet_data
import time
from sim.quadson import Quadson
from sim.interface import Interface

def main():
  p.connect(p.GUI) # (GUI for visualization, DIRECT for headless)
  p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
  p.resetSimulation()
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.setGravity(0, 0, -9.81)
  p.setTimeStep(1/240)
  p.loadURDF("plane.urdf")
  
  interface = Interface(type='gui', target='orientation')
  quadson = Quadson(interface)
  while True:
    quadson.update()
    p.stepSimulation()
    time.sleep(1 / 240)

if __name__ == "__main__":
  main()
