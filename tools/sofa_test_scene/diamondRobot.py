# -*- coding: utf-8 -*-

import os
import sys

#   STLIB IMPORT
from stlib.scene import MainHeader
from stlib.solver import DefaultSolver
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

# SOFTROBOTS IMPORT
from softrobots.actuators import PullingCable

actuatorsParam = [
        {'withName' : 'cable1',
         'withCableGeometry' : [[0, 5, -300]],
         'withAPullPointLocation' : [0, 200, -300]
        },
        {'withName' : 'cable2',
         'withCableGeometry' : [[0, 5, -300]],
         'withAPullPointLocation' : [0, 200, -100]
        },
        {'withName' :'cable3',
         'withCableGeometry' : [[0, 5, -300]],
         'withAPullPointLocation' : [0, 200, -400]
        }
    ]

meshPath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):

    rootNode.createObject('VisualStyle', displayFlags='showVisualModels showForceFields')

    rootNode.findData('gravity').value=[0.0,0.0,-9810];
    rootNode.findData('dt').value=1

    plugins=["SofaPython","SoftRobots","ModelOrderReduction"]
    for name in plugins:
        rootNode.createObject('RequiredPlugin', name=name, printLog=False)
        
    rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")


    modelNode = ElasticMaterialObject(
        attachedTo=rootNode,
        volumeMeshFileName=meshPath+'cylinder_real_sparse.vtu',
        # volumeMeshFileName=meshPath+'siliconeV0.vtu',
        name='modelNode',
        # rotation=[90, 0.0, 0.0],
        # translation=[0.0, 0.0, 35],
        rotation=[0.0, 0.0, 0.0],
        translation=[0.0, 0.0, 0],
        totalMass=0.5,
        withConstrain=False,
        surfaceMeshFileName=meshPath+'cylinder.STL',
        surfaceColor=[0.7, 0.7, 0.7, 0.7],
        poissonRatio=0.45,
        youngModulus=450)
    
    modelNode.createObject('GenericConstraintCorrection', solverName='solver')

    FixedBox(
            atPositions=[-15, -5, -20,  15, 25, 10],
            applyTo=modelNode,
            name="box1",   
            doVisualization=True)       
    FixedBox(
            atPositions=[-15, -5, -615, 15, 25, -585],
            applyTo=modelNode,
            name="box2",
            doVisualization=True)

    for i in range(len(actuatorsParam)):
        cable = PullingCable(
                    attachedTo=modelNode,
                    name=actuatorsParam[i]['withName'],
                    cableGeometry=actuatorsParam[i]['withCableGeometry'],
                    pullPointLocation=actuatorsParam[i]['withAPullPointLocation'],
                    valueType="displacement")

    return rootNode
