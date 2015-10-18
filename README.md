# CS351_VirtualCreature_API
All classes in vcreature.phenotype package are part of the API.
All other classes (those in vcreature.mainSimulation) are examples of how to use the API.

The example main is in vcreature.mainSimulation.MainSim.java.
The default creature is FlappyBird using the old, now deprecated, API.
While running, this example, the following are keyboard commands:

    q: quit
    p: toggle camera rotation (default is on)
    c: The first time this is pressed, the default FlappyBird creature is
       removed form the sim and a new FlappyBird2 creature is created.
       FlappyBird2 is the same as FappyBird only all blocks are rotated 30
       degrees.
       The second time this is pressed, the FlappyBird2 creature has one of
       its legs deleted and then a new leg is added.
