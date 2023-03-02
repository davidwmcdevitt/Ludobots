CS396 LudoBots Final Project:

Concept Introduction:

For the final Ludobots project, I wanted to conduct an exploration into the world of antagonistic evolution. Up until this point, all tasks that we have optimized for over the course of simulations have been independent linear tasks, such as walking motions based on a defined phenotype or randomly generated bodies capable of moving in a certain manner. While this does represent what likely occurred during the earliest stages of organic evolution (i.e., the evolutionary path between single celled organisms, not all outcomes and convergences on features in organic evolution were a result of independent linear tasks. Many of the features that most prominently define the phenotypes of organisms that we see around us are the result of antagonistic evolution - they are the product of competition between an organism and its environmental hazards. 

This project will feature an evolutionary combat tournament between pairs of generated organisms. Rather than having limbs for locomotion, these organisms will be rooted to a platform with a single joint in the middle. This simplifies the evolutionary process and focuses it onto an organisms ability to interact and another organism. A base spinal cube will be generated, rooted by a single prismatic (joystick) joint in the middle of the platform. This will give the organism the ability to move freely on in the X and Y direction of its platform. On top of the base spinal cube will be a generated body, whose generation process will be detailed later. On top of the body will be a head cube, which will be the key to simulating combat. Inside the head cube will be a sensor neuron, which will "kill" the organism if contact between the head and another object is ever detected. A diagram of a rudimentary organism can be seen below:

      <img width="357" alt="image" src="https://github.com/davidwmcdevitt/Ludobots/blob/52e87c134e5686fbf1094ca09ed89c90c715aeb7/challenger_diagram.jpeg">
      

In every given simulation, two organisms will be present on adjacent platforms featuring different body generations. As illustrated below (same color key applies below):

[Image link of matchup diagram]

The evolved behavior will be a combat simulation between the two organisms in a given simulation. At the beginning of a simulation, both organisms will begin moving until one organism "kills" another by making contact with its head (or a predetermined time cap is reached). 

Selection:

Selection will be determined by a round robin tournament of all pairwise combinations of organisms in a population. Points will be awarded based on the outcome of each pairwise matchup. The point system will be structured in a manner that awards a victorious simulation over surviving until the time cap, but will award survival to the end over defeat (similar to the win, loss, tie point system utilized in typical round robin tournaments like the World Cup). Once all pairwise matchups have been completed, the top organisms will move on to the next generation, which will include mutations and newly instantiated organisms. 

Pyrosim Considerations HW8 Exploration:

The HW8 exploration will be dedicated to developing a parallel combat system in Pyrosim. The primary challenge to overcome would be the fact that having two independent organisms inside a single simulation is not intuitive in the current Pyrosim structure. However, it would be possible to simulate this scenario with a single bilateral organism, as long as two statements remain true throughout the simulation:
1. Two "fitness" values will be recorded in a simulation, one for each organism's evaluation
2. The brain from one organism involves only interactions between an organisms own sensor and motor neurons
	- The current brain state of Pyrosim is a [Number of Sensors] x [Number of Motors] matrix, so that every sensor and motor combination has a weight to their interaction. A potential solution would be to reshape the brain to resemble two matrices appended to one another, and to develop a new system of "thinking" to ensure the proper lookup indices are being made. 

