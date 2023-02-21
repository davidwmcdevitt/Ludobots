CS396 LudoBots Module 7:

Grading Note:

This assignments goal was to "expand the design space of your random creature generator to 3D." While I've done my best to adhere to the spirit of that assignment, these are not necessarily going to be 3D generated snakes. Instead, these are going to be randomly generated tardigrades, which are insect like creatures that are meant to be one of multiple entities present in my final project. I have done my best to preserve the "randomness" of their body structure for the sake of this assignment, but the goal of the work this week was to explore some of its key behaviors for the project. If any of the spirit of the assignment is not being met, let me know and I'd be happy to update.

1. Summary

      a. Body

      A tardigrade is a randomly generated insect-like creature that will be evolving attraction/avoidance behavior from a predator in my final project. Their  body consists of head, which contains a sensor, a randomly generated number of body links, and the random presence of legs or no legs on that body link. The legs of a respective body link always generate in perpindicular pairs, and consist of two segments connected at a fixed right angle.
      
      Examples of tardigrade bodies:
      

      b. Brain and Movement

    A tardigrade's brain consists of a sensor neuron in its head, and sensor neurons in the outer segment of each of its legs (its "foot"). Unlike previous simulations, the tardigrade's legs move in constant sinusoidal motion. A sinusoidal wave with amplitude equal to the maximum joint angle is generated and divided into walking steps. At each step-interval, the angle of a joint is designated as the value of a given step on the sinusoidal wave. Here is a plot showing the angle.

     ![image](https://user-images.githubusercontent.com/31931152/220223503-ea92d8e3-26c7-43d8-a149-1f039dd33f15.png)

    At the next step-interval in the simulations sequence, the angle of the joint is designated to be X number of steps away from the original point. The plot below has a second point that is 5 steps away from the original point.

     ![image](https://user-images.githubusercontent.com/31931152/220222546-509ba4bb-ee03-4ceb-8443-39847740339a.png)

    The purpose for this design is to ensure that tardigrade's legs are constantly moving in a walking motion. Input from the tardigrade's brain is incorporated into the tardigrade's movement by the pace at which the angle moves along (in a positive or negative direction) the x-axis of the sinusoidal wave. If a neuron is delivering a small positive value, the legs will move along the sinusoidal wave at a slow pace. If a neuron is delivering a large positive value, the legs will move along sinusoidal wave at a faster pace. If a neuron is delivering a negative value, the legs will move backwards along the sinusoidal wave at a pace corresponding. In combination, this allows each leg of the tardigrade to move in a forward or backwards motion in a manner that corresponds to forwards or backwards walking behavior. The purpose for this design is to allow for external sensors to influence walking behavior without undermining the core ability of a tardigrade to walk. 
    
    Video of evolved tardigrade walking: https://youtu.be/TNA-IDKolBk
    
    c. Death
    
    A large part of the exploration in this stage had to do with the the manner at which the tardigrade is able to interact with the environment around it. This allowed me to build in the feature for a tardigrade to deactivate - or - to use a more lifelike nomenclature - a tardigrade can die. Death is triggered by a new function that checks the sensor value of the head at a given step-interval. If the head is making contact with anything other than its own body, it triggers the mechanism that "kills" the robot ("killing" the robot is completed by freezing all motors, leaving it inert until the end of the simulation). At the moment, this encourages the act of upright walking, rather than flailing around wildly to generate movement away from a spawn point. For the final project, this "kill" switch will be used to trigger death by proximity to some other object, or , a "predator."
    
    Video of a tardigrade's "death" mechanism being triggered: https://youtu.be/RoqIM0av1aI 


