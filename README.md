CS396 LudoBots Module 7:

Grading Note:

This assignments goal was to "expand the design space of your random creature generator to 3D." While I've done my best to adhere to the spirit of that assignment, these are not necessarily going to be 3D generated snakes. Instead, these are going to be randomly generated tardigrades, which are insect like creatures that are meant to be one of multiple entities present in my final project. I have done my best to preserve the "randomness" of their body structure for the sake of this assignment, but the goal of the work this week was to explore some of its key behaviors for the project. If any of the spirit of the assignment is not being met, let me know and I'd be happy to update.

1. Summary

  a. Body

    A tardigrade is a randomly generated insect-like creature that will be evolving attraction/avoidance behavior from a predator in my final project. Their  body consists of head, which contains a sensor, a randomly generated number of body links, and the random presence of legs or no legs on that body link. The legs of a respective body link always generate in perpindicular pairs, and consist of two segments connected at a fixed right angle.

  b. Brain Movement
  
    A tardigrade's brain consists of a sensor neuron in its head, and sensor neurons in the outer segment of each of its legs (its "foot"). Unlike previous simulations, the tardigrade's legs move in constant sinusoidal motion. A sinusoidal wave with amplitude equal to the maximum joint angle is generated and divided into walking steps. At each step-interval, the angle of a joint is designated as the value of a given step on the sinusoidal wave. Here is a plot showing the angle 
    
  ![image](https://user-images.githubusercontent.com/31931152/220221906-39a90635-4ab4-4dff-95c4-224cdaeaf2f5.png)

  

3. Movement


