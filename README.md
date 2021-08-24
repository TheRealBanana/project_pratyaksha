# Project Pratyaksha
 Extra sensory perception via tactile stimulation


I was inspired by biohackers who have changed the sensory inputs their brains were receiving, showing that the brain can adapt quite well. Bicycles that turn the opposite way of the handlebars, glasses that enable you to see backwards, magnets under the finger to detect EM fields, and more. I wondered what it would be like to add some extra sense. How quickly would the brain learn to use the new information or would it just ignore the data as noise? 

The first idea I had was a simple navigation aid. A belt with 6 vibrating motors around the circumference that always vibrates in the direction of north, adding magnetoreception to our senses. There are many practical applications from helping seeing impaired individuals to a navigation aid for hikers. There's also no constraint as to the source of data being input. We could attach a software defined radio and instead buzz in the direction of certain radio signals. You could have a drone with object recognition inputting data as to the relative location of other people around you. Such a system would give an individual a real-life "spidy-sense".

It will be an interesting experiement. As of the creation of the repo its been under development for a little while (approx around the start of SensorMonitor repo) and I decided to track the progress on github.

--

May 2021 update

Here's what the first complete prototype system looks like. The belt is made from velcro material with the "buckles" self-designed and printed. Each "buckle" has a small 10mm diameter DC cellphone vibration motor glued into a matching recess. The leads from all 6 motors and a ground wire are routed out each end of the belt in 3-wire and 4-wire bundles. Each wire bundle is terminated at its end with a 6-pin header block. The main brainbox contains a custom subboard and a small atmel atmega2560 based prototyping board. The belt plugs into the subboard while the atmel controls everything. The magnetic heading data is taken from the SEN0140 10 degree-of-freedom sensor package. The magnetic heading data is corrected for tilt using the on-board IMU. The placement of the SEN0140 sensor is still undecided so its on a long lead. 

![pratyaksha_belt-fullsystem_prototype_1](https://user-images.githubusercontent.com/10580033/117729590-e97afd00-b19f-11eb-87e9-2b2f864d2289.jpeg)

Aug 2021 update
Turns out not knowing anything about designing circuit boards came back to bite me. My current design has overcurrent issues burning traces out. I needed to add a current limiting resistor to the motor outputs (not sure why I didnt) and maybe thicker traces. Taking a break from the project for a while to concentrate on a laser projector I'm trying to design.
