All of the data and the IPython Notebook we used to generate figures for for our paper, "Critical interplay between density-dependent predation and the evolution of the selfish herd" are contained in the included `selfish-herd` directory.

------------------------------------------------------------------------------------------

If you have questions, contact us at:

Randal S. Olson (rso@randalolson.com)

David B. Knoester (dk@msu.edu)

------------------------------------------------------------------------------------------

RUNNING IPYTHON NOTEBOOK

To run our IPython Notebook, you must first download the required packages:

* Python
* IPython
* IPython Notebook
* glob
* NumPy
* SciPy
* pandas

We recommend downloading the Enthought Python distribution, which has all of the packages you will need. There is a free version for people with an active .edu email here:

http://enthought.com/products/edudownload.php

or a smaller, free version for those without an active .edu email here:

http://enthought.com/products/epd_free.php

Once you have installed the required packages, open your terminal and navigate to the eos-predator-confusion directory. Once there, enter the command:

ipython notebook --pylab=inline

This will open up the IPython Notebook terminal. Select the notebook labeled "Predator Confusion Final Graphs." This notebook is entirely interactive, so feel free to explore the data yourself.

------------------------------------------------------------------------------------------

DIRECTORY LABELS

Each experiment has its own directory. `kd` and the number following it represents the predator handling time, while `np` and the number following it represents the number of predators in the experiment. The directory names are as follows:

* swarm-sd30-selfish-herd-artificial-selection-outside-attack-kd4: Artificial selection experiment with 250 prey and outside attacks.

* swarm-sd30-selfish-herd-artificial-selection-outside-attack-probdeath-kd4: Artificial selection experiment with 250 prey, outside attacks, and density-dependent predation.

* swarm-sd30-selfish-herd-artificial-selection-rand-attack-kd4: Artificial selection experiment with 250 prey and uniformly random attacks.

* swarm-sd30-selfish-herd-artificial-selection-rand-attack-probdeath-kd4: Artificial selection experiment with 250 prey, uniformly random attacks, and density-dependent predation.

* swarm-sd30-selfish-herd-artificial-selection-randwalk-attack-kd4: Artificial selection experiment with 250 prey and random walk attacks.

* swarm-sd30-selfish-herd-artificial-selection-randwalk-attack-probdeath-kd4: Artificial selection experiment with 250 prey, random walk attacks, and density-dependent predation.

* swarm-sd30-selfish-herd-rand-starting-pos-density-dependent-survival-swarmsize100-np4-kd10: Predator-prey coevolution experiment with 100 prey and density-dependent predation.

* swarm-sd30-selfish-herd-rand-starting-pos-swarmsize100-np4-kd10: Predator-prey coevolution experiment with 100 prey.

------------------------------------------------------------------------------------------

CSV FILE FORMAT

Each csv file is one replicate for that experiment. The number following the word "run" in the csv file name is the random number generator seed for that replicate. e.g., run24LOD.csv has a random number generator seed of 24.

In the LOD files, there will be a single entry for each ancestor in the final best swarm agent's LOD. LOD files will be in csv format with the column headers listed at the top. Column headers are in the following order:

* generation: the generation the ancestor was born

* prey_fitness: the fitness of the ancestor prey

* predator_fitness: the fitness of the ancestor predator

* num_alive_end: the number of surviving prey at the end of the fitness evaluation

* avg_bb_size: the average bounding box size of the swarm during the simulation

* var_bb_size: the variance in the bounding box size of the swarm during the simulation

* avg_shortest_dist: the average distance from every prey agent to the prey agent closest to it

* swarm_density_count: the average number of prey agents within safety distance units of each other

* prey_neurons_connected_prey_retina: the number of conspecific sensor neurons that the prey Markov network brain is connected to

* prey_neurons_connected_predator_retina: the number of predator sensor neurons that the prey Markov network brain is connected to

* predator_neurons_connected_prey_retina: the number of prey sensor neurons that the predator Markov network brain is connected to

* num_attacks: the number of attacks the predator made on prey during the simulation

------------------------------------------------------------------------------------------

OTHER FILES

Some directories, such as swarm-sd30-selfish-herd-rand-starting-pos-swarmsize100-np4-kd10, also have .genome and .dot files.

Generally, we save Markov network brain files as .genome files. These files contain integer values which encode the Markov network brain.

DOT files are the picture representations of Markov network brain structure and connectivity. We recommend using the Graphviz software to view these images.

------------------------------------------------------------------------------------------