# Instructions

The **case 2** simulation is implemented in Python. Running the files `create_data_figure_9.py` and `create_data_figure_10.py` will generate data to create the plots. These scripts use `definition.py`, describing the planning problem, and `runners.py`, defining utility functions to run the simulation.

First install the dependencies in a Python 3.7 environment.
```bash
pip install cython
pip install numpy acrolib acrobotics
```

More info on **acrolib** [here](https://github.com/JeroenDM/acrolib) and on **acrobotics** [here](https://github.com/JeroenDM/acrobotics) (only supported on Linux because of the [python-fcl](https://pypi.org/project/python-fcl/) dependency, sorry :s ).
