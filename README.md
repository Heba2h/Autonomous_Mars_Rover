## Mars Rover Project: NASA Search and Sample Return Challenge!

This search and sample return project was based after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html), and it provides experience with the three essential elements of robotics, which are perception, decision making and actuation. This specific project uses the Unity game engine to simulate the environment.

## Dependencies

The included code requires Python 3 and many dependencies. Using Anaconda is the easiest way to get things working. clone the repositry and follow these steps

- `conda create --name <ENVIOREMENT NAME> --file requirements.txt`
- `conda activate <ENVIOREMENT NAME>`

Or

- `conda create -n <ENVIOREMENT NAME>`
- `conda activate <ENVIOREMENT NAME>`
- `conda install --file requirements.txt`

## Run the Code

You can test out the simulator by opening it up and choosing "Training Mode."

To run the automated code included in this repository:

- Activate the conda environment with `conda activate <ENVIOREMENT NAME>` (setup by following the instructions [here](https://github.com/ryan-keenan/RoboND-Python-Starterkit/blob/master/doc/configure_via_anaconda.md))
- Run `python ./code/drive_rover.py` to start the automation logic (this communicates with the simulator directly)
- Start the simulator (double click `Roversim.x86_64` or `Roversim.x86`) and choose "Autonomous Mode."

### To Run ROS

- `roscore`
- `rosrun gmapping slam_gmapping scan:=base_scan _particles:=30 _temporalUpdate:=0.01 _map_update_inerval:=1.0 _resampleThreshold:=4`
- `rviz`

Press Ctrl+O to and open GMapping_Config.rvoz

- `conda activate <ENVIOREMENT NAME>`
- `python ./code/drive_rover.py`

## Notebook Analysis

This [Jupyter Notebook](https://github.com/Heba2h/Autonomous_Mars_Rover/blob/main/phase%202/Rover_Project_Test_Notebook.ipynb) includes all of the major functions, which are broken out into individual sections as follows:
