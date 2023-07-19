# Hanging Drone: An Approach to UAV Landing for Monitoring
Official implementation of **[Hanging Drone: An Approach to UAV Landing for Monitoring]()**

## Description

This work's approach allows a Tello Drone to land in strategic locations for data acquisition, resulting in significantly less battery consumption. The method uses principles from stereo vision through a monocular camera motion to estimate the relative position of a selected landing site, allowing a drone to hang itself by a hook in an artificial (e.g., aluminum frame, power line) or natural (e.g., tree branch) location. However, the system is limited to static landing sites where the FAST feature detector algorithm can detect features.

**See the demonstration video:**

<!---->
[<img src="https://img.youtube.com/vi/-aCFcoKEJI8/maxresdefault.jpg" width="50%">](https://youtu.be/-aCFcoKEJI8)

## Drone Utilized

<img src="drone/drone_image.png" width="50%">

## Citation

```
@InProceedings{Cechinel:2023,
    author    = {Cechinel, Alan K. and Röning, Juha and Tikanmaki, Antti and De Pieri, Edson R. and Plentz, Patricia D. M.},
    title     = {{Hanging Drone}: An Approach to UAV Landing for Monitoring},
    booktitle = {Proceedings of the 20th International Conference on Informatics in Control, Automation and Robotics - ICINCO},
    month     = {},
    year      = {2023},
    pages     = {}
}
```

## Requirements

Install the [TelloPy](https://github.com/hanyazou/TelloPy) library and replace the original tello.py file with the [tello.py](extra/tello.py) (provided in this repository) modified to work with multiple drones. keep in mind that each drone must be connected to a dedicated WiFi interface.


```bash
$ git clone https://github.com/alankc/landing_pipeline
$ cd landing_pipeline
$ pip install -r requirements.txt
```

## Usage

### Running the system

### Control Modes

#### Waiting Connection

#### Manual Control

#### Manual Land

#### Autonomous Landing

#### Go To
