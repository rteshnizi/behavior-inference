# Behavior Inference in Dynamic Surveillance Settings

[![License](https://img.shields.io/badge/license-CC%20BY--NC-blue.svg)](LICENSE)
<!-- ![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg) -->

## Table of Contents
- [Behavior Inference in Dynamic Surveillance Settings](#behavior-inference-in-dynamic-surveillance-settings)
	- [Table of Contents](#table-of-contents)
	- [Introduction](#introduction)
	- [Demonstration](#demonstration)
	- [Project Structure](#project-structure)
	- [Installation](#installation)
	- [Usage](#usage)
	- [Citation](#citation)
	- [License](#license)

## Introduction
This repository contains the code, data, and supplementary materials for my PhD dissertation titled
**"A Practical Method of Behavior Estimation in Dynamic Robotic Surveillance Settings"**.
The slide deck that I prepared for my defense session is available [here](/docs/derfense.pdf).

This work studies how a robotic system can recognize patterns in movements
of a target based on formal specifications given by an end-user.
Specifically, we treat cases in which a network of autonomous robots
equipped with sensors are deployed in a dynamic environment and collect information
for an end-user to determine whether an unpredictable target exhibits certain---normal or anomalous---behavior.
We give an algorithmic solution to processing information streamed over extended periods of time about an environment with
dynamic elements and partial observability.
In realizing a practical solution, this dissertation contributes
by addressing this problem on three fronts by answering the questions of:
how to write formal specifications for motion-based behavior,
what is involved in recognizing behavior using these specifications,
and how to design a modular perception process to integrate models for behavior recognition.

## Demonstration
A video recording of the case study presented in the dissertation is linked below.

[![Video Demonstration](https://img.youtube.com/vi/ccrulp8tqR4/0.jpg)](https://www.youtube.com/watch?v=ccrulp8tqR4)

## Project Structure
[WIP]

<!-- - `data`: Contains raw and processed datasets.
- `notebooks`: Jupyter notebooks for data exploration and analysis.
- `src`: Source code for data processing, model training, and utilities.
  - `data_processing`: Scripts for processing raw data.
  - `models`: Implementation of machine learning models.
  - `utils`: Utility functions.
- `results`: Contains results from experiments.
- `experiments`: Configuration files and scripts to run experiments. -->

## Installation
[WIP]
<!-- To set up the project, follow these steps:

1. Clone the repository:
	```bash
	git clone https://github.com/yourusername/your-repo-name.git
	cd your-repo-name
	```

2. Create a virtual environment:
	```bash
	python -m venv env
	source env/bin/activate  # On Windows use `env\Scripts\activate`
	```

3. Install the required packages:
	```bash
	pip install -r requirements.txt
	``` -->

## Usage
[WIP]
<!-- To run the experiments, you can use the following commands:

1. **Preprocess Data:**
	```bash
	python src/data_processing/preprocess_data.py
	```

2. **Train Model:**
	```bash
	python src/models/train_model.py --config experiments/config.yaml
	```

3. **Evaluate Model:**
	```bash
	python src/models/evaluate_model.py --config experiments/config.yaml
	``` -->

<!-- Experiment configurations are stored in the `experiments` directory. Each configuration file contains the parameters for a specific experiment. To run an experiment, use:

```bash
python src/run_experiment.py --config experiments/experiment1.yaml -->

## Citation
If you use any part of this work in your research, please cite the dissertation using the following BibTeX entry:

```bibtex
@phdthesis{Teshnizi2024,
  author       = {Reza H. Teshnizi},
  title        = {A Practical Method of Behavior Estimation in Dynamic Robotic Surveillance Settings},
  school       = {Texas A\&M University},
  year         = {2024},
  address      = {College Station, TX, USA},
  type         = {PhD Dissertation},
  month        = {August},
}
```
You can also cite it as:

> Reza H. Teshnizi, "A Practical Method of Behavior Estimation in Dynamic Robotic Surveillance Settings," PhD dissertation, Texas A&M University, 2024.

## License

This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) License - see the [LICENSE](LICENSE) file for details.
