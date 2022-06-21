# Altitude/Airspeed Autopilot System
The objective of this project is to develop an Autopilot system that can maintain a reference altitude and airspeed, similar to the conditions experienced during cruise flight. The Autopilot is designed to hold a reference altitude and airspeed, simulating cruise conditions for Piper PA30. It is a critical component for maintaining flight stability and fuel efficiency. The script models the aircraft's dynamics and implements a control system to achieve this goal.

The script utilizes state-space representation, LQR controller design and fundamental control theory principles to design and evaluate Autopilot's performance. The code encompasses various aspects, including  system modeling, stability analysis, controller design, observer design, and considerations for wind effects. The goal is to provide a comprehensive solution for controlling and maintaining the desired altitude and airspeed of an aircraft.

## Contents

- [System Modeling](#system-modeling)
- [Stability Analysis](#stability-analysis)
- [Dynamic Accuracy Analysis](#dynamic-accuracy-analysis)
- [Controllability and Observability Analysis](#controllability-and-observability-analysis)
- [Controller Design (LQR)](#controller-design-lqr)
- [Flying Qualities Analysis](#flying-qualities-analysis)
- [Altitude/Airspeed Hold Autopilot Design ](#Altitude_and_Airspeed_Hold_Autopilot)
- [Respose improving at worst condition](#Respose-improving-at-worst-condition)
- [Observer Design](#observer-design)
- [Overall system Analysis](#Overallsystem-analysis)
- [Wind Model](#wind-model)

## System Modeling

The system modeling section defines the state-space matrices `A`, `B`, `C`, and `D`, which represent the dynamics of the Altitude/Airspeed Autopilot system. These matrices describe how the system's state variables, including true airspeed, pitch angle, pitch rate, pitch attitude, altitude, and engine RPM, evolve over time in response to control inputs. This detailed modeling is essential for accurate control system design.
 **Control Matrices**: Defines matrices (A, B, C, and D) representing the linearized control system. These matrices are used for control analysis and design.
 The state vector includes variables such as true airspeed (TAS), attitude angle (alpha), pitch rate (q), pitch angle (teta), altitude (h), and engine speed (n).
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/LinearizedlongitudinalModel_for_Piper_PA30_Aircraft.PNG" width="80%"></p>

## Stability Analysis

Stability analysis is a crucial step in assessing the behavior of the autopilot system. The code checks whether the system is asymptotically stable by analyzing the eigenvalues of matrix `A` and visualizes pole locations and damping ratios. The results provide insights into the system's inherent stability and whether it requires control intervention. It also uses the damp and pzmap functions to visualize pole locations and damping ratios also provides information about the poles of the transfer function and the system's stability.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/StatbilityofopenloopsystemwithoutSAS.PNG" width="60%"></p>

## Dynamic Accuracy Analysis

This section calculates dynamic performance parameters for different aircraft modes, including the short period, phugoid, engine, and energy modes. Parameters such as natural frequency, damping coefficient, maximum overshoot, and settling time are computed. These metrics help evaluate how well the autopilot can maintain the desired altitude and airspeed while ensuring passenger comfort. Also a table of these parameters is used to assess the dynamic accuracy of the open-loop system.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/DynamicAccuracy_Analysis.PNG" width="80%"></p>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/ModelAnalysis.PNG" width="80%"></p>

## Controllability and Observability Analysis

Controllability and observability are critical aspects of control system design. The code assesses whether the system is controllable and observable using matrices `P` and `O`. Full controllability and observability are essential for effective control system implementation. It verifies if the system is completely controllable and observable. 
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/contobs.PNG" width="60%"></p>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/ExitngModeAnalysis.PNG" width="80%"></p>

## Controller Design (LQR)

The autopilot controller is designed using the Linear Quadratic Regulator (LQR) method, a powerful technique for optimizing control performance. to LQR computes the optimal feedback gain (K) that minimizes a performance index. The code iteratively adjusts weighting matrices `Q` and `R` to fine-tune the controller's behavior. The LQR controller aims to minimize a performance index, ensuring precise control of altitude and airspeed. The code iteratively adjusts the weighting matrices Q and R to optimize controller performance and minimize a performance index.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/LQR.PNG" width="80%"></p>
  <p align="center"">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/LQR1.PNG" alt="condition to have asymptotical solution" width="80%">
    <p align="center">Condition to have asymptotical solution</p>
  </div>
  <p align="center">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/LQR2.PNG" alt="Steps To Find Optimal Gain" width="80%">
    <p align="center">Steps To Find Optimal Gain</p>
  </div>
</div>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/Eigenval.PNG" width="80%"></p>

## Flying Qualities Analysis

Flying qualities are a key consideration for aircraft safety and comfort. This section assesses parameters such as natural frequencies, damping ratios, maximum overshoot, and settling times for short period and phugoid modes. A comparison is made between the original and feedback systems to evaluate the autopilot's impact on flying qualities.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/FlyingQualitiesComparison.PNG" width="80%"></p>

## Altitude/Airspeed Hold Autopilot Design 
Calculates the feedforward gain for the altitude and airspeed hold autopilot. It determines the desired control input to maintain the desired attitude and airspeed.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Machine-Learning-Projects1/blob/Master/Altitude_AirspeedAutopilot/Figures/Altitude_AirSpeed_hold.PNG" width="90%"></p>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/SSCP.PNG" width="80%"></p>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/PiperPA30.PNG" width="90%"></p>


## Respose improving at worst condition
We can analyze the response without forcing the maneuver and starting from the worst initial condition possible. First in open loop without feedback and then in closed loop with feedback, Such evidence shows the
improvement of the flying qualities due to the SAS.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/Worstcond.PNG" width="80%"></p>

## Observer Design

An observer is a critical component for estimating unmeasured states of the system. Observer poles are placed strategically to ensure that the estimation error vanishes asymptotically. This observer enhances the system's ability to maintain altitude and airspeed by providing accurate state estimates.
<div style="display: flex; flex-direction: row;">
  <p align="center">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/Observerde.PNG" alt="" width="60%">
    <p align="center"></p>
  </div>
  <p align="center">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/Observerde2.PNG" alt="" width="60%">
    <p align="center"></p>
  </div>
</div>
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/ObserverPoles.PNG" width="80%"></p>

## Overall System Analysis
Combines the observer and feedback control system to create the overall closed-loop system. It analyzes the eigenvalues of the combined system to verify asymptotic stability.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/Overallsyst.PNG" width="80%"></p>

## Wind Model Integration
Accounts for wind effects by extending the system dynamics to include wind disturbances on the autopilot system. Parameters related to wind modeling, including wind covariance and correlation time constants and incorporates them into the model. Understanding and mitigating the effects of wind are essential for robust autopilot performance.
<p align="center"><img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/WindModel.PNG" width="90%"></p>

## Results Plots

<div style="display: flex; flex-direction: row;">
  <p align="center">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/VelocityComparison.PNG" alt="VelocityComparison" width="60%">
    <p align="center">Velocity Comparison</p>
  </div>
  <p align="center">
    <img src="https://github.com/Rohit-Gupta2/Autonomy-and-Autopilots/blob/Master/Altitude%20Airspeed%20Autopilot%20System/Figures/AltitudesComparison.PNG" alt="Altitudes Comparison" width="60%">
    <p align="center">Altitudes Comparison</p>
  </div>
</div>

