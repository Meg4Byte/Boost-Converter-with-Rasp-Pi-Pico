### Prototip verzija na engleskom...
---

### Introduction

In modern electronics, efficient power conversion is a critical requirement in many applications. One such method of achieving efficient voltage conversion is through a boost converter, which steps up a lower input voltage to a higher output voltage. The focus of this project is the implementation of a boost power supply using a Proportional-Integral-Derivative (PID) controller. The PID controller offers an effective means of regulating the output voltage, ensuring system stability and desired performance.

The Raspberry Pi Pico, a versatile and affordable microcontroller, is utilized in this project to manage the control algorithms and monitor the system in real-time. The choice of the Raspberry Pi Pico is based on its low cost, high performance, and ease of integration with custom electronics. By combining these elements, the project aims to deliver an efficient and reliable boost converter design suitable for a variety of applications, such as renewable energy systems and portable electronics.

---

### Scope of the Project

This project will focus on the following key areas:

1. **Design and Simulation of the Boost Converter**  
   The theoretical design of the boost converter will be developed, followed by simulations to verify its performance under various operating conditions.

2. **Implementation of the PID Controller**  
   A PID controller will be designed and implemented on the Raspberry Pi Pico to regulate the output voltage of the boost converter. The controller will be tuned to achieve optimal stability and response.

3. **Hardware Development**  
   The physical circuit for the boost converter will be constructed, and the necessary sensors and feedback loops will be integrated to enable real-time monitoring and control.

4. **Testing and Optimization**  
   The system will be tested under various load conditions to evaluate its efficiency and stability. Adjustments to the PID controller parameters will be made to ensure optimal performance.

5. **Documentation and Analysis**  
   The results of the implementation will be documented, including an analysis of the efficiency, response times, and system stability.
    .
    .
    .
    TO DO ...
---

### Boost Converter Brief Overview

A boost converter is a type of DC-DC converter that increases (or "boosts") a lower input voltage to a higher output voltage. This is achieved through the use of energy storage elements such as inductors, capacitors, and switches, typically a transistor and a diode. The key principle behind its operation lies in energy storage and release: when the switch is closed, energy is stored in the inductor, and when the switch is opened, this energy is transferred to the load through the diode, raising the output voltage above the input.

The boost converter is widely used in applications where the input voltage is lower than the required output voltage, such as battery-powered systems, renewable energy systems like solar power, and power supply circuits for various electronic devices. Its advantages include a relatively simple design, high efficiency, and the ability to handle a wide range of input voltages. However, one of the main challenges is maintaining a stable output voltage, especially under varying load conditions.

To address this, control techniques like Proportional-Integral-Derivative (PID) control are often employed. The PID controller helps to regulate the output voltage by adjusting the duty cycle of the switch in real-time based on feedback from the output voltage. This ensures that the boost converter operates efficiently while maintaining the desired output voltage, even when the input voltage or load fluctuates.


### Detailed Operation of the Boost Converter

The boost converter operates on the principle of inductor energy storage and controlled switching. In its basic form, the circuit consists of an inductor, a switch (usually a MOSFET), a diode, and an output capacitor. The input voltage source provides energy to the system, which is then transferred to the load at a higher voltage through the following stages:

##### Switch ON (Charging Mode):
When the switch is closed, current flows from the input through the inductor. During this time, the inductor stores energy in its magnetic field. Since the diode is reverse-biased, no current flows to the load. The voltage across the inductor increases linearly, following VL=LdidtVL​=Ldtdi​, where VLVL​ is the voltage across the inductor, LL is the inductance, and di/dtdi/dt is the rate of change of current
##### Switch OFF (Discharging Mode):
When the switch opens, the inductor resists the sudden drop in current by releasing its stored energy. The polarity of the voltage across the inductor reverses, forward-biasing the diode, and the stored energy is delivered to both the output capacitor and the load. The output voltage, VoutVout​, is now higher than the input voltage, VinVin​, with the relationship given by:
Vout=Vin1−D
Vout​=1−DVin​
where DD is the duty cycle of the switching signal, representing the fraction of time the switch is ON in a complete cycle. The duty cycle plays a crucial role in determining the output voltage. As DD increases, the output voltage increases, provided that DD is less than 1 (typically between 0 and 0.8 in practical applications)
Steady-State Analysis:
In steady-state operation, the average voltage across the inductor must be zero over one complete switching period to prevent saturation. This implies that the energy stored during the ON period is equal to the energy transferred during the OFF period. The key performance metrics for a boost converter include efficiency, output voltage ripple, and transient response. These depend on the inductor value, switching frequency, and the quality of the components used
Component Selection:
The selection of the inductor, switch, and capacitor is critical to the converter’s performance. The inductor value is chosen based on the desired current ripple, calculated as:
ΔIL=VinDfsL
ΔIL​=fs​LVin​D
where ΔILΔIL​ is the peak-to-peak inductor current ripple, fsfs​ is the switching frequency, and LL is the inductance. Lower current ripple requires a larger inductor, but at the cost of slower transient response
The output capacitor smooths the output voltage by storing charge during the ON cycle and releasing it during the OFF cycle. The output voltage ripple is given by:
ΔVout=IoutDfsC
ΔVout​=fs​CIout​D
where ΔVout/ΔVout​ is the output voltage ripple, IoutIout​ is the load current, and CC is the capacitance. A larger capacitor reduces ripple but may increase the size and cost of the circuit
Efficiency Considerations:
The efficiency of the boost converter is defined as the ratio of output power to input power:
η=PoutPin=VoutIoutVinIin
η=Pin​Pout​​=Vin​Iin​Vout​Iout​
Losses occur primarily due to the resistance of the inductor (copper losses), the on-resistance of the MOSFET, and the forward voltage drop across the diode. To maximize efficiency, low-ESR components and fast-recovery diodes are used, while synchronous rectification can replace the diode with another MOSFET to further reduce losses.

### Odd Behavior of the Boost Converter: Understanding How it Increases Output Voltage

At first glance, it might seem counterintuitive that a boost converter can produce a higher output voltage than its input voltage. Doesn't this defy the law of conservation of energy? The key lies in understanding how energy is stored and transferred in the circuit.
Energy Storage and Transfer

A boost converter operates by temporarily storing energy in an inductor and then releasing it in a controlled manner to increase the output voltage. Here's how the process works:

During the "on" phase, the switch (typically a transistor) closes, and current flows through the inductor, storing energy in the magnetic field. In this phase, the output capacitor supplies the load, while the input voltage is applied directly across the inductor.

During the "off" phase, the switch opens, and the inductor releases its stored energy. The inductor generates a voltage that adds to the input voltage, effectively boosting the total voltage applied to the output. The energy stored in the magnetic field is transferred to both the load and the output capacitor, charging it to a higher voltage than the input.

This process doesn't violate energy conservation because the total energy delivered to the load is a result of converting the stored energy in the inductor into electrical energy. However, as per energy conservation:
Pin=Pout+losses
Pin​=Pout​+losses

Where:

PinPin​ is the power supplied from the input source.
PoutPout​ is the power delivered to the load.
"Losses" include inefficiencies from non-ideal elements such as resistances and switching losses.

So, while the voltage is higher at the output, the current at the output is lower, meaning the overall power remains conserved. The converter trades higher voltage for lower current, which preserves energy balance in the system.
Non-Idealities in Boost Converters: Their Impact on Performance

In real-world implementations, ideal components do not exist. The performance of the boost converter is affected by various non-idealities in its components, including inductors, capacitors, switches, and diodes. These non-ideal factors lead to inefficiencies, reduced performance, and sometimes unexpected behavior.

1. Inductor Non-Idealities

    Series Resistance (DCR): Real inductors have series resistance due to the winding wire, which causes power losses as heat. This reduces the efficiency of the boost converter, particularly at high currents. The energy stored in the inductor during the "on" phase is not fully recovered during the "off" phase.

    Core Losses: At higher switching frequencies, core losses (hysteresis and eddy current losses) become significant, reducing the energy transfer efficiency. These losses increase with higher switching frequencies and high current levels.

    Saturation: Inductors have a saturation limit, beyond which they lose their ability to store energy effectively. If the inductor saturates, its inductance decreases rapidly, leading to poor voltage regulation and increased ripple.

2. Capacitor Non-Idealities

    Equivalent Series Resistance (ESR): Capacitors have an ESR, which introduces resistive losses in the converter. This causes ripple in the output voltage and reduces the overall efficiency, especially under high current conditions. High ESR also limits the ability of the capacitor to smooth out voltage variations effectively.

    Leakage Current: In real capacitors, leakage current causes slow discharge over time, which can impact voltage regulation and reduce efficiency.

3. Switching Losses in MOSFETs (or Transistors)

    Conduction Losses: When the switch is conducting, it has an on-state resistance (RDS(on)DS(on)​), which causes voltage drops and energy dissipation in the form of heat. Higher RDS(on)RDS(on)​ leads to greater conduction losses and lower efficiency.

    Switching Losses: Each time the switch turns on or off, a brief period occurs where both the voltage and current are non-zero, leading to switching losses. These losses increase with higher switching frequencies and reduce efficiency.

    Gate Drive Losses: The energy required to charge and discharge the gate capacitance of the MOSFET during each switching cycle adds to the losses, particularly at high switching frequencies.

4. Diode Non-Idealities

    Forward Voltage Drop: Real diodes have a forward voltage drop (typically 0.7V for silicon diodes or around 0.2V for Schottky diodes). This voltage drop translates into power loss during each switching cycle, reducing the output voltage and efficiency.

    Reverse Recovery Time: In fast-switching circuits, the reverse recovery time of the diode can cause inefficiencies. During reverse recovery, the diode briefly conducts in the wrong direction, which wastes energy and generates heat.

5. Parasitic Elements in the Circuit

    Parasitic Capacitances and Inductances: The layout and physical characteristics of the circuit introduce parasitic capacitances and inductances that cause oscillations and radiate electromagnetic interference (EMI). These parasitics can lead to switching noise, voltage spikes, and reduced performance, particularly at high frequencies.

    PCB Trace Resistance: The resistance of PCB traces introduces small, but significant, power losses, especially in high-current paths. This results in additional heat generation and efficiency reduction.

Overall Impact of Non-Idealities on Performance

The combined effects of these non-idealities lead to:

- Reduced Efficiency: Real-world efficiency is always lower than the ideal case due to the resistive and switching losses in components.

- Voltage Ripple: Inductor and capacitor non-idealities contribute to larger voltage ripple, making it harder to regulate a stable output voltage.
- Thermal Issues: Power dissipation in resistive elements (inductor, capacitor, switch, and diode) generates heat, which can degrade performance over time and potentially cause thermal runaway if not managed properly.
- Degraded Dynamic Response: Non-idealities affect the transient response, making it slower and less predictable. The system may exhibit overshoot, longer settling times, or reduced stability margins.

**How Non-Idealities Influence the Control Strategy**

Non-ideal components must be considered when designing the PID controller or any control strategy for the boost converter. For instance:

- Controller Tuning: The presence of non-idealities may necessitate retuning of the PID gains. Increased resistance or slower response times may require lower KpKp​ values to avoid overshoot, or higher KiKi​ to compensate for steady-state errors caused by voltage drops in non-ideal components.

- Compensation for Non-Idealities: Techniques like feedforward control or gain scheduling can be employed to compensate for known losses (e.g., diode forward voltage drop). For example, the controller could be designed to inject extra control effort to account for losses in the switch or inductor.

- Lowering Switching Frequency: Reducing the switching frequency can help mitigate switching losses, but this comes at the cost of a slower transient response and potentially larger voltage ripple. This tradeoff needs to be balanced when designing the system.

### Large Component Values: Challenges and Solutions

In boost converter design, both the capacitor and inductor play a key role in determining the converter’s performance, particularly in terms of ripple, stability, and transient response. However, the required values for these components can become quite large in certain cases, posing practical challenges.

1. Large Inductor Values:

    - Large inductance values often require physically larger inductors, which can be bulky, heavy, and costly. In power electronics applications, inductors with high inductance and low series resistance (DCR) are ideal, but such components tend to be large and suffer from higher core losses.
    - Inductors with larger values may also have slower transient responses, making them less responsive to sudden changes in load or input conditions.

    Possible Solutions:
    - Use of High-Frequency Switching: By increasing the switching frequency, the required inductance can be reduced. This allows for the use of smaller inductors without significantly affecting the performance of the converter. However, higher switching frequencies also introduce additional challenges such as increased switching losses and electromagnetic interference (EMI).
    - Multiphase or Interleaved Converters: Another approach is to use a multiphase boost converter, where multiple smaller inductors are used in parallel. This spreads the current across multiple phases, reducing the size of each individual inductor while still achieving the desired overall inductance and current capacity. Interleaving also helps reduce output ripple and inductor current ripple.
    - Coupled Inductors: In some designs, coupled inductors can be used, which combine inductive elements into a smaller package while maintaining the necessary inductance. This can help reduce the overall size of the converter without sacrificing performance.

2. Large Capacitor Values:


    - Large capacitors are needed to reduce output voltage ripple and store sufficient energy during the off-cycle of the converter. However, high-capacitance values can result in physically large capacitors, especially if low Equivalent Series Resistance (ESR) capacitors are needed to maintain performance and efficiency.
    - Electrolytic capacitors, which are often used for large capacitance, have higher ESR compared to ceramic capacitors, leading to increased ripple and potential heating issues. Physically large capacitors also introduce design constraints in terms of PCB layout and mechanical stability.
    Possible Solutions:
    - Use of Multiple Smaller Capacitors in Parallel: To achieve the desired capacitance value while maintaining a compact design, multiple smaller capacitors can be placed in parallel. This approach not only meets the capacitance requirement but also helps reduce ESR since the ESR of capacitors in parallel is reduced. A combination of electrolytic capacitors (for bulk energy storage) and ceramic capacitors (for high-frequency filtering) can be used.
    - Higher Switching Frequency: Similar to the inductor, increasing the switching frequency reduces the size of the required capacitor since the output ripple is inversely related to frequency. This allows for smaller capacitors to be used while maintaining the same level of ripple suppression.
    - Advanced Capacitor Technologies: Technologies like polymer capacitors or supercapacitors offer higher capacitance per unit volume and lower ESR compared to traditional electrolytic capacitors, providing a more compact solution.

#### Inrush Current in Boost Converters
**Why is Inrush Current Large?**

Inrush current refers to the initial surge of current when the converter is first powered on. In boost converters, this can occur when the inductor and diode allow an uncontrolled amount of current to flow into the system due to the lack of initial energy storage in the capacitor or control circuitry. The inrush current can be particularly high in the following cases:

Inductor Charging: When the converter is first turned on, the inductor has no initial current flowing through it. As the circuit starts to operate, the inductor begins to charge, and if there is no control over the switching process, it can draw a large amount of current from the input source. This can cause high peak currents that may damage the inductor, switch, or other components in the converter.

Capacitor Charging: At startup, the output capacitor is uncharged and behaves like a short circuit. This results in a large surge of current as the input source and the converter’s inductor attempt to charge the capacitor to the output voltage. The initial current drawn from the input can be many times larger than the steady-state operating current, leading to stress on components and potential damage.

**How to Mitigate Inrush Current?**

**Soft-Start Mechanism** 
A common method to reduce inrush current is to implement a soft-start function in the controller. Soft-start limits the initial duty cycle of the converter, gradually increasing it over time. This controls the inductor current and output voltage rise, preventing sudden surges of current. Soft-start can be implemented by ramping up the reference voltage or limiting the duty cycle during startup.

**Pre-Charging the Output Capacitor** 
In some designs, the output capacitor can be pre-charged to a certain voltage before the converter begins switching. This can be done using a small resistor or current-limiting circuit in series with the capacitor, reducing the initial current spike when the converter first turns on. Once the capacitor is partially charged, the converter can switch into normal operation.

**Current Limiting Circuit** 
A current-limiting circuit can be added to prevent excessive current from flowing through the inductor and the diode during startup. This can be achieved using a current sense resistor in combination with a control loop that limits the maximum allowable current through the circuit. If the current exceeds a preset threshold, the converter can reduce its duty cycle or momentarily halt operation to prevent component damage.

**NTC Thermistor** 
A Negative Temperature Coefficient (NTC) thermistor can be placed in series with the input or inductor. At startup, the NTC thermistor presents a high resistance, limiting the inrush current. As current flows and the thermistor heats up, its resistance decreases, allowing normal operation with minimal losses during steady-state conditions.

**Active Inrush Limiting** 
Some designs employ an active inrush limiting circuit, which uses a MOSFET or relay in series with the input to control the inrush current. When the converter starts up, the MOSFET or relay limits the current by reducing the input voltage until the capacitor is charged. Once the converter reaches steady-state, the MOSFET is fully turned on, minimizing losses

### Control theory 

**Stability Analysis of the Boost Converter**

Boost converters are inherently non-linear due to their switching nature, which complicates their control. For effective control design, the converter is often linearized around a steady-state operating point using small-signal models. This linearization provides insight into how small perturbations in the input or load affect the system’s stability and performance.

**Small-Signal Modeling**

To perform stability analysis, a small-signal model of the boost converter is developed. This involves perturbing the duty cycle, inductor current, and output voltage around their steady-state values. The system can be described by linear differential equations that represent the relationship between input, duty cycle, and output voltage in the frequency domain. The small-signal transfer function of the boost converter can be approximated as:
Gv(s)=ΔVout(s)ΔD(s)=Vout1−D⋅1LCs2+(ReqL)s+1
Gv​(s)=ΔD(s)ΔVout​(s)​=1−DVout​​⋅LCs2+(Req​L)s+11​

where:

L is the inductance,
C is the output capacitance,
Req​ is the equivalent series resistance of the components, and
s is the Laplace transform variable.

This transfer function provides the foundation for frequency-domain analysis, enabling the use of tools like Bode plots and Nyquist criteria to assess stability.

**Bode Plot and Phase Margin**

A Bode plot is often used to analyze the system’s gain and phase margins, which are critical for ensuring stable operation under feedback control. The gain margin indicates how much the system gain can increase before the system becomes unstable, while the phase margin provides a measure of how close the system is to oscillation.

For a well-designed PID controller, the goal is to achieve a phase margin of around 45 to 60 degrees, ensuring that the converter remains stable with good transient response. A higher phase margin generally corresponds to more stability, though it may slow down the system's response.
Compensator Design

In some cases, a simple PID controller may not suffice due to the complex dynamics of the boost converter, particularly under varying loads or input disturbances. To further refine control, compensators such as lead-lag compensators can be added to improve the dynamic response and stability.
Lead Compensator

A lead compensator introduces phase lead, which helps to increase the phase margin and improve the transient response of the system. It can be designed using the following transfer function:
Glead(s)=K⋅(s+ωz)(s+ωp)
Glead​(s)=K⋅(s+ωp​)(s+ωz​)​

where:

ωzωz​ is the zero frequency,
ωpωp​ is the pole frequency (with ωz>ωpωz​>ωp​),
KK is the gain.

The lead compensator improves the system's bandwidth and response time, making it more resilient to disturbances.
Lag Compensator

A lag compensator, on the other hand, is used to improve low-frequency performance, reducing steady-state error while sacrificing some response speed. Its transfer function is:
Glag(s)=K⋅(s+ωp)(s+ωz)
Glag​(s)=K⋅(s+ωz​)(s+ωp​)​

where ωp<ωzωp​<ωz​. The lag compensator is effective in improving the DC gain and reducing steady-state error, making it useful in applications where precise voltage regulation is essential.

**Practical Implementation of PID on the Raspberry Pi Pico**

Implementing a PID controller on the Raspberry Pi Pico involves several key considerations, particularly in managing real-time constraints and ensuring efficient execution of control algorithms.
ADC and Feedback Signal Processing

The Raspberry Pi Pico's built-in Analog-to-Digital Converter (ADC) is crucial for measuring the output voltage in real-time. The accuracy and resolution of the ADC (12-bit) must be sufficient to capture fine variations in the output voltage for precise feedback control. However, noise in the ADC signal can significantly degrade the performance of the PID controller. Therefore, filtering techniques such as low-pass filters are applied to smooth the ADC readings before feeding them into the control loop.
PWM Generation

The Raspberry Pi Pico features multiple PWM channels that can be used to control the switching of the boost converter's MOSFET. The duty cycle is adjusted in real-time by the PID controller to regulate the output voltage. To achieve high efficiency and minimize switching losses, the switching frequency fsfs​ is chosen carefully. A higher frequency reduces the size of passive components but increases switching losses. Typical switching frequencies for boost converters range from 20 kHz to 200 kHz, depending on the application.
#### Timing and Interrupt Handling

The PID algorithm must be executed at regular intervals, synchronized with the PWM cycle. This is typically achieved using timers or interrupts on the Raspberry Pi Pico, ensuring that the control loop runs at a fixed frequency. A typical control loop frequency ranges from 1 kHz to 10 kHz, depending on the converter's dynamics and desired response speed.
Code Optimization

Given the real-time nature of the control, the PID algorithm should be optimized for execution speed. This includes using fixed-point arithmetic instead of floating-point, which can be computationally expensive on microcontrollers. The PID gains KpKp​, KiKi​, and KdKd​ can be pre-scaled to fit within the limits of fixed-point precision.
Advanced Control Techniques

While a PID controller provides a robust solution for most applications, advanced control techniques may be required for more demanding systems where disturbances are frequent or where fast transient response is essential.
Sliding Mode Control (SMC)

- Sliding Mode Control (SMC) is a non-linear control method that forces the system to "slide" along a predetermined surface in the state space, leading to high robustness against parameter variations and external disturbances. In the context of boost converters, SMC ensures that the output voltage remains stable even in the presence of large disturbances, such as sudden load changes or input voltage drops.

The control law for SMC is defined as:
u(t)={Vin,if σ(x)>00,if σ(x)≤0
u(t)={Vin​,0,​if σ(x)>0if σ(x)≤0​

where σ(x)σ(x) is the sliding surface defined based on the system's state variables.
Model Predictive Control (MPC)

- Model Predictive Control (MPC) is another advanced technique that predicts the future behavior of the system based on a dynamic model and optimizes the control inputs accordingly. In the case of a boost converter, MPC can predict future output voltage and adjust the duty cycle to minimize deviations from the reference, providing superior performance compared to classical PID control in systems with rapid disturbances.
Real-World Performance Tuning

In practice, tuning a PID controller on a boost converter requires balancing various performance metrics. The Ziegler-Nichols method is commonly used to set initial PID gains, followed by fine-tuning based on system response. For example:

- Aggressive tuning (high Kp​, moderate Kd, low Ki) is ideal for applications requiring fast response but can introduce overshoot and oscillations.
- Conservative tuning (low Kp​, higher Ki, moderate Kd) ensures smooth and stable operation with minimal overshoot but may result in slower response times.

Further tuning may be performed empirically through trial-and-error to ensure the desired trade-off between speed, accuracy, and stability
    
### Stability of the Boost Converter

Boost converters are dynamic systems with non-linearities that result from their switching nature. In steady-state, the converter’s operation can be approximated by a linear model to simplify the analysis of stability. The focus of stability analysis is ensuring that the closed-loop system (with the PID controller) remains stable under all operating conditions, i.e., the output converges to the desired voltage without oscillation or divergence, despite disturbances or parameter variations.
#### Bode Plot: Frequency-Domain Analysis

A Bode plot is a key tool for analyzing the stability of a system in the frequency domain. It consists of two plots:

The magnitude plot shows how the gain of the system changes with frequency.
    
The phase plot shows how the phase shift between input and output varies with frequency.

For a boost converter under closed-loop control, the open-loop transfer function G(s)H(s)G(s)H(s), where G(s)G(s) represents the plant (the boost converter) and H(s)H(s) is the feedback (the PID controller), is used to generate the Bode plot.
Gain Margin and Phase Margin

Two critical metrics for determining system stability are the gain margin and phase margin:

- Gain Margin (GM): The gain margin is the amount of gain that can be increased before the system becomes unstable. It is determined by the distance (in dB) from the point where the phase reaches -180° to the point where the magnitude plot crosses 0 dB. A gain margin of at least 6 dB is typically considered stable.

- Phase Margin (PM): The phase margin is the difference between the phase at the gain crossover frequency (where the magnitude is 0 dB) and -180°. A phase margin of at least 45° is generally desired to ensure a good balance between fast response and stability.

To improve phase margin, a lead compensator can be introduced, which adds phase lead at higher frequencies, allowing the system to stabilize faster without significant overshoot. Conversely, to improve steady-state accuracy, a lag compensator may be used, which improves low-frequency gain but may reduce phase margin if not carefully tuned.
Example of Bode Plot Analysis

Consider the transfer function of a boost converter in the s-domain:
G(s)=VoutVin⋅1LCs2+(ReqL)s+1
G(s)=Vin​Vout​​⋅LCs2+(Req​L)s+11​

This represents a second-order system with a double pole at:
ωn=1LC
ωn​=LC
​1​

At low frequencies, the system behaves as a low-pass filter with a flat magnitude response. However, as frequency increases and approaches the natural frequency ωnωn​, the phase begins to drop sharply, and the magnitude decreases at a rate of -40 dB/decade beyond the resonant frequency. The sharp phase drop near resonance is where stability issues arise, and proper compensator design is crucial.

#### Nyquist Plot: Stability in the Complex Plane

The Nyquist plot provides a different perspective by plotting the frequency response of the open-loop transfer function G(s)H(s)G(s)H(s) in the complex plane. The key principle used here is the Nyquist criterion, which relates the number of poles and zeros of the closed-loop system to the encirclements of the critical point (-1, 0) on the plot.
Nyquist Criterion for Stability

The Nyquist criterion states that for the system to be stable, the Nyquist plot of the open-loop transfer function must not encircle the critical point (-1, 0) in the complex plane. The number of encirclements of the critical point corresponds to the number of unstable poles in the open-loop system. If the plot encircles the point in a clockwise direction, it indicates that the system will become unstable.

Nyquist plots are particularly useful when dealing with non-minimum phase systems or systems with time delays, as they reveal stability information that may not be as easily observed in a Bode plot. A boost converter typically has a right-half-plane zero (RHP zero) due to its nature, making Nyquist analysis important.
Inserting Poles and Zeros for Stability Fine-Tuning

Adding poles and zeros is a common strategy for adjusting the stability margins and fine-tuning the frequency response of the boost converter’s control system.
Lead Compensator

A lead compensator introduces a zero to the system, typically at a frequency below the crossover frequency, and a pole at a higher frequency. The effect is an increase in the phase margin, which improves the system's stability and transient response.

The transfer function of a lead compensator is:
Glead(s)=K⋅s+ωzs+ωp
Glead​(s)=K⋅s+ωp​s+ωz​​

where ωzωz​ and ωpωp​ are the frequencies of the zero and pole, respectively, and ωz<ωpωz​<ωp​. The compensator adds positive phase at mid-to-high frequencies, helping to counteract the phase lag introduced by the boost converter’s double-pole characteristic.
Lag Compensator

A lag compensator introduces a pole at a lower frequency and a zero at a higher frequency. This improves the low-frequency gain, enhancing the system's ability to track slow changes in input or load, thus reducing steady-state error.

The transfer function of a lag compensator is:
Glag(s)=K⋅s+ωps+ωz
Glag​(s)=K⋅s+ωz​s+ωp​​

with ωp<ωzωp​<ωz​. Lag compensators, however, introduce additional phase lag, so they must be used judiciously to avoid destabilizing the system.
PID Controller with Additional Poles and Zeros

In some cases, it is useful to extend the PID controller by adding extra zeros or poles beyond the classical PID structure. For example, adding an additional zero can enhance the control of high-frequency dynamics, while an extra pole can attenuate high-frequency noise or unwanted resonances. The extended PID transfer function becomes:
GPID(s)=Kp+Ki⋅1s+Kd⋅s+a⋅s+bs+c
GPID​(s)=Kp​+Ki​⋅s1​+Kd​⋅s+s+ca⋅s+b​

where a,b,ca,b,c represent the additional zero and pole parameters. This enhances the flexibility of the PID controller in compensating for specific dynamics within the boost converter.
Nonlinearities in Boost Converters

Boost converters exhibit nonlinear behavior due to the switching nature of the circuit and the time-varying nature of the system components. The nonlinearity arises primarily from two sources:

- Switching Nonlinearity:
    The periodic switching of the MOSFET creates discontinuities in the system. The converter operates in two different states (ON and OFF), resulting in nonlinear differential equations that describe the system's behavior. This is referred to as piecewise linear dynamics.

- Right-Half-Plane Zero (RHP Zero):
    Boost converters exhibit an RHP zero in their transfer function when operating in continuous conduction mode (CCM). This zero introduces a non-minimum phase behavior, meaning that any attempt to increase the output voltage initially results in a dip, followed by an increase. This behavior complicates the control, as it limits the achievable bandwidth and causes phase lag.

- Inductor Saturation:
    As the current through the inductor increases, its inductance may decrease due to magnetic core saturation, leading to nonlinearities in the inductor’s behavior. This impacts the system's dynamics and can degrade performance.

- Load Nonlinearity:
In real-world applications, the load may vary unpredictably, introducing additional nonlinearity in the system response. A boost converter driving a dynamic load (such as motors or LEDs) faces sudden changes in current demand, causing variations in output voltage.

#### Impact of Nonlinearities on Performance

The nonlinearities in the boost converter lead to several challenges in control:

- Reduced Stability Margins: Nonlinear effects like RHP zeros and switching dynamics reduce the stability margins, making it harder for classical PID control to maintain stability.
- Poor Transient Response: The presence of RHP zeros introduces a delay in the response, causing an initial undershoot before the output reaches the desired value.
- Increased Overshoot and Oscillations: Nonlinearities tend to increase overshoot and induce oscillations in the output voltage, particularly when the load changes rapidly.

A natural question that comes to mind is 'Can PID Control Actually Handle Nonlinearities?'
And the answer is , well , almost.

A classical PID controller is designed for linear systems, and while it can handle mild nonlinearity, it struggles with highly nonlinear systems like a boost converter operating under extreme conditions.
Strengths of PID in Nonlinear Systems

- Simplicity: The PID controller is straightforward to implement and provides reasonable control over the boost converter in most operating conditions.
- Steady-State Error Correction: The integral component helps eliminate steady-state error, even in the presence of some nonlinearities, such as load changes.

Limitations of PID in Nonlinear Systems

- RHP Zero Handling: The presence of an RHP zero limits the effectiveness of the derivative term, as the system requires a phase lead for stability but the RHP zero introduces phase lag.Luckly boost converter is a maximal phase system, so this won't be a problem.

-Slow Response to Large Disturbances: The PID controller may exhibit slow response times when dealing with large disturbances or rapid changes in input voltage or load.Once again this shouldn't be a problem , especially since circuit is designed to work in steady-state mode and no sudden changes are present.

To handle severe nonlinearities, advanced control techniques such as sliding mode control (SMC) or model predictive control (MPC) are often preferred. These methods offer robust performance even under non-ideal operating conditions, where classical PID control might fail.They will be briefly stated with in the following chapters.

### Alternative Control Algorithms for Boost Converters

While PID control is popular due to its simplicity, other advanced control methods can offer better performance in systems like boost converters, which exhibit significant nonlinearity and time-varying dynamics.

1. Sliding Mode Control (SMC)

    Sliding Mode Control (SMC) is a robust nonlinear control technique that forces the system to "slide" along a predefined surface in the state space. SMC is well-suited for systems with significant disturbances and parameter variations, as it offers strong robustness and stability.

    Pros: Highly robust to parameter changes and disturbances, fast dynamic response.
    Cons: May cause high-frequency switching (chattering), which can increase losses and wear out components.

SMC works by defining a sliding surface σ(x)σ(x) that represents the desired behavior of the system. The control law switches between two states based on the value of σ(x)σ(x), ensuring that the system stays on or near the sliding surface.

2. Model Predictive Control (MPC)

Model Predictive Control (MPC) uses a dynamic model of the system to predict future behavior and optimize the control inputs over a finite time horizon. MPC is ideal for handling constraints and nonlinearity, making it a powerful tool for controlling boost converters.

Pros: Optimizes performance over a time horizon, handles constraints and nonlinearity effectively.
Cons: Computationally intensive, requiring significant processing power, making it less suitable for low-cost microcontrollers like the Raspberry Pi Pico.

MPC adjusts the duty cycle based on predictions of the future output voltage, minimizing deviations from the reference. It’s particularly effective in systems with frequent disturbances or rapid dynamic changes.

3. H∞ Control

H∞ control is a robust control technique designed to minimize the worst-case gain from disturbances to output. It provides excellent disturbance rejection and robust performance, even in the presence of uncertainty or nonlinearity in the system model.

Pros: Strong disturbance rejection, robust performance even with model uncertainty.
Cons: Complex design process, may require advanced modeling and tuning efforts.

H∞ controllers are typically used in high-performance applications where robustness is critical, though they are more complex to design than traditional PID controllers.

4. Fuzzy Logic Control

Fuzzy Logic Control (FLC) is a knowledge-based control method that mimics human decision-making. It does not rely on a mathematical model of the system but instead uses a set of fuzzy rules to control the system. FLC is particularly useful in systems with highly nonlinear behavior and uncertainty.

Pros: Can handle nonlinearity and uncertainty effectively, does not require a precise model of the system.
Cons: Performance is highly dependent on the quality of the fuzzy rule set, can be difficult to design and tune.

Fuzzy control is often used in applications where the system dynamics are too complex for traditional control methods or where the system is subject to unpredictable disturbances.

### Pros and Cons of PID Control
- Pros:

    - Simplicity: PID controllers are straightforward to implement and widely used across industries due to their simplicity. They require only three parameters—proportional (Kp​), integral (Ki), and derivative (Kd)—to control a system effectively.
    - Wide Applicability: PID control is versatile and can be applied to a variety of systems, including boost converters, as long as the system dynamics are relatively linear or well-behaved within a certain operating range.
    - Real-Time Feedback: PID provides real-time correction based on system feedback. The proportional term reacts immediately to changes, the integral term corrects steady-state errors, and the derivative term helps to predict future errors based on current trends.
    - Good Performance for Linear Systems: In linear systems, PID control offers excellent performance in terms of stability, transient response, and steady-state accuracy.

- Cons:

    - Limited Performance in Nonlinear Systems: PID controllers assume a linear relationship between input and output. In nonlinear systems like the boost converter, PID struggles with phenomena like saturation, switching, and right-half-plane (RHP) zeros, leading to poor performance.
    - Poor Handling of Time-Varying Dynamics: In systems with varying parameters (e.g., changing loads or input voltages), PID may fail to maintain stability without retuning. This is especially problematic in boost converters, where load conditions can change dynamically.
    - Derivative Sensitivity to Noise: The derivative term is highly sensitive to noise, which is common in real-world systems. Noise in the feedback signal can lead to excessive and erratic control actions.
    - Difficult Tuning: Proper tuning of PID gains requires trial and error or advanced tuning methods like Ziegler-Nichols. Improperly tuned PID controllers may result in overshoot, oscillations, or sluggish performance.

#### Using PI Control in a Boost Converter

For a boost converter, a PI (Proportional-Integral) controller is often sufficient and sometimes preferred over a full PID controller. This is because the derivative term Kd, while useful in improving the response in some systems, tends to be less effective in power electronics applications, especially in the presence of noise and switching dynamics. Here's why PI control can be a good option:
**Why PI Control?**

- Simplified Control: By eliminating the derivative term, PI control becomes simpler to implement and tune. This is particularly useful in systems where the derivative term introduces noise-related problems.

- Handling Steady-State Error: The integral term Ki​ is essential for eliminating steady-state error, which is critical in boost converters that need precise regulation of the output voltage over time.

- Reduction of Noise Sensitivity: Removing the derivative term reduces sensitivity to noise, which is a common issue in switching converters where high-frequency components can corrupt feedback signals.

**Limitations of PI Control**

- Slower Transient Response: Without the derivative term, the system may have a slower response to disturbances. This can result in increased overshoot or longer settling times.
- Less Predictive Action: PI control does not take into account the future behavior of the error, potentially leading to more oscillations in systems with significant nonlinearity.

**PI Control Transfer Function**

The transfer function of a PI controller is given by:
GPI(s)=Kp+Kis
GPI​(s)=Kp​+sKi​​

This introduces one zero at s=−KiKps=−Kp​Ki​​, which can help improve the phase margin and enhance stability in the boost converter.
Tips and Tricks to Use PID in Nonlinear Systems

Although PID control is not inherently well-suited to nonlinear systems like the boost converter, several techniques can help mitigate its limitations:
1. Gain Scheduling


    Gain scheduling is a method where the PID controller parameters KpKp​, KiKi​, and KdKd​ are dynamically adjusted based on operating conditions (e.g., input voltage, load, or duty cycle). This approach tailors the PID gains to different regions of operation, improving stability and performance under varying conditions.

    For instance, you might use lower gains during startup (to avoid overshoot) and higher gains during steady-state operation for faster transient recovery.

2. Anti-Windup for Integral Action

    In systems with large disturbances or saturation, the integral term can accumulate excessive error (called windup), leading to instability or slow recovery when the system exits saturation. Anti-windup schemes prevent this by limiting the accumulation of the integral term when the system is saturated, ensuring a quicker recovery when normal conditions are restored.

3. Filtered Derivative Term

    To reduce the sensitivity of the derivative term to high-frequency noise, you can apply a low-pass filter to the derivative component. The filtered derivative is less prone to amplifying noise, making the control action smoother while still preserving the predictive benefits of Kd.

    The filtered derivative term is often expressed as:
    Dfiltered=Kdsτds+1
    Dfiltered​=τd​s+1Kd​s​

    , where τd is the time constant of the filter.

4. Adaptive PID Control

    Adaptive PID controllers continuously adjust the controller gains in real time based on system performance. This allows the controller to adapt to changing system dynamics (such as load variations) without requiring manual retuning. Adaptive control algorithms use estimation techniques to identify system parameters and adjust the PID gains accordingly.

5. Feedforward Control

    A feedforward control can be combined with the PID controller to improve the response to known disturbances (e.g., changes in input voltage or load). The feedforward term adds a control signal based on the disturbance, reducing the load on the feedback PID controller and improving the transient response.

