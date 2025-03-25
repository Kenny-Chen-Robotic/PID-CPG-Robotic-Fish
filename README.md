# Autonomous Robotic Fish
This repository is about the algorithm framework of a wire-driven robotic fish, containing a basic CPG module which is coded with C (implementing in STM32 micro-controller) and an closed-loop orientation module combining with trajectory generator (e.g. Bezier curve) being coded with Python (implementing in Raspberry Pi 4B). The algorithm is robust and applied for scientific experiment in real world directly, thus hareware connection is indispendable at the very beginning. Moreover, this desperate design offers a convenient hardware platform for validating several AI agent algorithm by substituting PID algorithm. 

Relevent Link: https://www.sciencedirect.com/science/article/pii/S002980182300817X.

DOI: https://doi.org/10.1016/j.oceaneng.2023.114433.

# Prototype introduction
<div align="center">
  <img src="images/3D_swimming.png" alt="3D Swimming" style="width: 800px; height: auto;"/>

  *Figure1: 3D swimming*
</div>


<br>  <!-- 这是空行间隔 -->

<div align="center">
  <img src="images/Cruising.jpg" alt="Cruising" style="width: 650px; height: auto;"/>

  *Figure2: Cruising*
</div>


<br>  <!-- 这是空行间隔 -->

<div align="center">
  <img src="images/Hareware_framework.png" alt="Hardware Framework" style="width: 650px; height: auto;"/>
  
  *Figure3: Hardware Framework*
</div>


<br>  <!-- 这是空行间隔 -->


<div align="center">
  <img src="images/Control_diagram.png" alt="Control diagram" style="width: 650px; height: auto;"/>
  
  *Figure4: Control diagram*
</div>

# Advanced research
DRL Agent robotic / LLM controller / Sim2real with ROS & Gazebo.

<br>  <!-- 这是空行间隔 -->
