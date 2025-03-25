# Autonomous Robotic Fish
This repository is about the algorithm framework of a wire-driven robotic fish, which contains basic CPG module coded with C codes (implementing in STM32 micro-controller) and orientation module and trajectory generator (e.g. Bezier curve) are coded with Python (impplementing in Raspberry Pi 4B). The algorithm is robust and applied for experiment testing in real world directly, hareware connection is indispendable at the very beginning. Moreover, this desperate design offers a convenient hardware platform for validating other AI agent algorithm by substituting PID algorithm. 

Relevent Links: https://www.sciencedirect.com/science/article/pii/S002980182300817X

DOI: https://doi.org/10.1016/j.oceaneng.2023.114433

<div align="center">
  <img src="images/3D_swimming.png" alt="3D Swimming" style="width: 800px; height: auto;"/>
  
  *Figure1: 3D swimming*
</div>


<br>  <!-- 这是空行间隔 -->


<div align="center">
  <img src="images/Hareware_framework.png" alt="Hardware Framework" style="width: 650px; height: auto;"/>
  
  *Figure2: Hardware Framework*
</div>


<br>  <!-- 这是空行间隔 -->


<div align="center">
  <img src="images/Control_diagram.png" alt="Control diagram" style="width: 650px; height: auto;"/>
  
  *Figure3: Control diagram*
</div>


Advanced work: DRL Agent robotic / LLM controller / Sim2real with ROS & Gazebo
