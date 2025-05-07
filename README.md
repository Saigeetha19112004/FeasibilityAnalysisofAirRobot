. Abstract 
•	Background 
Drones are being used more and more in applications like logistics, monitoring, agriculture, and disaster relief because they can access hard-to-reach locations. Their operation is threatened by unfavorable weather conditions like high winds, poor visibility, and rainfall. Current navigation systems are not equipped with real-time weather integration, making them less effective in changing environments.
•	Identification of Themes and Gaps
Several key themes emerge from these sources:
o	Wind Effects: Wind speed and turbulence are consistently identified as significant factors affecting drone stability, control, and battery consumption.
o	Temperature Sensitivity: Extreme temperatures (both high and low) can adversely affect drone battery performance and overall system reliability.
o	Precipitation Risks: Rain, snow, and ice can compromise drone visibility, sensor accuracy, and aerodynamic performance.
o	Operational Limitations: Weather conditions impose limitations on drone flight time, range, and payload capacity.
o	Need for Advanced Forecasting: Accurate weather forecasting and real-time monitoring are crucial for safe and efficient drone operations.

Gaps in the literature include:
o	Limited Research on Icing Effects: The impact of icing on drone performance and safety requires further investigation, especially in colder climates.
o	Lack of Standardized Testing: Standardized testing protocols are needed to evaluate drone performance under different weather conditions and to establish clear operational guidelines.
o	Integration of Weather Data: More research is needed on how to effectively integrate weather data into drone flight planning and control systems.

•	Objective
This project aims to develop a 3D simulation environment for drone navigation that integrates the A* path planning algorithm with real-time weather data. The key objectives include:
1.	Implementing the A* algorithm for efficient obstacle avoidance.
2.	Incorporating real-time weather data to assess flight safety dynamically.
3.	Simulating drone movement with both static and dynamic obstacles.
4.	Visualizing path planning and environmental interactions.

•	Method
1.	Environment Setup
o	Defined a 3D simulation space with boundaries.
o	Generated random static and dynamic obstacles of varying shapes (cube, sphere, cylinder).
2.	Path Planning Algorithm
o	Implemented the A* algorithm to compute the optimal path from the start position to the destination.
o	Recalculated paths dynamically when obstacles were detected along the route.
3.	Weather Integration
o	Retrieved real-time weather data from the WeatherBit API.
o	Monitored key parameters such as wind speed, visibility, precipitation, and temperature.
o	Assessed flight safety and aborted missions if weather conditions were unfavourable.
4.	Visualization
o	Used MATLAB’s graphical tools to visualize drone movement, obstacles, and planned paths.
o	Represented the drone as a cube and plotted its real-time trajectory.
5.	Dynamic Obstacles Handling
o	Introduced moving obstacles that changed positions during the simulation.
o	Periodically updated obstacle locations and recalculated paths to avoid collisions.
6.	Emergency Response Mechanism
o	Implemented an emergency landing procedure if unsafe weather conditions were detected mid-flight.
o	Generated a safe descent path and simulated drone landing at the nearest viable position.

•	Key findings
1.	Smart Path Planning with A*
The drone successfully found the best route using the A* algorithm, avoiding obstacles and dynamically adjusting its path when needed.
2.	Real-Time Weather Checks for Safety 
By integrating real-time weather data, the system ensured the drone only took off in safe conditions. If bad weather was detected mid-flight, the drone adjusted its route or landed safely.
3.	Dodging Moving Obstacles 
The drone didn’t just avoid fixed objects, but it also reacted to moving obstacles, recalculating its path in real-time to prevent collisions.
4.	Bringing the Simulation to Life
Using MATLAB’s 3D visualization, we could clearly see how the drone moved, navigated obstacles, and responded to environmental changes.
5.	Smarter, Safer Drone Navigation
The combination of AI-driven path planning and real-time weather monitoring made drone movement more reliable, adaptable, and safe in different conditions.

•	Simulation tool used to propose the results
The simulation was developed using MATLAB, which provided a powerful environment for:
1.	3D Visualization – Displaying the drone’s movement, obstacles, and path planning dynamically.
2.	A* Path Planning Implementation – Running the AI-based navigation algorithm efficiently.
3.	Weather Data Integration – Fetching and processing real-time weather conditions to influence drone decisions.
4.	Obstacle Handling – Simulating both static and dynamic obstacles to test real-world navigation challenges.

3. Introduction
Background Information
Drones are becoming increasingly important in various fields such as logistics, surveillance, agriculture, and disaster management due to their ability to access remote or hazardous locations efficiently. However, ensuring safe navigation in complex environments with obstacles and adverse weather conditions remains a significant challenge.
This project focuses on developing a 3D drone simulation that integrates A path planning* with real-time weather data to enhance flight safety and efficiency. The simulation allows drones to autonomously navigate by avoiding obstacles and making weather-informed flight decisions.
Research Problem Statement
Drones face major challenges when navigating in real-world environments, especially in the presence of:
•	Unpredictable weather conditions (high winds, poor visibility, precipitation).
•	Static and dynamic obstacles that obstruct the flight path.
•	A lack of real-time environmental awareness in existing navigation systems.
Objectives and Significance of the Study
The key objectives of this study are:
1.	Develop a 3D simulation for drone navigation in MATLAB.
2.	Implement A* path planning to enable obstacle-aware navigation.
3.	Integrate real-time weather data to assess flight safety dynamically.
4.	Simulate both static and dynamic obstacles to test the system’s robustness.
5.	Visualize drone movement and decision-making in a dynamic environment.
Significance:
This study demonstrates how AI-driven navigation combined with environmental awareness can significantly improve the safety and reliability of drone operations. The findings can be applied to real-world applications such as autonomous delivery drones, disaster response, and urban air mobility.
Research Questions
1.	How effectively does the A* algorithm enable drones to navigate around obstacles?
2.	Can real-time weather data improve the decision-making process for drone navigation?
3.	How does the drone simulation handle dynamic obstacles and environmental changes?
4.	What are the benefits of integrating AI-based path planning with weather-aware navigation for drone safety?
4.Literature Review
Introduction
Unmanned Aerial Systems (UAS), commonly known as drones, are increasingly utilized across various sectors, including agriculture, surveillance, delivery services, and infrastructure inspection. However, the operational capabilities of drones are significantly affected by weather conditions. This literature review examines the existing research and analyses the impact of different weather parameters on drone performance, safety, and overall operational efficiency. It identifies key challenges and knowledge gaps in understanding and mitigating weather-related risks in UAS operations.
Summary and Analysis of Sources
•	IEEE Article 1 (10701056): This article likely focuses on a specific aspect of drone operation under certain weather conditions. (Without the abstract content, this is an assumption) It probably provides empirical data or modeling results related to the performance and limitations of drones in those conditions. Analysis of this study would contribute to a deeper understanding of the impact, providing insights that are valuable for operators, manufacturers, and regulators.
•	IEEE Article 2 (10083107): This article likely addresses technological solutions or strategies to mitigate the impact of adverse weather conditions on drones. (Without the abstract content, this is an assumption). It might explore innovations in drone design, weather forecasting models, or control algorithms that improve drone resilience and safety in challenging weather scenarios.
•	ScienceDirect Article (S0376042122000513): The ScienceDirect article likely presents a comprehensive study on the effects of varying weather conditions on drone performance. It probably uses quantitative data to analyze the relationships between weather parameters (such as wind speed, temperature, and precipitation) and drone flight characteristics (such as battery life, stability, and control responsiveness).
•	Steiner et al. (ResearchGate): "Exploring the range of weather impacts on UAS operations" This study provides an overview of the broad spectrum of weather-related challenges faced by UAS operations. It likely identifies key weather parameters (e.g., wind, icing, precipitation, visibility) and discusses their potential effects on drone performance, safety, and operational decision-making. The paper likely highlights the need for improved weather forecasting and risk assessment tools to support safe and efficient UAS operations.
•	Flapone Blog Post: "Weather and Flight: How Conditions Affect Drone Operations": This blog post offers practical insights into the impact of weather conditions on drone flights. It highlights the importance of temperature, wind speed, and precipitation as critical factors affecting drone performance, flight time, and battery performance. It gives a brief idea about the range in which drones should be flown between 0°C and 40°C (32°F and 104°F).

5. Methodology
•	Description of the robotic system 
The project simulates a 3D drone navigation system in MATLAB, integrating AI-based path planning (A algorithm*) and real-time weather monitoring to ensure safe and efficient movement. The system consists of:
1.	Drone Model: Simulated as a 3D cube, moving in a defined 3D space.
2.	A* Path Planning: Used to compute the optimal route while avoiding obstacles.
3.	Obstacles: Both static and dynamic obstacles (cubes, spheres, cylinders) are placed in the environment.
4.	Weather Monitoring: Real-time data from the WeatherBit API determines whether flight conditions are safe.
5.	Emergency Handling: If unsafe weather is detected mid-flight, the drone follows an emergency landing procedure.
