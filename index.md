---
title: Robotics for Software Engineers 
subtitle: CS 4501 - Fall 2022
layout: page
---

 

{% include notification.html message="CS 3100 (or DS2) is a requirement for this course, no exceptions." %}


{% include notification.html message="This course is part of our ongoing effort to bridge the gap between **software engineering** and **robotics**.  If you are a *faculty* member interested in teaching a course like this, reach out to us as we have  supplementary material and hard-earned experiences that might be helpful. Thank you!  - Sebastian Elbaum." 
status="is-success" 
icon="fas fa-exclamation-triangle" %}

# Team

* Sebastian Elbaum - Instructor, selbaum at virginia  
* Trey Woodlief - Teaching Assistant, adw8dm at virginia
* Chris Morse - Teaching Assistant, drb6yv at virginia 
* Meriel Stein - Supporting instructor, meriel at virginia
* Carl Hildebrandt - Supporting instructor, hildebrandt.carl at virginia


# Goal and Scope

Developing software for robot systems is challenging as they must sense, actuate, and represent the physical world. Sensing the physical world is usually noisy,  actuating in and on the world is often inaccurate, and the knowledge and representation of the world is  incomplete and uncertain.  In this class we will explore software engineering approaches to cope with those challenges. You will learn to use domain-specific abstractions, architectures, libraries, and validation approaches and tools to safely perform robot activities like motion, navigation, perception, planning, and interaction.  The expectation is that this course will open up new career options in robotics for our students. 


# Class location and time
* Monday and Wednesday from 3:30PM to 4:45PM 
* Classes will be in person at Olsson Hall 011, with most lectures on Mondays and labs on Wednesdays

# Office Hours
* Sebastian Elbaum: Wednesday 4:45PM - 6:00PM
* Trey Woodlief:  TBD
* Chris Morse:   TBD

# Prerequisites
 CS 3100 (DSA2) or equivalent with a grade of C or above.


# Tentative Schedule

| Week | Monday                                 | Wednesday                                                             |
|:-----|:---------------------------------------| :---------------------------------------------------------------------|
|1     | ---                                    | Introduction                                                          |
|2     |Distinguishing Development Features     | Lab-1: Set up and Basic ROS                                           |
|3     |---Labor Day---                         | Lab-2: ROS processes, Communication, and Simulation Environment       |
|4     |Software Machinery + Q1                 | Lab-3: Types and Machines                                             |
|5     |Robot and World Types                   | Lab-4: Sensor Types and Handling Uncertainty and Noise                |
|6     |Abstractions for Perception  + Q2       | Lab-5: Perception through Image Analysis                              |
|7     |---UVA Break Day---                     | Invited Speaker                                                       |
|8     |Controlling your Robot                  | Lab-E: Robotics and Ethics                                            |
|9     |Tradeoffs when Making Plans + Q3        | Lab-6: Controlling and Testing Tobots                                 |
|10     |Localization and Navigation Stack       | Lab-7: Mapping and Motion Planning Software Stack                     |
|11    |Overloading and Transformations         | Lab-8: Pose Transformations                                           |
|12    |Advanced Robotics + Q4                  | Lab 9: Swarms and Safety                                              |
|13    |Project parameters                      | Invited Speaker                                                       |
|14    |Project check                           | Project consult                                                       |
|15    |Project Presentations and Demos         | Project Presentations and Demos                                       |
|16    |Taking stock                            | ---                                                                   |
 
# Course Policies

* Students must fully comply with all the provisions of the University’s Honor Code. Students are expected to work independently unless instructed otherwise. Offering and accepting solutions from others is a serious offense. All suspected violations will be forwarded to the Honor Committee, and you may, at the instructor's discretion, receive an immediate zero on that assignment and fail the course regardless of any action taken by the Honor Committee.
    * All graded labs, quizzes, and project must be pledged. 
    * You can discuss labs and project, but you cannot share code.
    * Do not exchange information during online quizzes.
* Labs can get full credit if returned within a week of the class when they were introduced. After a week, the labs get 50% credit. After two weeks the labs get 0 credit. 
* Depending on the class size and the TA availability, only a sample of the students will be graded on a weekly basis.  
* Students are responsible for all missed work. It is also the absentee's responsibility to get all missing notes or materials.
* If you anticipate any issues related to the format, materials, or requirements of this course, please meet with me outside of class so we can explore potential options. 
* If you are unsure if you require an accommodation or to learn more about their services, you may contact the SDAC at the number above or by visiting their website at [https://studenthealth.virginia.edu/sdac](https://studenthealth.virginia.edu/sdac).
* If you are struggling with violence or discrimination, I am ready to provide support and guide you towards the many resources available at the University of Virginia.
* If you need academic accommodation for a religious observance, please submit an email request to me as far in advance as possible. Note that accommodations do not relieve you of the responsibility for completion of any part of the coursework missed as the result of a religious observance.

# Tentative Grades Distribution
* ~9 Labs: 70 points
* 1 Project: 20 points 
* 1 Video: 2 points
* 4 Quizzes: 8 points 

# Letter Grade
* A+: [98,100],  A: [93, 98), A-: [90, 93) 
* B+: [87, 90),  B: [83, 87), B-: [80, 83) 
* C+: [77, 80), C-: [73, 77), C-: [70, 73)
* D+: [67, 70),  D: [63, 67), D-: [60, 63), F: [0,60)

# FAQ
1. **Is this course for me?**
This is a class for students who have no or limited experience in  robotics but are interested in learning more about how we develop systems that interact with the physical world. Note that the material and schedule is likely to be tweaked as the course evolves, so you need to be comfortable taking an exploratory class with us.
2. **What is this course NOT about?**
This class is not about AI, mechanical design, or electronic design. It is mainly about how to build software that will operate mobile robots in the physical world.
3. **What is the structure of the course?**
This class will include multiple development labs, a team project, and a handful of quizzes. 
4. **What robot will be used?** 
Drones. All in simulation. Our focus is on the software.