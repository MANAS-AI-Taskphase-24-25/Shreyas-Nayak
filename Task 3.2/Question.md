# Task 3.2 -  Decision Trees

## Description
Cybersecurity Threat analysis

### Overview
This dataset represents a crucial segment of network security monitoring - real-time analysis of network traffic data aimed at identifying potential cybersecurity threats.<br>
The data is composed of anonymized numerical features representing various network parameters,<br>
such as traffic volume, port numbers, and types of protocols used.<br>
The objective is to determine whether a particular network activity is benign or a potential security threat.<br>

### Content
Due to the sensitive nature of network security data, the information in this dataset has been thoroughly anonymized. <br>
This level of anonymity means that traditional feature engineering approaches may be less applicable,<br>
presenting an interesting challenge in handling high-dimensional, imbalanced data.<br>
<br>
The primary challenge in most cases is to to develop machine learning models that can effectively identify these rare threat events without being overwhelmed by the volume of non-threatening data .<br>
Performance should be evaluated using metrics like AUC, F1 Score, Matthews Correlation Coefficient (MCC), or  recall rate, with an emphasis on robust cross-validation techniques.<br>
<br>
Approach This dataset offers an opportunity to test the use of models like decision trees and understand the train-test split on complex data.<br>
