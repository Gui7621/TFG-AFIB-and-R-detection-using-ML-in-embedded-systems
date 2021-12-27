The project data shared in this repository has been divided into certain folders that involve different steps performed in TFG.

The first one (R Peak Detection Algorithm) has the Python file of the algorithm built along with the tests done to attest to its functionality.

The second folder (Schematic file in Fritzing) has the file to be edited in Fritzing with the schematic on the protoboard of the assembled prototype.

The "Jupyter Notebook Files" folder contains several Python files that show the entire development of the machine learning project, from working with the database including filtering and normalizing records to building and testing the neural model.

The folder "Model classifications for ECG data sampled in SimMan Simulator" contains the Excel files containing the results of the model classifications for the exams performed in SimMan.

The folder "ECG data sampled in SimMan Simulator" has some Excel files that contain RS and FA ECG records collected from SimMan, where some were used to test the model in Jupyter Notebook files.

The sixth folder (Link of the project with the final model in Edge Impulse) contains a txt file with the link to the public project in Edge Impulse that has the neural model used to make the inference in ESP32. There you can also check the model's training and test results, as well as its input data, which are precisely the individual heartbeats.

Finally, the "Program in C++" folder has the .ino file of the final program used to make the inference of the model in C++ in ESP32, including all steps of sampling the cardiac signal, data processing, obtaining the R peaks and the individual heartbeats to the final 10s model and exam scores.


