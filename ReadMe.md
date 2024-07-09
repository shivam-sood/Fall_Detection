The Website folder contains the website of the dashboard we used, as well as an Android app apk that can be installed which also contains the dashboard. The main dashboard is in index.html. The website has also been hosted at shivam-sood.github.io/

The Fall Data (New) directory contains the data we have collected for our project. With the detail_info.xlsx file containing the specific information on which file contains which data and how it was collected.

The Fall data (Old) directory is the previous version of collected data when we were working with only 2 categories, Fall and ADL. Now we work with 3 Categories which is reflected in Fall Data (New), which are Fall, ADL, Same-floor fall.

The SisFall_dataset directory contains a public dataset which we initially used to figure out which machine learning model works best in this scenario.

The src directory contains all our python code used to make our ML models.

The Old models subdirectory is for previous intermediate models we created. Can be mostly ignored, however they are still uploaded incase specific parts are needed to be used again later.

The final_sisfall_traditional.ipynb file contains our results of using traditional ML models like SVM, KNN, XGB. The results are present in the output cells of the jupyter notebook.

The final_sisfall_nn.ipynb file contains our results of using neural network ML models like CNN,MLP. The results are present in the output cells of the jupyter notebook.

From output of above two files, we decided to use CNN.

Final_full.ipynb contains the CNN model we trained on our own dataset. On running all cells, the model is saved in Model folder, also a model.h file is also saved in model folder, which contains tflite model which we can directly import into our arduino code, as shown in code in Arduino Folder.

Final_reduced.ipynb contains the CNN model we trained on our own dataset, but with less parameters, and smaller size to run on micro-controller. However we found that even full version can run on microcontroller. Just the reduced version takes 8 ms, while full version takes 30 ms. On running all cells, the model is saved in Model folder, also a model.h file is also saved in model folder, which contains tflite model which we can directly import into our arduino code, as shown in code in Arduino Folder.

The Arduino folder contains the project_esw_static folder which runs our CNN model on microcontroller(ESP32) on one predefined data point. Also has the aes_encryption file which encrypts any message using AES standard.
The project_esw folder runs our cnn model on data collected in real time by sensors, and last folder is for collecting and
sending vitals data to thingspeak.

We also have a video of the working of our project, as well as a presentation and report pdf.